#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

/*
  Merged ESP32-S3 camera stream + mobile joystick controller.

  - Visit http://192.168.4.1/ after connecting to the ESP AP.
  - The MJPEG camera stream is shown as the page background.
  - Joystick commands are sent by the browser to:
      /joy?fb=<forward_back>&lr=<left_right>&en=<0_or_1>
  - Commands are written to STM using your original Agile joystick format:
      #P15=<raw_lr>,P8=<raw_fb>!\r\n
    Balance enable is sent separately as:
      #BAL=<0_or_1>!\r\n
  - Battery voltage is read back from STM UART telemetry:
      vbus=<voltage> V\r\n
  - UART pins match your joystick project: TX=43, RX=44.
*/
// ---------- WiFi AP settings ----------
static const char *AP_SSID = "Agile-Cam-Joystick";
static const char *AP_PASS = "12345678";  // Must be at least 8 chars for WPA2

// ---------- ESP32-S3-EYE camera pins ----------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// ---------- STM / Agile serial settings ----------
// These pins are from your joystick project. Change only if your wiring differs.
#define AGILE_SERIAL Serial1
static const int AGILE_UART_TX_PIN = 43;  // ESP TX -> STM RX
static const int AGILE_UART_RX_PIN = 44;  // ESP RX <- STM TX
static const uint32_t AGILE_UART_BAUD = 115200;
static const unsigned long JOY_SEND_PERIOD_MS = 100UL;

// ---------- Shared joystick state ----------
static volatile int g_fb_cmd = 0;      // -1000 ~ +1000, maps to P8
static volatile int g_lr_cmd = 0;      // -1000 ~ +1000, maps to P15
static volatile int g_enable = 0;      // 0 / 1
static volatile float g_bus_voltage_v = 0.0f;
static volatile int g_bus_voltage_valid = 0;
static volatile unsigned long g_bus_voltage_ms = 0;
static unsigned long g_last_uart_ms = 0;
static char g_agile_rx_line[64];
static size_t g_agile_rx_line_len = 0;

httpd_handle_t main_httpd = nullptr;
httpd_handle_t stream_httpd = nullptr;

#define PART_BOUNDARY "123456789000000000000987654321"

static int clamp_int(int x, int min_v, int max_v) {
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static void sendJoyFrame() {
  char buf[48];
  const int p8_raw  = clamp_int(g_fb_cmd, -1000, 1000);
  const int p15_raw = clamp_int(g_lr_cmd, -1000, 1000);
  snprintf(buf, sizeof(buf), "#P15=%d,P8=%d!\r\n", p15_raw, p8_raw);
  AGILE_SERIAL.print(buf);
}

static void sendBalanceFrame() {
  char buf[20];
  const int en = (g_enable != 0) ? 1 : 0;
  snprintf(buf, sizeof(buf), "#BAL=%d!\r\n", en);
  AGILE_SERIAL.print(buf);
}

static void handleAgileLine(const char *line) {
  if (strncmp(line, "vbus=", 5) != 0) {
    return;
  }

  char *end = nullptr;
  const float vbus = strtof(line + 5, &end);
  if (end == line + 5) {
    return;
  }

  if ((vbus >= 0.0f) && (vbus < 100.0f)) {
    g_bus_voltage_v = vbus;
    g_bus_voltage_valid = 1;
    g_bus_voltage_ms = millis();
  }
}

static void serviceAgileRx() {
  while (AGILE_SERIAL.available() > 0) {
    const char c = (char)AGILE_SERIAL.read();

    if ((c == '\r') || (c == '\n')) {
      if (g_agile_rx_line_len > 0) {
        g_agile_rx_line[g_agile_rx_line_len] = '\0';
        handleAgileLine(g_agile_rx_line);
        g_agile_rx_line_len = 0;
      }
      continue;
    }

    if (g_agile_rx_line_len < (sizeof(g_agile_rx_line) - 1U)) {
      g_agile_rx_line[g_agile_rx_line_len++] = c;
    } else {
      g_agile_rx_line_len = 0;
    }
  }
}
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Agile Camera Joystick</title>
  <style>
    html, body { width: 100%; min-height: 100%; }
    body {
      margin: 0;
      background: #111;
      color: #fff;
      font-family: Arial, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: flex-start;
      min-height: 100vh;
      padding: 24px 16px 40px;
      box-sizing: border-box;
      overflow: hidden;
      touch-action: none;
    }
    #cameraBg {
      position: fixed;
      inset: 0;
      width: 100vw;
      height: 100vh;
      object-fit: cover;
      z-index: 0;
      background: #000;
    }
    #shade {
      position: fixed;
      inset: 0;
      background: rgba(0,0,0,0.30);
      z-index: 1;
      pointer-events: none;
    }
    h2 { margin: 8px 0 12px; }
    .panel {
      position: relative;
      z-index: 2;
      width: 100%;
      max-width: 420px;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 16px;
    }
    .row {
      width: 100%;
      display: flex;
      gap: 12px;
      justify-content: center;
      flex-wrap: wrap;
    }
    button {
      border: 0;
      border-radius: 12px;
      padding: 14px 18px;
      font-size: 18px;
      min-width: 140px;
    }
    .on  { background: #1f8f4e; color: white; }
    .off { background: #555; color: white; }
    .stop { background: #c0392b; color: white; }
    #joy {
      position: relative;
      width: 280px;
      height: 280px;
      border-radius: 50%;
      background: #222;
      border: 3px solid #444;
      touch-action: none;
      user-select: none;
    }
    #stick {
      position: absolute;
      width: 96px;
      height: 96px;
      left: 92px;
      top: 92px;
      border-radius: 50%;
      background: #4da3ff;
      opacity: 0.95;
      box-shadow: 0 0 18px rgba(77,163,255,0.45);
    }
    .status {
      width: 100%;
      max-width: 420px;
      background: rgba(26,26,26,0.72);
      backdrop-filter: blur(4px);
      border-radius: 14px;
      padding: 14px 16px;
      box-sizing: border-box;
      line-height: 1.8;
      font-size: 18px;
    }
    .hint {
      opacity: 0.8;
      font-size: 14px;
      text-align: center;
      line-height: 1.6;
    }
  </style>
</head>
<body>
  <img id="cameraBg" alt="camera stream">
  <div id="shade"></div>
  <div class="panel">
    <h2>Agile Camera Joystick</h2>

    <div class="row">
      <button id="enableBtn" class="off">Balance OFF</button>
      <button id="stopBtn" class="stop">CENTER / STOP</button>
    </div>

    <div id="joy">
      <div id="stick"></div>
    </div>

    <div class="status">
      <div>Battery: <span id="vbusText">--.- V</span></div>
      <div>P8 raw / FB: <span id="fbText">0</span></div>
      <div>P15 raw / LR: <span id="lrText">0</span></div>
      <div>Enable: <span id="enText">0</span></div>
      <div>TX: <span id="cmdText">#P15=0,P8=0!</span></div>
    </div>

    <div class="hint">
      Push up/down changes P8 raw, left/right changes P15 raw.<br>
      ESP32 sends: #P15=&lt;LR&gt;,P8=&lt;FB&gt;! every 100 ms.
    </div>
  </div>

<script>
const cameraBg = document.getElementById('cameraBg');
cameraBg.src = location.protocol + "//" + location.hostname + ":81/stream";
const joy = document.getElementById('joy');
const stick = document.getElementById('stick');
const fbText = document.getElementById('fbText');
const lrText = document.getElementById('lrText');
const enText = document.getElementById('enText');
const cmdText = document.getElementById('cmdText');
const vbusText = document.getElementById('vbusText');
const enableBtn = document.getElementById('enableBtn');
const stopBtn = document.getElementById('stopBtn');

const size = 280;
const stickSize = 96;
const center = size / 2;
const radius = 92;
let dragging = false;
let fb = 0;
let lr = 0;
let en = 0;

function updateStick(dx, dy) {
  stick.style.left = `${center - stickSize / 2 + dx}px`;
  stick.style.top  = `${center - stickSize / 2 + dy}px`;
}

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function sendJoy() {
  fbText.textContent = fb;
  lrText.textContent = lr;
  enText.textContent = en;
  cmdText.textContent = `#P15=${lr},P8=${fb}!`;
  fetch(`/joy?fb=${fb}&lr=${lr}&en=${en}`).catch(() => {});
}

function pollStatus() {
  fetch('/status')
    .then((res) => res.json())
    .then((data) => {
      const hasFreshVoltage =
        typeof data.vbus === 'number' &&
        typeof data.vbusAgeMs === 'number' &&
        data.vbusAgeMs < 2000;
      vbusText.textContent = hasFreshVoltage ? `${data.vbus.toFixed(2)} V` : '--.- V';
    })
    .catch(() => {
      vbusText.textContent = '--.- V';
    });
}

function resetJoy(sendNow = true) {
  fb = 0;
  lr = 0;
  updateStick(0, 0);
  if (sendNow) sendJoy();
}

function handlePointer(clientX, clientY) {
  const rect = joy.getBoundingClientRect();
  let dx = clientX - (rect.left + rect.width / 2);
  let dy = clientY - (rect.top + rect.height / 2);

  const dist = Math.hypot(dx, dy);
  if (dist > radius) {
    dx = dx / dist * radius;
    dy = dy / dist * radius;
  }

  lr = clamp(Math.round(dx / radius * 1000), -1000, 1000);
  fb = clamp(Math.round(-dy / radius * 1000), -1000, 1000);

  updateStick(dx, dy);
  sendJoy();
}

joy.addEventListener('pointerdown', (e) => {
  dragging = true;
  joy.setPointerCapture(e.pointerId);
  handlePointer(e.clientX, e.clientY);
});

joy.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  handlePointer(e.clientX, e.clientY);
});

function releaseJoy() {
  dragging = false;
  resetJoy(true);
}

joy.addEventListener('pointerup', releaseJoy);
joy.addEventListener('pointercancel', releaseJoy);
joy.addEventListener('pointerleave', () => {
  if (dragging) releaseJoy();
});

enableBtn.addEventListener('click', () => {
  en = en ? 0 : 1;
  enableBtn.textContent = en ? 'Balance ON' : 'Balance OFF';
  enableBtn.className = en ? 'on' : 'off';
  sendJoy();
});

stopBtn.addEventListener('click', () => {
  resetJoy(false);
  en = 0;
  enableBtn.textContent = 'Balance OFF';
  enableBtn.className = 'off';
  sendJoy();
});

updateStick(0, 0);
setInterval(sendJoy, 100);
setInterval(pollStatus, 250);
pollStatus();
</script>
</body>
</html>
)rawliteral";

static int get_query_int(httpd_req_t *req, const char *key, int fallback) {
  char query[128];
  char value[24];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return fallback;
  if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return fallback;
  return atoi(value);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t joy_handler(httpd_req_t *req) {
  const int old_enable = g_enable;

  g_fb_cmd = clamp_int(get_query_int(req, "fb", g_fb_cmd), -1000, 1000);
  g_lr_cmd = clamp_int(get_query_int(req, "lr", g_lr_cmd), -1000, 1000);
  g_enable = get_query_int(req, "en", g_enable) != 0 ? 1 : 0;

  if (g_enable != old_enable) {
    sendBalanceFrame();
  }

  sendJoyFrame();
  g_last_uart_ms = millis();

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "ok");
}

static esp_err_t status_handler(httpd_req_t *req) {
  char json[160];
  const int voltage_valid = g_bus_voltage_valid;

  if (voltage_valid) {
    const float vbus = g_bus_voltage_v;
    const unsigned long age_ms = millis() - g_bus_voltage_ms;
    snprintf(json, sizeof(json),
             "{\"fb\":%d,\"lr\":%d,\"en\":%d,\"vbus\":%.3f,\"vbusAgeMs\":%lu}",
             g_fb_cmd, g_lr_cmd, g_enable, vbus, age_ms);
  } else {
    snprintf(json, sizeof(json),
             "{\"fb\":%d,\"lr\":%d,\"en\":%d,\"vbus\":null,\"vbusAgeMs\":null}",
             g_fb_cmd, g_lr_cmd, g_enable);
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = nullptr;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      esp_camera_fb_return(fb);
      res = ESP_FAIL;
      break;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    fb = nullptr;

    if (res != ESP_OK) break;
    delay(10);
  }

  return res;
}

static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, 1);   // needed for ESP32-S3-EYE orientation
  }

  return true;
}

static void start_servers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = nullptr };
  httpd_uri_t joy_uri = { .uri = "/joy", .method = HTTP_GET, .handler = joy_handler, .user_ctx = nullptr };
  httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = nullptr };

  if (httpd_start(&main_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(main_httpd, &index_uri);
    httpd_register_uri_handler(main_httpd, &joy_uri);
    httpd_register_uri_handler(main_httpd, &status_uri);
  }

  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = 32769;
  stream_config.max_uri_handlers = 2;

  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = nullptr };

  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(500);
  Serial.println("\nBooting merged camera + joystick firmware");

  if (!init_camera()) {
    Serial.println("Camera failed. Web server will not start.");
    return;
  }

  AGILE_SERIAL.begin(AGILE_UART_BAUD, SERIAL_8N1, AGILE_UART_RX_PIN, AGILE_UART_TX_PIN);
  Serial.printf("Agile UART started at %lu baud on RX=%d TX=%d\n", AGILE_UART_BAUD, AGILE_UART_RX_PIN, AGILE_UART_TX_PIN);
  sendBalanceFrame();
  sendJoyFrame();
  g_last_uart_ms = millis();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(300);

  Serial.printf("AP start: %s\n", ap_ok ? "OK" : "FAILED");
  Serial.printf("SSID: %s\n", AP_SSID);
  Serial.printf("PASS: %s\n", AP_PASS);
  Serial.printf("Open: http://%s/\n", WiFi.softAPIP().toString().c_str());

  start_servers();
  Serial.println("Main UI:    http://192.168.4.1/");
  Serial.println("Raw stream: http://192.168.4.1:81/stream");
}

void loop() {
  serviceAgileRx();

  if (millis() - g_last_uart_ms >= JOY_SEND_PERIOD_MS) {
    sendJoyFrame();
    g_last_uart_ms = millis();
  }
  delay(5);
}
