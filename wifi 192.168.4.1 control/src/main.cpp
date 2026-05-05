#include "main.h"

HardwareSerial AgileSerial(1);
WebServer server(80);

const char* ap_ssid = "Joystick2";
const char* ap_password = "12345678";

static volatile int g_fb_cmd = 0;      // -1000 ~ +1000
static volatile int g_lr_cmd = 0;      // -1000 ~ +1000
static volatile int g_enable = 0;      // 0 / 1
static unsigned long g_last_uart_ms = 0;
static const unsigned long JOY_SEND_PERIOD_MS = 100UL;

static int clamp_int(int x, int min_v, int max_v)
{
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static void sendJoyFrame()
{
  char buf[48];

  /*
   * Agile 2.6.0 combined joystick command format:
   *   #P15=<raw_lr>,P8=<raw_fb>!
   *
   * Firmware-side mapping in Agile 2.6.0:
   *   P15 raw -1000..1000 -> yaw rate -180..180 deg/s
   *   P8  raw -1000..1000 -> speed    -60..60 rpm
   *
   * Keep CRLF after '!' so the same frame works in both VOFA-style
   * hash-command mode and plain CLI line mode.
   */
  const int p8_raw  = clamp_int(g_fb_cmd, -1000, 1000);  // old fb -> P8
  const int p15_raw = clamp_int(g_lr_cmd, -1000, 1000);  // old lr -> P15

  snprintf(buf, sizeof(buf), "#P15=%d,P8=%d!\r\n", p15_raw, p8_raw);
  AgileSerial.print(buf);
}

static void sendBalanceFrame()
{
  char buf[20];
  const int en = (g_enable != 0) ? 1 : 0;

  snprintf(buf, sizeof(buf), "#BAL=%d!\r\n", en);
  AgileSerial.print(buf);
}

static void handleRoot()
{
  static const char page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Agile ESP32 Joystick</title>
  <style>
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
    }
    h2 { margin: 8px 0 12px; }
    .panel {
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
      background: #1a1a1a;
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
  <div class="panel">
    <h2>Agile ESP32 Joystick</h2>

    <div class="row">
      <button id="enableBtn" class="off">Balance OFF</button>
      <button id="stopBtn" class="stop">CENTER / STOP</button>
    </div>

    <div id="joy">
      <div id="stick"></div>
    </div>

    <div class="status">
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
const joy = document.getElementById('joy');
const stick = document.getElementById('stick');
const fbText = document.getElementById('fbText');
const lrText = document.getElementById('lrText');
const enText = document.getElementById('enText');
const cmdText = document.getElementById('cmdText');
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
</script>
</body>
</html>
)rawliteral";

  server.send_P(200, "text/html", page);
}

static void handleJoy()
{
  const int old_enable = g_enable;

  if (server.hasArg("fb")) g_fb_cmd = clamp_int(server.arg("fb").toInt(), -1000, 1000);
  if (server.hasArg("lr")) g_lr_cmd = clamp_int(server.arg("lr").toInt(), -1000, 1000);
  if (server.hasArg("en")) g_enable = (server.arg("en").toInt() != 0) ? 1 : 0;

  if (g_enable != old_enable) {
    sendBalanceFrame();
  }

  sendJoyFrame();
  g_last_uart_ms = millis();
  server.send(200, "text/plain", "ok");
}

void setup()
{
  Serial.begin(115200);
  AgileSerial.begin(AGILE_UART_BAUD, SERIAL_8N1, AGILE_UART_RX_PIN, AGILE_UART_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);

  server.on("/", handleRoot);
  server.on("/joy", handleJoy);
  server.begin();

  delay(200);
  sendBalanceFrame();
  sendJoyFrame();
}

void loop()
{
  server.handleClient();

  if (millis() - g_last_uart_ms >= JOY_SEND_PERIOD_MS) {
    sendJoyFrame();
    g_last_uart_ms = millis();
  }
}
