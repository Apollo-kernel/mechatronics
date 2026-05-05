#ifndef _MAIN_H_
#define _MAIN_H_

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

#define AGILE_UART_BAUD    115200

// ESP32-S3 UART1 wiring for the STM32 joystick / telemetry link:
//   STM32 USART1_TX (PA9)  -> ESP32 RX (GPIO44)
//   STM32 USART1_RX (PA10) <- ESP32 TX (GPIO43)
//   GND must be shared between both boards.
#define AGILE_UART_TX_PIN  43
#define AGILE_UART_RX_PIN  44

#endif
