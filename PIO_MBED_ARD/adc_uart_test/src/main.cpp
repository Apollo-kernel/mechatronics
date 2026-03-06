#include <Arduino.h>

// Analog input pin definitions for potentiometer and light dependent resistor
#define POTENTIOMETER_PIN A0
#define LDR_PIN A1

// Digital output pin definitions for LED indicators
#define LED_POT_PIN D2
#define LED_LDR_PIN D3

// Timestamp variables to track the last time each LED state was toggled
unsigned long lastPotToggle = 0;
unsigned long lastLdrToggle = 0;

// State variables to track whether each LED is currently on or off
bool potState = false;
bool ldrState = false;

void setup() {
  // Initialize UART serial communication at 9600 baud rate for debugging
  Serial.begin(9600);
  
  // Configure analog-to-digital converter resolution to 12-bit (0-4095 range)
  analogReadResolution(12);
  
  // Set LED pins as output mode to allow control of the LEDs
  pinMode(LED_POT_PIN, OUTPUT);
  pinMode(LED_LDR_PIN, OUTPUT);
}

void loop() {
  // Read the analog voltage from potentiometer (returns 0-4095 for 12-bit resolution)
  int potValue = analogRead(POTENTIOMETER_PIN);
  
  // Read the analog voltage from light dependent resistor (returns 0-4095 for 12-bit resolution)
  int ldrValue = analogRead(LDR_PIN);
  
  // Output sensor readings to serial terminal for real-time monitoring
  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print(", LDR: ");
  Serial.println(ldrValue);
  
  // Get the current system time in milliseconds since startup
  unsigned long currentMillis = millis();
  
  // Control potentiometer LED blinking using non-blocking timing method
  int potDelay = (potValue > 1900) ? 1000 : 100;  // 1Hz (slow) if above 1900, else 10Hz (fast)
  if (currentMillis - lastPotToggle >= potDelay / 2) {
    // Toggle the LED state between on and off
    potState = !potState;
    digitalWrite(LED_POT_PIN, potState ? HIGH : LOW);
    // Update the timestamp for the next toggle
    lastPotToggle = currentMillis;
  }
  
  // Control LDR LED blinking using non-blocking timing method
  int ldrDelay = (ldrValue > 900) ? 1000 : 100;  // 1Hz (slow) if above 900, else 10Hz (fast)
  if (currentMillis - lastLdrToggle >= ldrDelay / 2) {
    // Toggle the LED state between on and off
    ldrState = !ldrState;
    digitalWrite(LED_LDR_PIN, ldrState ? HIGH : LOW);
    // Update the timestamp for the next toggle
    lastLdrToggle = currentMillis;
  }
}