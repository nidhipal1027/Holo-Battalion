#include <Arduino.h>

const int SOL_PIN = 23;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 1000;    // 1 kHz
const int PWM_RES = 8;        // 8-bit resolution -> values 0..255

const int PULLIN_MS = 300;    // full power time to pull-in (ms)
const int HOLD_MS = 2000;     // how long to hold for test (ms)
const int OFF_MS = 1000;      // off time (ms)
const int HOLD_DUTY = 120;    // 0..255. Lower = less heat. Increase if it drops.

void setup() {
  Serial.begin(115200);
  
  // Setup PWM channel (new API for ESP32 core 3.x+)
  ledcAttach(SOL_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(SOL_PIN, 0); // ensure off
  
  Serial.println("Solenoid PWM hold test starting...");
}

void loop() {
  Serial.println("PULL-IN: FULL");
  ledcWrite(SOL_PIN, 255); // full power
  delay(PULLIN_MS);

  Serial.print("HOLD at duty ");
  Serial.println(HOLD_DUTY);
  ledcWrite(SOL_PIN, HOLD_DUTY); // reduced hold duty
  delay(HOLD_MS);

  Serial.println("OFF");
  ledcWrite(SOL_PIN, 0);
  delay(OFF_MS);
}