#include <Arduino.h>
#define IR_PIN 13

void setup() {
    Serial.begin(115200);
    pinMode(IR_PIN, INPUT_PULLUP);
    Serial.println("IR Sensor Test Started");
}

void loop() {
    int state = digitalRead(IR_PIN);
    if (state == LOW) {
        Serial.println("Object Detected!");
    } else {
        Serial.println("Nothing here...");
    }
    delay(200);
}	