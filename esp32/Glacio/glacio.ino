#include "WiFi.h"
#include "PubSubClient.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// WiFi
const char* ssid = "The Atharva Robotics Club";
const char* password = "9211712712";

// MQTT
const char* broker_ip = "192.168.0.132";
const int broker_port = 1883;

// --- CONFIGURATION FOR GLACIO (ID 4) ---
#define CLIENT_ID "GlacioBot_4"
#define CMD_TOPIC "hb/cmd/4"
#define IR_TOPIC  "hb/sensor/4/ir"

// Pins
#define WHEEL1_PIN 18
#define WHEEL2_PIN 2
#define WHEEL3_PIN 21
#define BASE_PIN 25
#define ELBOW_PIN 27
#define SOLENOID_PIN 23
#define IR_SENSOR_PIN 15

Servo wheel1, wheel2, wheel3, baseServo, elbowServo;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

float cmd_m1 = 0.0, cmd_m2 = 0.0, cmd_m3 = 0.0;
float cmd_base = 120.0, cmd_elbow = 130.0;
int solenoid_state = 0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) return;

  solenoid_state = (int)doc["solenoid"];
  cmd_m1 = doc["m1"];
  cmd_m2 = doc["m2"];
  cmd_m3 = doc["m3"];
  cmd_base = doc["base"];
  cmd_elbow = doc["elbow"];
}

void reconnect() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(CLIENT_ID)) {
      mqttClient.subscribe(CMD_TOPIC);
    } else {
      delay(5000);
    }
  }
}

void setup() {
  wheel1.attach(WHEEL1_PIN); wheel2.attach(WHEEL2_PIN); wheel3.attach(WHEEL3_PIN);
  baseServo.attach(BASE_PIN); elbowServo.attach(ELBOW_PIN);
  
  pinMode(SOLENOID_PIN, OUTPUT); pinMode(IR_SENSOR_PIN, INPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  
  wheel1.write(90); wheel2.write(90); wheel3.write(90);
  baseServo.write(120); elbowServo.write(130);
  
  setup_wifi();
  mqttClient.setServer(broker_ip, broker_port);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  if (!mqttClient.connected()) reconnect();
  
  bool ir_state = !digitalRead(IR_SENSOR_PIN);
  mqttClient.publish(IR_TOPIC, ir_state ? "CRATE_DETECTED" : "NO_CRATE");
  mqttClient.loop();
  
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 20) {
    wheel1.write(constrain(90 + cmd_m1 * 40, 50, 130));
    wheel2.write(constrain(90 + cmd_m2 * 40, 50, 130));
    wheel3.write(constrain(90 + cmd_m3 * 40, 50, 130));
    
    baseServo.write(constrain(cmd_base, 0, 180));
    elbowServo.write(constrain(cmd_elbow, 0, 180));
    
    digitalWrite(SOLENOID_PIN, solenoid_state ? HIGH : LOW);
    lastUpdate = millis();
  }
}