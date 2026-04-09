/*
# * Team Id:          eYRC#1832
# * Author List:      Nidhi pal, Seeya kokam, Kashisa padhy
# * Filename:         low_level_code.cpp
# * Theme:            Holo Battalion
* Functions:        setup_wifi, mqttCallback, reconnect, setup, loop
* Global Variables: ssid, password, broker_ip, broker_port, wheel1, wheel2, wheel3, baseServo, elbowServo, espClient, mqttClient, cmd_m1, cmd_m2, cmd_m3, cmd_base, cmd_elbow, solenoid_state
*/

#include "WiFi.h"
#include "PubSubClient.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// Variable Name: ssid, password
// Description: Network credentials for connecting the ESP32 to the local WiFi.
const char* ssid = "The Atharva Robotics Club";
const char* password = "9211712712";

// Variable Name: broker_ip, broker_port
// Description: IP address and port of the MQTT broker running on the main ROS2 network.
const char* broker_ip = "192.168.0.132";
const int broker_port = 1883;

#define CLIENT_ID "CrystalBot_0"
#define CMD_TOPIC "hb/cmd/0"
#define IR_TOPIC "hb/sensor/0/ir"

// Hardware Pin Definitions
#define WHEEL1_PIN 18
#define WHEEL2_PIN 2
#define WHEEL3_PIN 21
#define BASE_PIN 25
#define ELBOW_PIN 27
#define SOLENOID_PIN 23
#define IR_SENSOR_PIN 15

// Variable Name: wheel1, wheel2, wheel3, baseServo, elbowServo
// Description: Servo objects mapped to the continuous rotation wheels and positional arm joints.
Servo wheel1, wheel2, wheel3, baseServo, elbowServo;

// Variable Name: espClient, mqttClient
// Description: Network client instances for handling the TCP/IP connection and the MQTT protocol stack.
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Variable Name: cmd_m1, cmd_m2, cmd_m3
// Description: Float variables holding the target velocity commands for the three omni-wheels. Expected range is roughly -1.0 to 1.0.
float cmd_m1 = 0.0, cmd_m2 = 0.0, cmd_m3 = 0.0;

// Variable Name: cmd_base, cmd_elbow
// Description: Float variables holding the target angles for the robotic arm. Expected range is 0.0 to 180.0.
float cmd_base = 120.0, cmd_elbow = 130.0;

// Variable Name: solenoid_state
// Description: Integer flag to control the electromagnet/gripper. 0 represents OFF (detach), 1 represents ON (attach).
int solenoid_state = 0;

/*
* Function Name: setup_wifi
* Input:         None
* Output:        None
* Logic:         Initializes the WiFi radio, connects to the specified SSID, and blocks execution while printing dots until a connection is established. Prints the assigned local IP.
* Example Call:  setup_wifi();
*/
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("WiFi: ");
  // Loop until the ESP32 successfully connects to the router
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
  }
  Serial.println("\nWiFi OK: " + WiFi.localIP().toString());
}

/*
* Function Name: mqttCallback
* Input:         topic -> char array containing the MQTT topic string, payload -> byte array containing the message data, length -> integer length of the payload
* Output:        None
* Logic:         Triggered whenever an MQTT message arrives. Deserializes the JSON payload to update the global motor, arm, and solenoid command variables.
* Example Call:  Called automatically by the PubSubClient library.
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Null-terminate the incoming payload to safely treat it as a string
  payload[length] = '\0';
  String message = String((char*)payload);
  
  Serial.println("\n📨 MQTT RECEIVED:");
  Serial.print("Topic: "); Serial.println(topic);
  Serial.print("Payload: "); Serial.println(message);

  // Allocate a JSON document and attempt to parse the incoming string
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON ERROR: "); Serial.println(error.c_str());
    return;
  }

  // Update hardware states from the parsed JSON data
  solenoid_state = (int)doc["solenoid"];  
  cmd_m1 = doc["m1"];
  cmd_m2 = doc["m2"];
  cmd_m3 = doc["m3"];
  cmd_base = doc["base"];
  cmd_elbow = doc["elbow"];

  // Print confirmation of the updated states to the serial monitor for debugging
  Serial.println(" COMMANDS UPDATED:");
  Serial.printf("m1:%.2f m2:%.2f m3:%.2f\n", cmd_m1, cmd_m2, cmd_m3);
  Serial.printf("base:%.1f elbow:%.1f\n", cmd_base, cmd_elbow);
  Serial.printf("SOLENOID STATE: %d (PIN: %d)\n", solenoid_state, digitalRead(SOLENOID_PIN));
}

/*
* Function Name: reconnect
* Input:         None
* Output:        None
* Logic:         Checks if the MQTT connection is alive. If not, blocks execution and continuously attempts to reconnect to the broker using the designated Client ID. Subscribes to the command topic upon success.
* Example Call:  reconnect();
*/
void reconnect() {
  // Loop until we're reconnected to the MQTT broker
  while (!mqttClient.connected()) {
    Serial.print("MQTT: ");
    if (mqttClient.connect(CLIENT_ID)) {
      mqttClient.subscribe(CMD_TOPIC);
      Serial.println("Connected!");
    } else {
      // Wait 5 seconds before retrying to prevent network spam
      delay(5000);
    }
  }
}

/*
* Function Name: setup
* Input:         None
* Output:        None
* Logic:         Standard Arduino initialization function. Configures serial baud rate, attaches servos to pins, sets I/O modes, initializes resting positions, and starts network connections.
* Example Call:  Called automatically by the microcontroller on boot.
*/
void setup() {
  Serial.begin(115200);
  
  // Attach motors and servos to their respective GPIO pins
  wheel1.attach(WHEEL1_PIN); 
  wheel2.attach(WHEEL2_PIN); 
  wheel3.attach(WHEEL3_PIN);
  baseServo.attach(BASE_PIN); 
  elbowServo.attach(ELBOW_PIN);
  
  pinMode(SOLENOID_PIN, OUTPUT); 
  pinMode(IR_SENSOR_PIN, INPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  
  // Write default resting positions to the hardware
  wheel1.write(90); 
  wheel2.write(90); 
  wheel3.write(90);
  baseServo.write(100); 
  elbowServo.write(160);
  
  setup_wifi();
  mqttClient.setServer(broker_ip, broker_port);
  mqttClient.setCallback(mqttCallback);
}

/*
* Function Name: loop
* Input:         None
* Output:        None
* Logic:         Standard Arduino main loop. Maintains network connection, reads and publishes the IR sensor state over MQTT, and applies the cached motor commands to the physical pins every 20ms.
* Example Call:  Called continuously by the microcontroller after setup().
*/
void loop() {
  if (!mqttClient.connected()) reconnect();
  
  // Read the IR sensor (inverted logic depending on hardware pull-up/pull-down)
  bool ir_state = !digitalRead(IR_SENSOR_PIN);
  mqttClient.publish(IR_TOPIC, ir_state ? "CRATE_DETECTED" : "NO_CRATE");
  mqttClient.loop();
  
  static unsigned long lastUpdate = 0;
  // Apply hardware updates at a fixed interval of 20 milliseconds (50Hz)
  if (millis() - lastUpdate > 20) {
    
    // Map normalized speed commands to continuous rotation servo PWM signals (90 is stop)
    wheel1.write(constrain(90 + cmd_m1 * 40, 50, 130));
    wheel2.write(constrain(90 + cmd_m2 * 40, 50, 130));
    wheel3.write(constrain(90 + cmd_m3 * 40, 50, 130));
    
    // Map arm position commands directly to servo angles, constrained for safety
    baseServo.write(constrain(cmd_base, 0, 180));
    elbowServo.write(constrain(cmd_elbow, 0, 180));
    
    // Energize or de-energize the electromagnet based on the commanded state
    digitalWrite(SOLENOID_PIN, solenoid_state ? HIGH : LOW);
    
    lastUpdate = millis();
  }
}