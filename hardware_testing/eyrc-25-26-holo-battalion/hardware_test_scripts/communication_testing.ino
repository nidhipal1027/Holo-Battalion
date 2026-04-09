#include "WiFi.h"
#include "PubSubClient.h"

const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

static inline void setup_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Waiting for hotspot..."));
        delay(500);
    }
    Serial.println(F("WiFi connected."));
    Serial.println(WiFi.localIP());
}