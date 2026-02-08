#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ===== WiFi =====
const char* WIFI_SSID = "OnePlusRajath";
const char* WIFI_PASS = "rajathkumarks";

// ===== MQTT Broker =====
// CHANGE this to your broker IP
const char* MQTT_HOST = "test.mosquitto.org";
const int   MQTT_PORT = 1883;

// Topic
const char* TOPIC_SUB = "robot/control";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===== MQTT Callback =====
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  Serial.print("Payload: ");

  // Convert payload to string
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.println(msg);
  Serial.println("----------------------");
}

// ===== WiFi Connect =====
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT Connect =====
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT...");

    String clientId = "esp8266-sub-";
    clientId += String(ESP.getChipId(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");

      mqtt.subscribe(TOPIC_SUB);
      Serial.print("Subscribed to: ");
      Serial.println(TOPIC_SUB);

    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqtt.connected()) {
    connectMQTT();
  }

  mqtt.loop();
}
