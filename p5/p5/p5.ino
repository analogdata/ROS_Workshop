#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ===== WiFi =====
const char* WIFI_SSID = "OnePlusRajath";
const char* WIFI_PASS = "rajathkumarks";

// ===== MQTT Broker (Public) =====
const char* MQTT_HOST = "test.mosquitto.org";
const int   MQTT_PORT = 1883;

// Topic
const char* TOPIC_SUB = "robot/control";

// ===== Motor Pins (same as before) =====
int M11 = D0;
int M12 = D1;
int M21 = D2;
int M22 = D3;
int En  = D4;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===== Motor Control =====
void stopMotor() {
  digitalWrite(M11, LOW);
  digitalWrite(M12, LOW);
  digitalWrite(M21, LOW);
  digitalWrite(M22, LOW);
}

void fwd() {
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M22, HIGH);
  digitalWrite(M21, LOW);
}

void rev() {
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);
  digitalWrite(M22, LOW);
  digitalWrite(M21, HIGH);
}

void ryt() {
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M22, LOW);
  digitalWrite(M21, HIGH);
}

void lft() {
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);
  digitalWrite(M22, HIGH);
  digitalWrite(M21, LOW);
}

// ===== Execute command text =====
void executeCommand(const String& cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "FORWARD" || cmd == "FWD") {
    fwd();
    Serial.println("ACTION: FORWARD (motors set)");
  }
  else if (cmd == "REVERSE" || cmd == "REV") {
    rev();
    Serial.println("ACTION: REVERSE (motors set)");
  }
  else if (cmd == "LEFT" || cmd == "LFT") {
    lft();
    Serial.println("ACTION: LEFT (motors set)");
  }
  else if (cmd == "RIGHT" || cmd == "RYT") {
    ryt();
    Serial.println("ACTION: RIGHT (motors set)");
  }
  else if (cmd == "STOP") {
    stopMotor();
    Serial.println("ACTION: STOP (motors off)");
  }
  else {
    stopMotor(); // safety
    Serial.print("ACTION: UNKNOWN -> STOP. Received: ");
    Serial.println(cmd);
  }
}

// ===== MQTT Callback =====
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Payload: ");
  Serial.println(msg);
  Serial.println("----------------------");

  executeCommand(msg);
  Serial.println("======================");
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
    Serial.print("Connecting to MQTT... ");

    String clientId = "esp8266-robot-";
    clientId += String(ESP.getChipId(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");

      mqtt.subscribe(TOPIC_SUB, 1);
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

  // Motor pins
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);

  pinMode(En, OUTPUT);
  digitalWrite(En, HIGH);  // enable motor driver
  stopMotor();             // safe state

  connectWiFi();
  connectMQTT();

  Serial.println("READY: Send FORWARD/REVERSE/LEFT/RIGHT/STOP to robot/control");
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
