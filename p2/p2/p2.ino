#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "esp8266wifi";
const char* password = "12345678";

ESP8266WebServer server(80);

#define LED_PIN 16  // D0 = GPIO16

// HTML page
String webpage = R"====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP8266 LED Control</title>
</head>
<body style="text-align:center; font-family:Arial;">
  <h2>ESP8266 LED Control</h2>
  <p>
    <a href="/on"><button style="padding:15px 30px; font-size:20px;">LED ON</button></a>
  </p>
  <p>
    <a href="/off"><button style="padding:15px 30px; font-size:20px;">LED OFF</button></a>
  </p>
</body>
</html>
)====";

void handleRoot() {
  server.send(200, "text/html", webpage);
}

void handleOn() {
  digitalWrite(LED_PIN, HIGH);
  server.send(200, "text/html", webpage);
}

void handleOff() {
  digitalWrite(LED_PIN, LOW);
  server.send(200, "text/html", webpage);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
}
