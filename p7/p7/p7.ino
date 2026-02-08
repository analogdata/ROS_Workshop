/*
 * ESP8266 UDP LED Server
 * =======================
 * This Arduino program runs on an ESP8266 microcontroller (like NodeMCU).
 * It connects to a WiFi network and listens for UDP messages.
 * When it receives "on", it turns the built-in LED ON.
 * When it receives "off", it turns the built-in LED OFF.
 * It also sends back an acknowledgment to the sender.
 *
 * How it works:
 *   1. ESP8266 connects to WiFi and gets an IP address
 *   2. It starts listening for UDP packets on port 4210
 *   3. When a packet arrives (e.g., "on"), it reads the message
 *   4. It turns the LED on/off based on the message
 *   5. It sends a reply back to whoever sent the message
 *
 * IMPORTANT: ESP8266's built-in LED is "active LOW"
 *   - digitalWrite(LOW)  = LED turns ON  (counterintuitive!)
 *   - digitalWrite(HIGH) = LED turns OFF
 *   This is because the LED is wired between VCC and the GPIO pin.
 */

// ESP8266WiFi.h - Library to connect ESP8266 to WiFi networks
#include <ESP8266WiFi.h>
// WiFiUdp.h - Library to send and receive UDP packets over WiFi
#include <WiFiUdp.h>

// --- WiFi Credentials ---
// The name (SSID) and password of the WiFi network to connect to
// Both the ESP8266 and your computer must be on the SAME network!
const char* ssid = "OnePlusRajath";
const char* password = "rajathkumarks";

// --- UDP Configuration ---
// The port number this ESP will listen on for incoming UDP packets
// Think of a port like an apartment number - the IP is the building address,
// the port is which apartment to deliver the message to
const unsigned int localPort = 4210;

// --- LED Pin ---
// LED_BUILTIN is the built-in LED on the ESP8266 board (usually GPIO2)
const int LED_PIN = LED_BUILTIN;

// Create a UDP object to handle sending/receiving packets
WiFiUDP udp;

// Buffer to store incoming messages (255 bytes max)
char packetBuffer[255];

// ============================================================
// setup() runs ONCE when the ESP8266 powers on or resets
// ============================================================
void setup() {
  // Start Serial communication at 115200 baud (for debugging via Serial Monitor)
  Serial.begin(115200);

  // Configure the LED pin as an output (so we can turn it on/off)
  pinMode(LED_PIN, OUTPUT);
  // Start with LED OFF (HIGH = OFF because active LOW)
  digitalWrite(LED_PIN, HIGH);

  // --- Connect to WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  // Wait until connected (this loop runs until WiFi is ready)
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  // Print the IP address - YOU NEED THIS to send commands from Python!
  // Look at the Serial Monitor to find this IP address
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  // Start listening for UDP packets on our port
  udp.begin(localPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localPort);
}

// ============================================================
// loop() runs REPEATEDLY forever after setup() finishes
// Think of it as: while(true) { ... }
// ============================================================
void loop() {
  // Check if a UDP packet has arrived
  // parsePacket() returns the size of the packet (0 = no packet)
  int packetSize = udp.parsePacket();

  if (packetSize) {
    // A packet arrived! Read its contents into our buffer
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';  // Add null terminator to make it a proper string
    }

    // Print what we received (visible in Serial Monitor)
    Serial.print("Received: ");
    Serial.println(packetBuffer);

    // Convert to Arduino String for easier comparison
    String msg = String(packetBuffer);
    msg.trim();  // Remove any whitespace/newlines

    // --- Process the command ---
    if (msg.equalsIgnoreCase("on")) {
      // Turn LED ON (remember: LOW = ON for active-low LED)
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED ON");

      // Send acknowledgment back to the sender
      // udp.remoteIP() and udp.remotePort() = the sender's address
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write("LED ON");
      udp.endPacket();

    } else if (msg.equalsIgnoreCase("off")) {
      // Turn LED OFF (HIGH = OFF for active-low LED)
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED OFF");

      // Send acknowledgment back
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write("LED OFF");
      udp.endPacket();

    } else {
      // Unknown command - let the sender know
      Serial.println("Unknown command");
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write("Unknown command");
      udp.endPacket();
    }
  }
  // If no packet arrived, loop() just runs again immediately
}
