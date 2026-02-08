#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ======= Your Pin Mapping (as given) =======
int M11 = D0;
int M12 = D1;
int M21 = D2;
int M22 = D3;
int En  = D4;

ESP8266WebServer server(80);

// ======= Motor Control =======
void stopMotor() {
  digitalWrite(M11, LOW);
  digitalWrite(M12, LOW);
  digitalWrite(M21, LOW);
  digitalWrite(M22, LOW);
}

void fwd(){
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M22, HIGH);
  digitalWrite(M21, LOW);
}

void rev(){
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);
  digitalWrite(M22, LOW);
  digitalWrite(M21, HIGH);
}

void ryt(){
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M22, LOW);
  digitalWrite(M21, HIGH);
}

void lft(){
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);
  digitalWrite(M22, HIGH);
  digitalWrite(M21, LOW);
}

// ======= Simple HTML UI =======
String pageHTML() {
  String html = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP8266 Motor Control</title>
  <style>
    body { font-family: Arial; text-align:center; margin: 20px; }
    .grid { display:grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; max-width: 360px; margin: 0 auto; }
    button { padding: 18px; font-size: 18px; border: 0; border-radius: 10px; cursor:pointer; }
    .stop { grid-column: 2; font-weight: bold; }
    .wide { grid-column: span 3; }
  </style>
</head>
<body>
  <h2>ESP8266 WiFi Motor Controller</h2>

  <div class="grid">
    <div></div>
    <button onclick="cmd('fwd')">▲ FWD</button>
    <div></div>

    <button onclick="cmd('lft')">◀ LEFT</button>
    <button class="stop" onclick="cmd('stop')">■ STOP</button>
    <button onclick="cmd('ryt')">RIGHT ▶</button>

    <div></div>
    <button onclick="cmd('rev')">▼ REV</button>
    <div></div>

    <button class="wide" onclick="cmd('stop')">EMERGENCY STOP</button>
  </div>

  <p id="status">Status: Ready</p>

  <script>
    function cmd(c){
      fetch('/cmd?go=' + c)
        .then(r => r.text())
        .then(t => document.getElementById('status').innerText = 'Status: ' + t)
        .catch(_ => document.getElementById('status').innerText = 'Status: ERROR');
    }
  </script>
</body>
</html>
)rawliteral";
  return html;
}

// ======= HTTP Handlers =======
void handleRoot() {
  server.send(200, "text/html", pageHTML());
}

void handleCmd() {
  if (!server.hasArg("go")) {
    server.send(400, "text/plain", "Missing arg: go");
    return;
  }

  String go = server.arg("go");

  // Safety: default stop if unknown
  if (go == "fwd")      { fwd();      server.send(200, "text/plain", "FORWARD"); }
  else if (go == "rev") { rev();      server.send(200, "text/plain", "REVERSE"); }
  else if (go == "lft") { lft();      server.send(200, "text/plain", "LEFT"); }
  else if (go == "ryt") { ryt();      server.send(200, "text/plain", "RIGHT"); }
  else if (go == "stop"){ stopMotor();server.send(200, "text/plain", "STOPPED"); }
  else {
    stopMotor();
    server.send(200, "text/plain", "UNKNOWN -> STOPPED");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);

  pinMode(En, OUTPUT);
  digitalWrite(En, HIGH);   // enable motor driver
  stopMotor();              // start safe

  // ======= WiFi STA Mode Connect =======
  WiFi.mode(WIFI_STA);
  WiFi.begin("esp8266wifi", "12345678");

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // ======= Web Server Routes =======
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();

  Serial.println("HTTP server started (port 80). Open the IP in browser.");
}

void loop() {
  server.handleClient();
}
