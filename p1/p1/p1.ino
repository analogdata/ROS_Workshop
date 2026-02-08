int led = D0;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led, HIGH); 
  delay(1000);
  digitalWrite(led, LOW); 
  delay(1000);
}
