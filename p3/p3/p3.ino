int M11 = D0;
int M12 = D1;
int M21 = D2;
int M22 = D3;

int En = D4;

void setup() {
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);

  pinMode(En, OUTPUT);
  digitalWrite(En, HIGH);

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



void loop() {
  // put your main code here, to run repeatedly:
  fwd();
  delay(10000);
  rev();
  delay(10000);
  ryt();
  delay(10000);
  lft();
  delay(10000);


}
