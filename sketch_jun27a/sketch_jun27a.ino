void setup() {
  // put your setup code here, to run once:
  pinMode(4, INPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13, digitalRead(4));
}
