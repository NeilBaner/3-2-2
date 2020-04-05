void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, 255);
  analogWrite(6, 0);
  analogWrite(9, 255);
  analogWrite(10, 0);
  delay(1000);
  analogWrite(5, 0);
  analogWrite(9, 0);
  while(1<2);
}
