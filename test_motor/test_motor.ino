void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(9, 255);
  analogWrite(10, 0);
  analogWrite(5, 255);
  analogWrite(6, 0);
}
