void setup() {
    // put your setup code here, to run once:

    Serial.begin(9600);
    
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(2), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(3), rightISR, RISING);

}

void leftISR(){
  Serial.println("left");
}

void rightISR(){
  Serial.println("right");
}

int leftTicks
void loop() {
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  analogWrite(5, 0);
  analogWrite(9, 0);
  
  delay(1000);

  analogWrite(5, 0);
  analogWrite(9, 0);

  while(1<2){
    delay(1000);
  }
}
