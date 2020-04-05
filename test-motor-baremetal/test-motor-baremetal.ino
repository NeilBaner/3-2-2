void setup() {
  // put your setup code here, to run once:
  TCCR0A = 0b10100001;
  TCCR1A = 0b10100001;

  TIMSK0 = 0b00000000;
  TIMSK1 = 0b00000000;

  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;

  TCNT0 = 0;
  TCNT1 = 0;

  DDRD |= 0b01100000;
  DDRB |= 0b00000110;
  
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}

void loop() {
  // put your main code here, to run repeatedly:
  OCR0A = 255;
  OCR1A = 255;
  OCR0B = 0;
  OCR1B = 0;
}
