// with "inspiration" from meister neil banerjee
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino.h>

//pin 5, 6 are port d 5, 6 respectively
#define PIN5MASK 0b00100000
#define PIN6MASK 0b01000000
#define pb1 2
#define pb2 3

volatile char dataSend, dataRecv;
static volatile int num = 18;

void setup() {
  // put your setup code here, to run once:
  cli();
  setupEINT();
  setupSerial();
  DDRD |= 0b01100000; //pins 5,6 are port d
  /*pinMode(pb1, INPUT_PULLUP);
  pinMode(pb2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pb1), switchISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(pb2), switchISR2, RISING);*/
  sei();
}

void setupEINT() {
  EICRA = 0b00001111; // both int1 and int0 are triggered at rising edge
  EIMSK = 0b00000011; // enable both int1 and int0
}

void setupSerial() {
  UCSR0C = 0b00000110; // async 8N1
  //UBRR0 = 103; // we want 9600
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0A = 0; // set to 0 first, when 0b10000000, it means uart has received data
  //UCSR0B = 0b00011000; // enable transfer, receive
  UCSR0B = 0b10011000;
}

/*ISR(INT0_vect) {
  dataSend = '6'; // pin 6, white
  UCSR0B |= 0b00100000;
}

ISR(INT1_vect) {
  dataSend = '5'; // pin 5, blue
  UCSR0B |= 0b00100000;
}*/

ISR(USART_RX_vect){
    dataRecv = UDR0;
    if(dataRecv == 5 + '0'){
        num = 5;
    }else if(dataRecv == 6 + '0'){
        num = 6;
    }
}

ISR(USART_UDRE_vect) {
  UDR0 = dataSend;
  UCSR0B &= 0b11011111;
}

void sendData(unsigned char data) {
  dataSend = data + '0';
  while ((UCSR0A & 0b00100000) == 0);
  UDR0 = (unsigned char)dataSend;
}

unsigned char receiveData(unsigned char data) {
  while ((UCSR0A & 0b10000000) == 0) ;
  unsigned char dataRecv = UDR0;  

  return dataRecv - '0';
}

void loop() {
  // put your main code here, to run repeatedly:
  if (num == 6) { // pin 6, white
    PORTD = PIN6MASK;
    delay(1000);
    PORTD = !PIN6MASK;
    delay(500);
  } else if (num == 5) { // pin 5, blue
    PORTD = PIN5MASK;
    delay(1000);
    PORTD = !PIN5MASK;
    delay(500);
  }
}
