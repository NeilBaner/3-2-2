// with "inspiration" from meister neil banerjee
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino.h>

//pin 5, 6 are port d 5, 6 respectively
#define PIN5MASK 0b00100000
#define PIN6MASK 0b01000000

volatile char dataSend, dataRecv;

void setup() {
  // put your setup code here, to run once:
  cli();
  setupEINT();
  setupSerial();
  DDRD |= 0b01100000; //pins 5,6 are port d
  sei();
}

void setupEINT() {
  EICRA = 0b00001111; // both int1 and int0 are triggered at rising edge
  EIMSK = 0b00000011; // enable both int1 and int0
}

void setupSerial() {
  UCSR0C = 0b00000110; // async 8N1
  UBRR0 = 103; // we want 9600
  UCSR0A = 0; // set to 0 first, when 0b10000000, it means uart has received data
  UCSR0B = 0b00011000; // enable transfer, receive
}

ISR(INT0_vect) {
  dataSend = '6'; // pin 6, white
  UCSR0B |= 0b00100000;
}

ISR(INT1_vect) {
  dataSend = '5'; // pin 5, blue
  UCSR0B |= 0b01000000;
}

ISR(USART_UDRE_vect) {
  UDR0 = dataSend;
  UCSR0B &= 0b11011111;
}

void sendData(unsigned char data) {
  while (UCSR0A & 0b00100000 == 0);
  UDR0 = (unsigned char)data;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (dataRecv == 6 + '0') { // pin 6, white
    PORTD = PIN6MASK;
    delay(1000);
    PORTD = !PIN6MASK;
    delay(500);
  } else if (dataRecv == 6 + '0') { // pin 5, blue
    PORTD = PIN5MASK;
    delay(1000);
    PORTD = !PIN5MASK;
    delay(500);
  }
}
