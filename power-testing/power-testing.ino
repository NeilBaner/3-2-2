#include <avr/sleep.h>
#include <avr/wdt.h>

void setup() {
  // put your setup code here, to run once:
  cli();
  timersSetup();
  wdtDisable();
  adcDisable();
  spiDisable();
  twiDisable();
  timersDisable();
  sleepMode();
  sei();
}

void wdtDisable(){
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1 << WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional
      time-out */
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
}
void adcDisable(){
    ADCSRA &= 0b01111111;
    PRR |= 0b00000001;
}
void spiDisable(){
    PRR |= 0b00000100;
}
void twiDisable(){
    PRR |= 0b10000000;
}
void timersDisable(){
    PRR |= 0b01101000;
}

void timersSetup(){
    TCCR0A = 0b10100001;  // Clear OC0X counting up, set counting down, PC PWM
    // 0x00 to 0xFF
    TIMSK0 = 0b00000000;  // No interrupts
    OCR0A = 0;
    OCR0B = 0;
    TCNT0 = 0;

    TCCR1A = 0b10100001;  // Clear OC1X counting up, set counting down, PC PWM
    // 0x000 to 0x0FF
    TIMSK1 = 0b00000000;  // No interrupts
    OCR1A = 0;
    OCR1B = 0;
    TCNT1 = 0;

    DDRD |= 0b01100000;
    DDRB |= 0b00000110;
    TCCR2A = 0b00000010;  // set to CTC mode
    OCR2A = 20;
    // 156 was the Prachi value
    TIMSK2 = 0b00000010;  // Interrupt on Compare Match w/OCR2A
    TCCR0B = 0b00000011;  // Prescalar 1/64
    TCCR1B = 0b00000011;  // Prescalar 1/64
    TCCR2B = 0b00000100;  // Prescalar 1/64
}
void sleepMode(){
    SMCR = 00001101;
    sleep_cpu();
}
void loop() {
  // put your main code here, to run repeatedly:
  
}
