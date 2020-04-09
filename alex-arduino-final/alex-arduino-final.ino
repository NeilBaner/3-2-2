#include <serialize.h>
#include <stdarg.h>

#include "constants.h"
#include "packet.h"

#define PI 3.141592654

#define COUNTS_PER_REV 192

#define WHEEL_DIAMETER 65
#define ALEX_LENGTH 16
#define ALEX_WIDTH 6

volatile unsigned long leftForwardTicks, rightForwardTicks;
volatile unsigned long leftReverseTicks, rightReverseTicks;
volatile unsigned long leftForwardTicksTurns, rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns, rightReverseTicksTurns;

volatile unsigned long leftRevs, rightRevs;

volatile unsigned long forwardDist, reverseDist;

volatile TDirection dir = STOP;


// SETUP ROUTINES

// INT0+INT1 enabled, FE triggered
void setupEINT(){
    EICRA = 0b00001010;
    EIMSK = 0b00000011;
}

// Setup serial comms, 9600 bps, 8N1, polling-based
void setupSerial(){
    UCSR0C = 0b00000110;
    UBRR0 = 103;
    UCSR0A = 0b00000000;
}

// Start Serial comms
void startSerial(){
    UCSR0B = 0b00011000;
}

// Setup timers 0 and 1 for PWM
void setupMotors(){
    /* Our motor set up is:
     *    A1IN - Pin 5, PD5, OC0B
     *    A2IN - Pin 6, PD6, OC0A
     *    B1IN - Pin 10, PB2, OC1B
     *    B2In - pIN 9, PB3, OC1A
     */
    TCCR0A = 0b10100001;  // Clear OC0X counting up, set counting down, PC PWM 0x00 to 0xFF
    TIMSK0 = 0b00000000;  // No interrupts
    OCR0A = 0;            
    OCR0B = 0;
    TCNT0 = 0;

    TCCR1A = 0b10100001;  // Clear OC1X counting up, set counting down, PC PWM 0x000 to 0x0FF
    TIMSK1 = 0b00000000;  // No interrupts
    OCR1A = 0;
    OCR1B = 0;
    TCNT1 = 0;

    DDRD |= 0b01100000; 
    DDRB |= 0b00000110;
}

// Start the timers and PWM
void startMotors(){
    TCCR0B = 0b00000011;  // Prescalar 1/64
    TCCR1B = 0b00000011;  // Prescalar 1/64
}

// Enable internal pull up resistors on the interrupt pins
void enablePullups(){
    DDRD &= 0b11110011;
    PORTD |= 0b00001100;
}

void initialiseState(){
    
}

// Clears all counters
void clearCounters() {
    leftForwardTicks = 0;
    leftReverseTicks = 0;
    leftForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightForwardTicks = 0;
    rightReverseTicks = 0;
    rightForwardTicksTurns = 0;
    rightReverseTicksTurns = 0;
    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

void clearCounters(int which){
    switch(which){
        case 0:
            leftForwardTicks = 0;
            break;
        case 1:
            rightForwardTicks = 0;
            break;
        case 2:
            leftReverseTicks = 0;
            break;
        case 3:
            rightReverseTicks = 0;
            break;
        case 4:
            leftForwardTicksTurns = 0;
            break;
        case 5:
            rightForwardTicksTurns = 0;
            break;
        case 6:
            leftReverseTicksTurns = 0;
            break;
        case 7:
            rightReverseTicksTurns = 0;
            break;
        case 8:
            leftRevs = 0;
            break;
        case 9:
            rightRevs = 0;
            break;
        case 10:
            forwardDist = 0;
            break;
        case 11:
            reverseDist = 0;
            break;
        default:
            clearCounters();
    }
}

// ISRs

ISR(INT0_vect){

}

ISR(INT1_vect){

}

void leftISR(){
    switch(dir){
        case FORWARD:
            leftForwardTicks++;
            forwardDist = ((double)(leftForwardTicks) * PI * WHEEL_DIAMETER) / (double)COUNTS_PER_REV;
            break;
        case BACKWARD:
            leftReverseTicks++;
            reverseDist = ((double)(leftReverseTicks) * PI * WHEEL_DIAMETER) / (double)COUNTS_PER_REV;
            break;
        case LEFT:
            leftReverseTicksTurns++;
            break;
        case RIGHT:
            leftForwardTicksTurns++;
            break;
    }
}

void rightISR(){
    switch(dir){
        case FORWARD:
            rightForwardTicks++;
            forwardDist = ((double)(rightForwardTicks) * PI * WHEEL_DIAMETER) / (double)COUNTS_PER_REV;
            break;
        case BACKWARD:
            rightReverseTicks++;
            reverseDist = ((double)(rightReverseTicks) * PI * WHEEL_DIAMETER) / (double)COUNTS_PER_REV;
            break;
        case LEFT:
            rightForwardTicksTurns++;
            break;
        case RIGHT:
            rightReverseTicksTurns++;
            break;
    }
}

// MOVEMENT ROUTINES

// Convert percentages to PWM values
int pwmVal(float speed) {
    if (speed < 0.0) speed = 0;

    if (speed > 100.0) speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

// Move Alex forwards "dist" cm at speed "speed"%. When dist = 0, Alex goes indefinitely.
void forward(float dist, float speed) {
    dir = FORWARD;
    int val = pwmVal(speed);
    int distDesired = forwardDist + dist;
    while (forwardDist < distDesired || dist == 0) {
        OCR0B = val;
        OCR1B = val;
        OCR0A = 0;
        OCR1A = 0;
    }
    stopAlex();
}

// Reverse Alex "dist" cm at speed "speed"%. When dist = 0, Alex reverses indefinitely.
void reverse(float dist, float speed) {
    dir = BACKWARD;
    int val = pwmVal(speed);
    int distDesired = reverseDist + dist;
    while (reverseDist < distDesired || dist == 0) {
        OCR0A = val;
        OCR1A = val;
        OCR0B = 0;
        OCR1B = 0;
    }
    stopAlex();
}

// Turn Alex left "ang" degrees at speed "speed"%. When ang = 0, Alex turns indefinitely
void left(float ang, float speed) {
    float alex_circ = PI * ALEX_WIDTH;
    int alex_circ_ticks = (alex_circ / WHEEL_CIRC) * COUNTS_PER_REV;
    dir = LEFT;
    int val = pwmVal(speed);
    int leftInit = leftReverseTicksTurns, rightInit = rightForwardTicksTurns;
    while((leftInit + (ang * alex_circ_ticks / 360) < leftReverseTicksTurns && rightInit + (ang * alex_circ_ticks / 360) < rightForwardTicksTurns) || ang == 0){
        OCR0B = val;
        OCR1A = val;
        OCR0A = 0;
        OCR1B = 0;
    }
    stopAlex();
}

// Turn Alex right "ang" degrees at speed "speed"%. When ang = 0, Alex turns indefinitely
void right(float ang, float speed) {
    float alex_circ = PI * ALEX_WIDTH;
    int alex_circ_ticks = (alex_circ / WHEEL_CIRC) * COUNTS_PER_REV;
    dir = RIGHT;
    int val = pwmVal(speed);    
    int leftInit = leftForwardTicksTurns, rightInit = rightReverseTicksTurns;
    while((leftInit + (ang * alex_circ_ticks / 360) < leftForwardTicksTurns && rightInit + (ang * alex_circ_ticks / 360) < rightReverseTicksTurns) || ang == 0){
        OCR0A = val;
        OCR1B = val;
        OCR0B = 0;
        OCR1A = 0;
    }
    stopAlex();
}

// stop Alex, no comment. 
void stopAlex() {
    dir = STOP;
    OCR0A = 0;
    OCR0B = 0;
    OCR1A = 0;
    OCR1B = 0;
}

// TEST ROUTINES

void testMovements(){
    forward(100, 100);
    stopAlex();
    reverse(50, 100);
    stopAlex();
    left(90, 50);
    right(180, 50);
    stopAlex();
}

void testCommunications(){

}

void setup(){
    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    initialiseState();
    sei();
}

void loop(){
    testMovements();
}
