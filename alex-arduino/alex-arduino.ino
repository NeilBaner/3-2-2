#include <serialize.h>
#include <stdarg.h>

#include "constants.h"
#include "packet.h"

typedef enum {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV 192

// Wheel circumference in cm.
#define WHEEL_CIRC 20.42

#define PI 3.141592654

// TODO: measure Alex
#define ALEX_LENGTH 16
#define ALEX_WIDTH 6

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF 6   // Left forward pin
#define LR 5   // Left reverse pin
#define RF 10  // Right forward pin
#define RR 9   // Right reverse pin

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Angle of Alex
volatile unsigned long angle;

// The direction in which Alex should move
volatile TDirection dir = STOP;

/*
 *
 * Alex Communication Routines.
 *
 */

TResult readPacket(TPacket *packet) {
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);
}

void sendStatus() {
    TPacket status;
    status.command = COMMAND_GET_STATS;
    status.packetType = PACKET_TYPE_RESPONSE;
    status.params[0] = leftForwardTicks;
    status.params[1] = rightForwardTicks;
    status.params[2] = leftReverseTicks;
    status.params[3] = rightReverseTicks;
    status.params[4] = leftForwardTicksTurns;
    status.params[5] = rightForwardTicksTurns;
    status.params[6] = leftReverseTicksTurns;
    status.params[7] = rightReverseTicksTurns;
    status.params[8] = forwardDist;
    status.params[9] = reverseDist;
    status.params[10] = angle;
    sendResponse(&status);
}

void sendMessage(const char *message) {
    // Sends text messages back to the Pi. Useful
    // for debugging.
    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
    va_list args;
    char buffer[128];
    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}

void sendBadPacket() {
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {
    // Tell the Pi that we don't understand its
    // command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
}

void sendBadResponse() {
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK() {
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */

// Enable pull up resistors on pins 2 and 3
void enablePullups() {
    // Use bare-metal to enable the pull-up resistors on pins
    // 2 and 3. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD &= 0b11110011;
    PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
    if (dir == FORWARD) {
        leftForwardTicks++;
        forwardDist = ((float)(leftForwardTicks)*WHEEL_CIRC) / COUNTS_PER_REV;
    } else if (dir == BACKWARD) {
        leftReverseTicks++;
        reverseDist = ((float)(leftReverseTicks)*WHEEL_CIRC) / COUNTS_PER_REV;
    } else if (dir == LEFT) {
        leftReverseTicks++;
    } else if (dir == RIGHT) {
        leftForwardTicks++;
    }
}

void rightISR() {
    if (dir == FORWARD) {
        rightForwardTicks++;
        forwardDist = ((float)(rightForwardTicks)*WHEEL_CIRC) / COUNTS_PER_REV;
    } else if (dir == BACKWARD) {
        rightReverseTicks++;
        reverseDist = ((float)(rightReverseTicks)*WHEEL_CIRC) / COUNTS_PER_REV;
    } else if (dir == RIGHT) {
        rightReverseTicks++;
    } else if (dir == LEFT) {
        rightForwardTicks++;
    }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
    EICRA = 0b00001010;
    EIMSK = 0b00000011;
}

ISR(INT0_vect) { leftISR(); }

ISR(INT1_vect) { rightISR(); }

/*
 * Setup and start codes for serial communications
 *
 */
// Set up the serial connection.
// TODO: Convert to bare-metal
void setupSerial() {
    UCSR0C = 0b00000110;
    UBRR0 = 103;
    UCSR0A = 0b00000000;
}

// Start the serial connection.
// TODO: Implement
void startSerial() {
    UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// TODO: Convert to bare-metal
int readSerial(char *buffer) {
    // int count = 0;
    // while (Serial.available()) buffer[count++] = Serial.read();
    // return count;
    int count = 0;
    while (UCSR0A & 0b10000000 == 0b10000000){
        buffer[count++] = UDR0;
    }
    return count;
}

// Write to the serial port. 
// TODO: Convert to bare-metal
void writeSerial(const char *buffer, int len) { Serial.write(buffer, len); }

/*
 * Alex's motor drivers.
 *
 */

// Set up Alex's motors.
void setupMotors() {
    /* Our motor set up is:
     *    A1IN - Pin 5, PD5, OC0B
     *    A2IN - Pin 6, PD6, OC0A
     *    B1IN - Pin 10, PB2, OC1B
     *    B2In - pIN 9, PB3, OC1A
     */
    TCCR0A = 0b10100001;  // Set OC0A and OC0B at BOTTOM, clear at OCR0x, use PC
                          // PWM from 0 to 0xFF
    TIMSK0 = 0b00000000;  // No interrupts
    OCR0A = 0;            // Start with these set to 0
    OCR0B = 0;
    TCNT0 = 0;
    TCCR1A = 0b10100001;  // Set OC1A and OC1B at BOTTOM, clear at OCR1x, use PC
                          // PWM from 0 to 0xFF (8-bit mode)
    TIMSK1 = 0b00000000;  // No interrupts
    OCR1A = 0;
    OCR1B = 0;
    TCNT1 = 0;

    DDRD |= 0b01100000;
    DDRB |= 0b00000110;
}

// Start the PWM for Alex's motors.
void startMotors() {
    TCCR0B = 0b00000011;  // Prescalar 1/64
    TCCR1B = 0b00000011;  // Prescalar 1/64
}

// Convert percentages to PWM values
int pwmVal(float speed) {
    if (speed < 0.0) speed = 0;

    if (speed > 100.0) speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
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

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
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

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
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

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
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

// stopAlex Alex. 
void stopAlex() {
    dir = STOP;
    OCR0A = 0;
    OCR0B = 0;
    OCR1A = 0;
    OCR1B = 0;
}

/*
 * Alex's setup and run codes
 *
 */

// Clears all our counters
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

// Clears one particular counter
void clearOneCounter(int which) { clearCounters()   ; }

void initializeState() { clearCounters(); }


void handleCommand(TPacket *command) {
    switch (command->command) {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((float)command->params[0], (float)command->params[1]);
            break;
        case COMMAND_REVERSE:
            sendOK();
            reverse((float)command->params[0], (float)command->params[1]);
            break;
        case COMMAND_TURN_LEFT:
            sendOK();
            left((float)command->params[0], (float)command->params[1]);
            break;
        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float)command->params[0], (float)command->params[1]);
            break;
        case COMMAND_STOP:
            sendOK();
            stopAlex();
            break;
        case COMMAND_CLEAR_STATS:
            sendOK();
            clearOneCounter(command->params[0]);
            break;
        case COMMAND_GET_STATS:
            sendOK();
            sendStatus();
            break;
        default:
            sendBadCommand();
    }
}

void waitForHello() {
    int exit = 0;

    while (!exit) {
        TPacket hello;
        TResult result;

        do {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK) {
            if (hello.packetType == PACKET_TYPE_HELLO) {
                sendOK();
                exit = 1;
            } else
                sendBadResponse();
        } else if (result == PACKET_BAD) {
            sendBadPacket();
        } else if (result == PACKET_CHECKSUM_BAD)
            sendBadChecksum();
    }  // !exit
}

void setup() {
    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    initializeState();
    sei();
    waitForHello();
}

//TODO: figure this stuff out too apparently
void handlePacket(TPacket *packet) {
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;
    }
}

void loop() {
  //forward(0, 100);
  delay(1000);
  stopAlex();
    // forward(0, 100);
    // Uncomment the code below for Week 9 Studio 2

    
     // put your main code here, to run repeatedly:
      TPacket recvPacket; // This holds commands from the Pi

      TResult result = readPacket(&recvPacket);
      



      if(result == PACKET_OK)
        handlePacket(&recvPacket);
      else
        if(result == PACKET_BAD)
        {
          sendBadPacket();
        }
        else
          if(result == PACKET_CHECKSUM_BAD)
          {
            sendBadChecksum();
          }
          



          
}
