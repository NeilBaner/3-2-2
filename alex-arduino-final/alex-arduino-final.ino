#include <avr/sleep.h>
#include <math.h>
#include <serialize.h>
#include <stdarg.h>

#include "constants.h"
#include "packet.h"

#define PI 3.141592654

#define COUNTS_PER_REV 192

#define WHEEL_DIAMETER 6.5
#define ALEX_LENGTH 16
#define ALEX_WIDTH 6
#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_STANDBY_MODE_MASK \
    0b11111100  // 110 for standby, last bit is the sleep enable bit

volatile unsigned long leftForwardTicks, rightForwardTicks;
volatile unsigned long leftReverseTicks, rightReverseTicks;
volatile unsigned long leftForwardTicksTurns, rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns, rightReverseTicksTurns;

volatile float leftForwardMultiplier, rightForwardMultiplier;
volatile float leftReverseMultiplier, rightReverseMultiplier;

volatile unsigned long leftRevs, rightRevs;

volatile unsigned long forwardDist, reverseDist;

volatile unsigned long deltaDist, newDist;
volatile unsigned long deltaTicks, newTicks;
volatile unsigned long initialSpeed;

volatile unsigned long leftForwardTicksPrevious, rightForwardTicksPrevious;
volatile unsigned long leftReverseTicksPrevious, rightReverseTicksPrevious;

volatile TDirection dir = STOP;

// SETUP ROUTINES

// INT0+INT1 enabled, FE triggered
void setupEINT() {
    EICRA = 0b00001010;
    EIMSK = 0b00000011;
}

// Setup serial comms, 9600 bps, 8N1, polling-based
void setupSerial() { Serial.begin(9600); }

// Start Serial comms
void startSerial() {}

// Setup timers 0 and 1 for PWM
void setupMotors() {
    /* Our motor set up is:
          A1IN - Pin 5, PD5, OC0B
          A2IN - Pin 6, PD6, OC0A
          B1IN - Pin 10, PB2, OC1B
          B2In - pIN 9, PB3, OC1A
    */
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
}

// Start the timers and PWM
void startMotors() {
    TCCR0B = 0b00000011;  // Prescalar 1/64
    TCCR1B = 0b00000011;  // Prescalar 1/64
}

// Setup Timer 2 for the "PID"
void setupTimer2() {
    TCCR2A = 0b00000010;  // set to CTC mode
    OCR2A = 156;
    TIMSK2 = 0b00000010;  // Interrupt on Compare Match w/OCR2A
}

void startTimer2() {
    TCCR2B = 0b00000100;  // Prescalar 1/64
}

// Enable internal pull up resistors on the interrupt pins
void enablePullups() {
    DDRD &= 0b11110011;
    PORTD |= 0b00001100;
}

void initialiseState() {
    clearCounters();
    leftForwardMultiplier = 1.0;
    rightForwardMultiplier = 1.0;
    leftReverseMultiplier = 1.0;
    rightReverseMultiplier = 1.0;
    stopAlex();
}

// Clears all counters
void clearCounters() {
    leftForwardTicks = 0;
    leftReverseTicks = 0;
    leftForwardTicksPrevious = 0;
    leftReverseTicksPrevious = 0;
    leftForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightForwardTicks = 0;
    rightReverseTicks = 0;
    rightForwardTicksPrevious = 0;
    rightReverseTicksPrevious = 0;
    rightForwardTicksTurns = 0;
    rightReverseTicksTurns = 0;
    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

void clearCounters(int which) {
    switch (which) {
        case 0:
            leftForwardTicks = 0;
            leftForwardTicksPrevious = 0;
            break;
        case 1:
            rightForwardTicks = 0;
            rightForwardTicksPrevious = 0;
            break;
        case 2:
            leftReverseTicks = 0;
            leftReverseTicksPrevious = 0;
            break;
        case 3:
            rightReverseTicks = 0;
            rightReverseTicksPrevious = 0;
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

ISR(INT0_vect) { leftISR(); }

ISR(INT1_vect) { rightISR(); }

ISR(TIMER2_COMPA_vect) { pidISR(); }

void leftISR() {
    //  leftForwardMultiplier = 0;
    //  leftReverseMultiplier = 0;
    switch (dir) {
        case FORWARD:
            leftForwardTicks++;
            forwardDist = ((double)(leftForwardTicks)*PI * WHEEL_DIAMETER) /
                          (double)COUNTS_PER_REV;
            //      leftForwardMultiplier = 1 - 0.1 * (leftForwardTicks -
            //      rightForwardTicks); leftReverseMultiplier = 0;

            break;
        case BACKWARD:
            leftReverseTicks++;
            reverseDist = ((double)(leftReverseTicks)*PI * WHEEL_DIAMETER) /
                          (double)COUNTS_PER_REV;
            //      leftReverseMultiplier = 1 - 0.1 * (leftReverseTicks -
            //      rightReverseTicks); leftForwardMultiplier = 0;
            break;
        case LEFT:
            leftReverseTicksTurns++;
            //      leftReverseMultiplier = 1 - 0.1 * (leftReverseTicks -
            //      rightForwardTicks); // tbh i'm not sure if this needs
            //      correction? leftForwardMultiplier = 0;
            break;
        case RIGHT:
            leftForwardTicksTurns++;
            //      leftForwardMultiplier = 1 - 0.1 * (leftForwardTicks -
            //      rightReverseTicks); // does it need correction?
            //      leftReverseTicks = 0;
            break;
    }
    //  OCR0B = pwmVal(initialSpeed * leftForwardMultiplier);
    //  OCR0A = pwmVal(initialSpeed * leftReverseMultiplier);
}

void rightISR() {
    //  rightForwardMultiplier = 0;
    //  rightReverseMultiplier = 0;
    switch (dir) {
        case FORWARD:
            rightForwardTicks++;
            forwardDist =
                ((double)(rightForwardTicks - 5) * PI * WHEEL_DIAMETER) /
                (double)COUNTS_PER_REV;
            //      rightReverseMultiplier = 0;
            //      rightForwardMultiplier = 1 - 0.1 * (rightForwardTicks -
            //      leftForwardTicks); //actually... why not rightForwardTicks =
            //      leftForwardTicks?

            break;
        case BACKWARD:
            rightReverseTicks++;
            reverseDist =
                ((double)(rightReverseTicks - 5) * PI * WHEEL_DIAMETER) /
                (double)COUNTS_PER_REV;
            //      rightForwardMultiplier = 0;
            //      rightReverseMultiplier = 1 - 0.1 * (rightReverseTicks -
            //      leftReverseTicks);
            break;
        case LEFT:
            rightForwardTicksTurns++;
            //      rightForwardMultiplier = 1 - 0.1 * (rightForwardTicks -
            //      leftReverseTicks); rightReverseMultiplier = 0;
            break;
        case RIGHT:
            rightReverseTicksTurns++;
            //      rightReverseMultiplier = 1 - 0.1 * (rightReverseTicks -
            //      leftForwardTicks); rightForwardMultiplier = 0;
            break;
    }
    //  OCR1B = pwmVal(initialSpeed * rightForwardMultiplier);
    //  OCR1A = pwmVal(initialSpeed * rightReverseMultiplier);
}

void pidISR() {
    int leftDist, rightDist;
    switch (dir) {
        case FORWARD:
            leftDist = leftForwardTicks - leftForwardTicksPrevious;
            rightDist = rightForwardTicks - rightForwardTicksPrevious;
            if (leftDist - rightDist > 0) {
                if (rightForwardMultiplier == 1) {
                    leftForwardMultiplier -= 0.01;
                } else {
                    rightForwardMultiplier += 0.01;
                }
            } else if (leftDist - rightDist < 0) {
                if (leftForwardMultiplier == 1) {
                    rightForwardMultiplier -= 0.01;
                } else {
                    leftForwardMultiplier += 0.01;
                }
            }
            break;
        case BACKWARD:
            leftDist = leftReverseTicks - leftReverseTicksPrevious;
            rightDist = rightReverseTicks - rightReverseTicksPrevious;
            if (leftDist - rightDist > 0) {
                if (rightReverseMultiplier == 1.0) {
                    leftReverseMultiplier -= 0.01;
                } else {
                    rightReverseMultiplier += 0.01;
                }
            } else if (leftDist - rightDist < 0) {
                if (leftReverseMultiplier == 1.0) {
                    rightReverseMultiplier -= 0.01;
                } else {
                    leftReverseMultiplier += 0.01;
                }
            }
            break;
    }
    leftForwardTicksPrevious = leftForwardTicks;
    leftReverseTicksPrevious = leftReverseTicks;
    rightForwardTicksPrevious = rightForwardTicks;
    rightReverseTicksPrevious = rightReverseTicks;
}

// MOVEMENT ROUTINES

// Convert percentages to PWM values
int pwmVal(float speed) {
    if (speed < 0.0) speed = 0;

    if (speed > 100.0) speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

// Move Alex forwards "dist" cm at speed "speed"%. When dist = 0, Alex goes
// indefinitely.
void forward(float dist, float speed) {
    dir = FORWARD;
    int val = pwmVal(speed);
    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 999999;
    }
    newDist = forwardDist + deltaDist;
    OCR0B = leftForwardMultiplier * val;
    OCR1B = rightForwardMultiplier * val;
    OCR0A = 0;
    OCR1A = 0;
}

// Reverse Alex "dist" cm at speed "speed"%. When dist = 0, Alex reverses
// indefinitely.
void reverse(float dist, float speed) {
    dir = BACKWARD;
    int val = pwmVal(speed);
    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 999999;
    }
    newDist = reverseDist + deltaDist;
    OCR0A = val;
    OCR1A = val;
    OCR0B = 0;
    OCR1B = 0;
}

// Turn Alex left "ang" degrees at speed "speed"%. When ang = 0, Alex turns
// indefinitely
void left(float ang, float speed) {
    float alex_circ = PI * ALEX_WIDTH;
    int alex_circ_ticks = (alex_circ / (WHEEL_DIAMETER * PI)) * COUNTS_PER_REV;
    dir = LEFT;
    int initialSpeed = speed;
    int val = pwmVal(speed);
    int leftInit = leftReverseTicksTurns, rightInit = rightForwardTicksTurns;
    while (
        (leftInit + (ang * alex_circ_ticks / 360) < leftReverseTicksTurns &&
         rightInit + (ang * alex_circ_ticks / 360) < rightForwardTicksTurns) ||
        ang == 0) {
        OCR0B = val;
        OCR1A = val;
        OCR0A = 0;
        OCR1B = 0;
    }
    stopAlex();
}

// Turn Alex right "ang" degrees at speed "speed"%. When ang = 0, Alex turns
// indefinitely
void right(float ang, float speed) {
    float alex_circ = PI * ALEX_WIDTH;
    int alex_circ_ticks = (alex_circ / (WHEEL_DIAMETER * PI)) * COUNTS_PER_REV;
    dir = RIGHT;
    int val = pwmVal(speed);
    int initialSpeed = speed;
    int leftInit = leftForwardTicksTurns, rightInit = rightReverseTicksTurns;
    while (
        (leftInit + (ang * alex_circ_ticks / 360) < leftForwardTicksTurns &&
         rightInit + (ang * alex_circ_ticks / 360) < rightReverseTicksTurns) ||
        ang == 0) {
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
    newDist = 0;
    deltaDist = 0;
}

// SERIAL ROUTINES

int readSerial(char *buffer) {
    int count = 0;
    while (Serial.available()) {
        buffer[count++] = Serial.read();
    }
    return count;
}

// int readSerialLessOld(char *buffer) {
//  int count = 0;
//  while (count < PACKET_SIZE) {
//    while (UCSR0A & 0b10000000 == 0);
//    buffer[count++] = UDR0;
//  }
//  return count;
//}
//
//
// int readSerialOld(char *buffer) {
//  int count = 0;
//  while (UCSR0A & 0b00100000 == 0);
//  while (UCSR0A & 0b10000000 == 0b10000000) {
//    buffer[count++] = UDR0;
//  }
//  return count;
//}

void writeSerial(const char *buffer, int len) { Serial.write(buffer, len); }

// COMMUNICATION ROUTINES

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

void sendBadPacket() {
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
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
    int len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
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
    sendResponse(&status);
}

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
            sendOK();
            break;
    }
}

void handleCommand(TPacket *command) {
    switch (command->command) {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((float)command->params[0], (float)command->params[1]);
            SMCR = 00001100;  // clear the enable sleep bit immediately after
                              // waking up?
            break;
        case COMMAND_REVERSE:
            sendOK();
            reverse((float)command->params[0], (float)command->params[1]);
            SMCR = 00001100;  // clear the sleep bit immediately after waking
                              // up?
            break;
        case COMMAND_TURN_LEFT:
            sendOK();
            left((float)command->params[0], (float)command->params[1]);
            SMCR = 00001100;  // clear the sleep bit immediately after waking
                              // up?
            break;
        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float)command->params[0], (float)command->params[1]);
            SMCR = 00001100;  // clear the sleep bit immediately after waking
                              // up?
            break;
        case COMMAND_STOP:
            sendOK();
            stopAlex();
            break;
        case COMMAND_CLEAR_STATS:
            sendOK();
            clearCounters(command->params[0]);
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
        // forward(10, 100);
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

// POWER SAVING ROUTINES
void WDT_off(void) {
    /* Global interrupt should be turned OFF here if not
      already done so */
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1 << WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional
      time-out */
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
}

/* Global interrupt should be turned ON here if
  subsequent operations after calling this function do
  not require turning off global interrupt */
void setupPowerSaving() {
    // Turn off the Watchdog Timer
    WDT_off();
    /*// Modify PRR to shut down TWI
      PRR |= PRR_TWI_MASK;
      // Modify PRR to shut down SPI
      PRR |= PRR_SPI_MASK;
      // Modify ADCSRA to disable ADC,
      PRR |= ADCSRA_ADC_MASK;
      // then modify PRR to shut down ADC
      PRR |= PRR_ADC_MASK; */
    // Set the SMCR to choose the STANDBY sleep mode
    SMCR |= SMCR_STANDBY_MODE_MASK;  // we want 00001100
                                     // Do not set the Sleep Enable (SE) bit yet
}

void putArduinoToStandby() {
    // called when the motors are stopped
    // Modify PRR to shut down TIMER 0, 1, and 2
    PRR |= 0b01101000;
    // Modify PRR to shut down TWI
    PRR |= PRR_TWI_MASK;
    // Modify PRR to shut down SPI
    PRR |= PRR_SPI_MASK;
    // Modify ADCSRA to disable ADC,
    PRR |= ADCSRA_ADC_MASK;
    // then modify PRR to shut down ADC
    PRR |= PRR_ADC_MASK;
    // Modify SE bit in SMCR to enable (i.e., allow) sleep
    SMCR |= SMCR_SLEEP_ENABLE_MASK;
    // The following function puts ATmega328P’s MCU into sleep;
    // it wakes up from sleep when USART serial data arrives
    sleep_cpu();
    // Modify SE bit in SMCR to disable (i.e., disallow) sleep
    SMCR &= SMCR_STANDBY_MODE_MASK;
    // Modify PRR to power up TIMER 0, 1, and 2
    PRR &= 0b0;
}

// TEST ROUTINES

void testMovements() {
    forward(100, 100);
    stopAlex();
    reverse(50, 100);
    stopAlex();
    left(90, 50);
    right(180, 50);
    stopAlex();
}

void testCommunications() {}

void setup() {
    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    setupTimer2();
    startTimer2();
    enablePullups();
    initialiseState();
    sei();
    // waitForHello();
    // setupPowerSaving();
}

void loop() {
    if (deltaDist > 0) {
        switch (dir) {
            case FORWARD:
                if (forwardDist >= newDist) {
                    stopAlex();
                }
                break;
            case BACKWARD:
                if (reverseDist >= newDist) {
                    stopAlex();
                }
        }
    }
    if (dir == STOP) {
        stopAlex();
        // putArduinoToStandby();
    }
    TPacket recvPacket;  // This holds commands from the Pi
    TResult result = readPacket(&recvPacket);
    if (result == PACKET_OK) {
        handlePacket(&recvPacket);
    } else if (result == PACKET_BAD) {
        sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD) {
        sendBadChecksum();
    }
}
