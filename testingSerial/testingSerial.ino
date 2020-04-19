#define __CONTROL_H__
#include <stdint.h>
#define MAX_STR_LEN   32
#define PACKET_SIZE 8
// This packet has 1 + 1 + 2 + 32 + 16 * 4 = 100 bytes
typedef struct
{
  char packetType;
  char command;
  char dummy[2]; // Padding to make up 4 bytes
  char data[MAX_STR_LEN]; // String data
  uint32_t params[16];
} TPacket;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  setupSerial();
  startSerial();
}

// Setup serial comms, 9600 bps, 8N1, polling-based
void setupSerial() {
  UCSR0C = 0b00000110; // 8 bits of data
  UBRR0 = 103;
  UCSR0A = 0b00000000;
  //UCSR0A = 0b10000000;
}

// Start Serial comms
void startSerial() {
  UCSR0B = 0b00011000;
}

int readSerial(char *buffer) {
  int count = 0;
  while (UCSR0A & 0b10000000 == 0b10000000) {
    buffer[count++] = UDR0;
    return count;
  }
}

bool readPacket(TPacket * packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return false;
  else
    return true;
}

void writeSerial(const char *buffer, int len) {
  int count = 0;
  UDR0 = buffer[count++];
}

void loop() {
  TPacket recvPacket;  // This holds commands from the Pi
  //TResult result = readPacket(&recvPacket);
  bool result = readPacket(&recvPacket);
  if (result) {
    //handlePacket(&recvPacket);
    digitalWrite(13, LOW); //if packet is received, light goes off
  } else
    digitalWrite(5, HIGH); //packet not received
}
