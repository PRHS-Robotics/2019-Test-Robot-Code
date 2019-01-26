#include <Wire.h>

// Workaround for problem in VSCode Arduino extension
#include "Sensors/Potentiometer.hh"
#include "Sensors/Ultra.hh"

// sda on A4
// scl on A5

void loop() {
  delay(10); 
}

Ultra sensor1(11, 12);

Potentiometer pot(A5);

#define BUFFER_SIZE 128
char printBuffer[BUFFER_SIZE];


// Raw data received from RoboRio
struct RxFrame {
  static constexpr const uint32_t magic_number = 0xAA55FAF5;
  uint32_t verification = 0;
};

RxFrame receivedFrame;

// Raw data send to RoboRio
struct TxFrame {
  static constexpr const uint32_t magic_number = 0x5FAF55AA;
  uint32_t verification = magic_number;
  uint16_t degrees = 0.0;
  double distance = 0.0;
};

TxFrame transmittedFrame;

int sendFrame(struct TxFrame frame) {
  return Wire.write(reinterpret_cast< unsigned char* >(&frame), sizeof(TxFrame));
}

// Represents an invalid frame
TxFrame nullFrame;

void setup(){
  nullFrame.verification = 0;

  Serial.begin(9600);
  Serial.println("yeet");
  Wire.begin(80);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); 
}

int receivedDigit = -1;

int getHandshakeDigit(unsigned char *data, int length) {
  if (length != 10) {
    return -1;
  }

  if (strncmp(reinterpret_cast< const char* >(data), "Sending ", 8)) {
    return -1;
  }

  int value = data[8] - '0';
  if (value < 0 || value >= 9) {
    return -1;
  }

  return value;
}

void receiveEvent(int bytes) {
  snprintf(printBuffer, 128, "Received %d bytes", bytes);
  Serial.println(printBuffer);

  unsigned char data[bytes];
  for (unsigned char *ptr = data; ptr != data + bytes; ++ptr) {
    *ptr = Wire.read();
  }

  receivedDigit = getHandshakeDigit(data, bytes);

  if (bytes == sizeof(RxFrame)) {
    memcpy(reinterpret_cast< void* >(&receivedFrame), data, sizeof(RxFrame));
  }
}

void requestEvent(){
  if (receivedDigit != -1) {
    char buf[2] = { static_cast< char >(receivedDigit + 1 + '0'), 0 };
    Wire.write(buf);

    Serial.print("Replied to handshake with ");
    Serial.println(buf);

    receivedDigit = -1;

    return;
  }

  if (receivedFrame.verification != RxFrame::magic_number) {
    Serial.println("Rx Verification incorrect, sending null frame");
    sendFrame(nullFrame);
  }
  else {
    transmittedFrame.distance = sensor1.getDistance();
    transmittedFrame.degrees = pot.getDegrees();

    sendFrame(transmittedFrame);
  }
}
