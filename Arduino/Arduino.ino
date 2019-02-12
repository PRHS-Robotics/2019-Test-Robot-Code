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

Potentiometer pot(A0);

#define BUFFER_SIZE 128
char printBuffer[BUFFER_SIZE];


// Raw data received from RoboRio
struct RxFrame {
  static constexpr const uint32_t magic_number = 0xAA55FAF5;
  uint32_t verification = 0;
  uint8_t light = 0;
  uint8_t align[3];
};

RxFrame receivedFrame;

// Raw data send to RoboRio
struct TxFrame {
  static constexpr const uint32_t magic_number = 0x5FAF55AA;
  uint32_t verification = magic_number;
  uint16_t degrees = 0.0;
  uint16_t distance = 0.0;
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
  Serial.println("yote");
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
  Serial.println("owo");
  snprintf(printBuffer, 128, "Received %d bytes", bytes);
  Serial.println(printBuffer);

  unsigned char data[bytes];
  unsigned char *ptr1 = data;
  for (int i = 0; i < bytes; ++i) {
    *ptr1++ = Wire.read();
  }

  receivedDigit = getHandshakeDigit(data, bytes);

  if (bytes == sizeof(RxFrame)) {
    Serial.println("aaaaa");
    for (int i = 0; i < bytes; ++i) {
      Serial.print(int(data[i]), HEX);
      Serial.print(", ");
    }
    Serial.println("");
    memcpy(reinterpret_cast< void* >(&receivedFrame), data, sizeof(RxFrame));

    Serial.print("Light ");
    Serial.println(receivedFrame.light);
    digitalWrite(10, receivedFrame.light);
  }
  else {
    Serial.print("Received ");
    Serial.print(bytes);
    Serial.println(",  is not RxFrame");
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
    Serial.println(receivedFrame.verification);
    sendFrame(nullFrame);
  }
  else {
    transmittedFrame.verification = TxFrame::magic_number;
    transmittedFrame.distance = sensor1.getDistance();
    transmittedFrame.degrees = pot.getDegrees();

    //transmittedFrame.distance = 12345;
    //transmittedFrame.degrees = 2.53;

    Serial.print("Distance: ");
    Serial.println(transmittedFrame.distance);
    Serial.print("Degrees: ");
    Serial.println(transmittedFrame.degrees);
    Serial.println(sizeof(double));

    for (int i = 0; i < sizeof(decltype(transmittedFrame.distance)); ++i) {
      Serial.print(int(reinterpret_cast< unsigned char* >(&transmittedFrame.distance)[i]), HEX);
      Serial.print(", ");
    }
    Serial.println("");

    sendFrame(transmittedFrame);
  }
}
