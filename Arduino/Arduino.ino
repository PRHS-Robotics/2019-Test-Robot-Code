#include <Wire.h>
#include <Ultra.h>

// sda on A4
// scl on A5

String piOutput = "none";

String input = "blank";

void loop() { 
 int sensorValue = analogRead(A5);
 int degreeVal = sensorValue / 3.86;
 //Serial.println(degreeVal);
 delay(10); 
 }

int receivedDigit;

Ultra sensor1(11, 12);

void setup(){
   Serial.begin(9600);
   Serial.println("yeet");
   Wire.begin(80);
   Wire.onReceive(receiveEvent);
   Wire.onRequest(requestEvent); 
 }

/*void requestEvent(){ 
   Wire.write("Sending 7");
}*/

bool sendUltrasonic = false;

bool sendHandshake = false;

void receiveEvent(int bytes){
  Serial.println(bytes);
  if (bytes != 10) {
    receivedDigit = -1;
    return;
  }

  char data[bytes];
  char *ptr = data;
  for (int i = 0; i < bytes; ++i) {
    *ptr++ = Wire.read();
  }

  sendUltrasonic = (data[0] == 2);

  sendHandshake = (strncmp(data, "Sending ", 8) == 0);

  if (strncmp(data, "Sending ", 8)) {
    receivedDigit = -1;
    return;
  }

  int value = data[8] - '0';
  Serial.print("Raw Value: ");
  Serial.println(int(data[8]));
  Serial.print("Value: ");
  Serial.println(value);

  receivedDigit = value + 1;
  if (receivedDigit <= 0 || receivedDigit > 9) {
    receivedDigit = -1;
    return;
  }

  Serial.print("Sending (Hex): ");
  Serial.println(receivedDigit + '0', HEX);

  
}

void requestEvent(){
    if (sendHandshake) {
      Serial.println("bird");
      char buf[2] = { 'b', 0 };
      buf[0] = receivedDigit + '0';
      Wire.write(buf);
    }
    if (sendUltrasonic) {
      double meters = sensor1.getDistance();
    	Serial.print(meters);
    	Serial.print("m");
      Wire.write(meters);
    }
  }
