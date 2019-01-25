#include <Wire.h>

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

void receiveEvent(int bytes){
  Serial.println(bytes);
  if (bytes != 10) {
    receivedDigit = -1;
    return;
  }
  char data[9];
  char *ptr = data;
  for (int i = 0; i < bytes; ++i) {
    *ptr++ = Wire.read();
  }

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
    Serial.println("bird");
    char buf[2] = { 'b', 0 };
    buf[0] = receivedDigit + '0';
    Wire.write(buf);
    //Wire.write("hello, world!");
  }
