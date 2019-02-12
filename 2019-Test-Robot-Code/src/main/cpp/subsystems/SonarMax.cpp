#include "subsystems/SonarMax.h"

SonarMax::SonarMax(int analogPort) :
    m_analogInput(analogPort)
{
    
}

double SonarMax::getDistance() {
    return (m_analogInput.GetVoltage() / (5.0 / 512.0)) + 2.0;
}


    
        /* const int anPin = 0;
     long anVolt, mm, inches;

     void setup() {
     Serial.begin(9600);
     }

     void read_sensor(){
     anVolt = analogRead(anPin);
     mm = anVolt * 5;
     inches = mm/25.4;
     }

     void print_range(){
     Serial.print("S1");
     Serial.print("=");
     Serial.print(mm);
     Serial.print(" ");
     Serial.println(inches);
     }

     void loop() {
     read_sensor();
     print_range();
     delay(100);
     const int pwPin1 = 3;
     long sensor, mm, inches;

     void setup() {
     Serial.begin(9600);
     pinMode(pwPin1, INPUT); }

     void read_sensor (){
     sensor = pulseIn(pwPin1, HIGH);
     mm = sensor;
     inches = mm/25.4;
     }

void print_range(){
Serial.print("S1");
Serial.print("=");
Serial.print(mm);
Serial.print(" ");
Serial.println(inches);
}

void loop() { read_sensor();
print_range();
delay(100);
}
     }*/


        





    }


}