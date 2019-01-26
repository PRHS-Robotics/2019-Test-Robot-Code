#include "Potentiometer.h"

Potentiometer::Potentiometer(int pin) :
    inputPin(pin)
{

}

double Potentiometer::getDegrees() {
    return analogRead(inputPin) / 3.86;
}