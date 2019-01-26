#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

class Potentiometer {
public:
    Potentiometer(int pin);

    double getDegrees();

private:
    int inputPin;
};


#endif