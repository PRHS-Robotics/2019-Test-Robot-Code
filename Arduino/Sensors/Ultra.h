/*
 * Ultra.h
 *
 *  Created on: Jan 12, 2019
 *      Author: Kaleb Barber
 */

#ifndef ULTRA_H_
#define ULTRA_H_
#include <Arduino.h>

class Ultra {
private:
	int trigPin; //trigPin location
	int echoPin; //echoPin location
public:
	Ultra(int trig, int echo);
	double getDistance();
};

#endif /* ULTRA_H_ */
