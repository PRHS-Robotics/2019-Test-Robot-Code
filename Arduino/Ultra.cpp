/*
 * Ultra.cpp
 *
 *  Created on: Jan 12, 2019
 *      Author: Kaleb Barber
 */

#include "Ultra.h"

Ultra::Ultra(int trig, int echo){
	trigPin = trig;
	echoPin = echo;
	//Set trigPin as input
	pinMode(trigPin, OUTPUT);
	//Set echoPin as output
	pinMode(echoPin, INPUT);
}

double Ultra::getDistance(){
			//Send waves, LOW waves cancel out to make way for HIGH wave
			digitalWrite(trigPin, LOW);
			delayMicroseconds(5);
			digitalWrite(trigPin, HIGH);
			delayMicroseconds(10);
			digitalWrite(trigPin, LOW);
			//Get duration of wave
			float duration = pulseIn(echoPin, HIGH);
			//Convert duration to units (meters)
			float meters = duration * 0.0003335;
			//Return distance
			return meters;
	}