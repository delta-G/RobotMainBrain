/*
 * Motor.cpp
 *
 *  Created on: Jun 17, 2017
 *      Author: david
 */


#include "Motor.h"

void Motor::init(){
	digitalWrite(enablePin, LOW);
	pinMode(enablePin, OUTPUT);
	digitalWrite(directionPin, invertForward);
	pinMode(directionPin, OUTPUT);
}

void Motor::stop(){
	digitalWrite(enablePin, LOW);
}

void Motor::driveForward(){
	digitalWrite(directionPin, invertForward);
	digitalWrite(enablePin, HIGH);
}

void Motor::driveBackward() {
	digitalWrite(directionPin, !invertForward);
	digitalWrite(enablePin, HIGH);
}


//  Speed must be constrained to between -255 and +255
void Motor::drive(int16_t aSpeed){

	if (aSpeed > 0){
		digitalWrite(directionPin, invertForward);
	} else if (aSpeed < 0){
		digitalWrite(directionPin, !invertForward);
	}

	// if aSpeed is 0 then direction is untouched but motor turns off just like stop()
	analogWrite(enablePin, abs(aSpeed));

}

