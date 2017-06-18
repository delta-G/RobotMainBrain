/*
 * Motor.h
 *
 *  Created on: Jun 17, 2017
 *      Author: david
 */

#ifndef MOTOR_H_
#define MOTOR_H_


#include "Arduino.h"



class Motor {

	uint8_t directionPin;
	uint8_t enablePin;

	uint8_t invertForward;  //HIGH / true for left LOW / false for right

public:

	Motor(uint8_t aDirpin, uint8_t aEnabpin, boolean aInvert) : directionPin(aDirpin), enablePin(aEnabpin), invertForward(aInvert){}

	Motor(uint8_t aDirpin, uint8_t aEnabpin) : directionPin(aDirpin), enablePin(aEnabpin), invertForward(false){}

	void init();

	void driveForward();
	void driveBackward();
	void drive(int16_t);
	void stop();



};




#endif /* MOTOR_H_ */
