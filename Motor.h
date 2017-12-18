/*

Robot Main Brain  --  runs on 1284P and handles onboard control of my robot
     Copyright (C) 2017  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
