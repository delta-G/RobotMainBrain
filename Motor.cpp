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

