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

void Motor::loop() {

	static uint32_t lastLoop = millis();
	uint32_t thisLoop = millis();

	if (thisLoop - lastLoop >= 10) {   // only every 10 ms.  One tick at full speed is like 8.5ms

		if (targetSpeed == 0){
			stop();
			return;
		}

		int32_t curSpeed = getSpeed();

		if (targetSpeed > 0){
			if(curSpeed < targetSpeed){
				drive(pwmSpeed +1);
			} else if(curSpeed > targetSpeed){
				if(pwmSpeed > 0){
					drive(pwmSpeed -1);
				}
			}
		}
		if (targetSpeed < 0){
			if(curSpeed > targetSpeed){
				drive(pwmSpeed -1);
			} else if (curSpeed < targetSpeed){
				if(pwmSpeed < 0){
					drive(pwmSpeed +1);
				}
			}
		}

	}
}


//  aSpeed is the PWM speed for the motor + for forward and - for reverse on the direction pin.
void Motor::drive(int16_t aSpeed){
	if(aSpeed > 255){
		aSpeed = 255;
	} else if(aSpeed < -255){
		aSpeed = -255;
	} else if((aSpeed > 0) && (aSpeed < 127)){
		aSpeed = 127;
	} else if((aSpeed < 0) && (aSpeed > -127)){
		aSpeed = -127;
	}
	pwmSpeed = aSpeed;
	if (aSpeed > 0){
		digitalWrite(directionPin, invertForward);
	} else if (aSpeed < 0){
		digitalWrite(directionPin, !invertForward);
	}
	// if aSpeed is 0 then direction is untouched but motor turns off just like stop()
	// otherwise the absolute value goes out there.
	analogWrite(enablePin, abs(aSpeed));
}


int32_t Motor::getSpeed(){
	return encoder.getSpeed();
}


void Motor::setSpeed(int32_t aTarget){
	targetSpeed = aTarget;
}

int16_t Motor::getPwmSpeed(){
	return pwmSpeed;
}
