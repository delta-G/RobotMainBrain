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
	targetSpeed = 0;
	motorPID.SetOutputLimits(-255.0, 255.0);
}

void Motor::stop(){
	digitalWrite(enablePin, LOW);
	pwmSpeed = 0;
	targetSpeed = 0;
}

void Motor::driveForward(){
	digitalWrite(directionPin, invertForward);
	digitalWrite(enablePin, HIGH);
	targetSpeed = 0x7FFFFFFF;
}

void Motor::driveBackward() {
	digitalWrite(directionPin, !invertForward);
	digitalWrite(enablePin, HIGH);
	targetSpeed = 0xFFFFFFFF;
}

void Motor::drivePWM() {
	if (pwmSpeed > 0) {
		digitalWrite(directionPin, invertForward);
	} else if (pwmSpeed < 0) {
		digitalWrite(directionPin, !invertForward);
	}
	// if aSpeed is 0 then direction is untouched but motor turns off just like stop()
	// otherwise the absolute value goes out there.
	analogWrite(enablePin, abs(pwmSpeed));
}

void Motor::setTuningByString(char* aTuningString){
	char tuningString[30];
	double Kp, Ki, Kd;
	strncpy(tuningString, aTuningString, 29);
	char* p = strtok(tuningString, ",");
	if (*p == 'x'){
		Kp = motorPID.GetKp();
	} else {
		Kp = atof(p);
	}
	p = strtok(NULL, ",");
	if (*p == 'x'){
		Ki = motorPID.GetKi();
	} else {
		Ki = atof(p);
	}
	p = strtok(NULL, ",");
	if (*p == 'x'){
		Kd = motorPID.GetKd();
	} else {
		Kd = atof(p);
	}

	motorPID.SetTunings(Kp,Ki,Kd);
}

void Motor::loop() {

	static uint32_t lastLoop = millis();
	uint32_t thisLoop = millis();

	if (thisLoop - lastLoop >= 10) {   // only every 10 ms.  One tick at full speed is like 8.5ms

		//  These are all the cases for where speed is under manual control
		if (targetSpeed == 0){
			stop();
			return;
		}
		else if (targetSpeed == 0x7FFFFFFF){
			driveForward();
			return;
		}
		else if (targetSpeed == 0xFFFFFFFF){
			driveBackward();
			return;
		}
		else if (targetSpeed == 0x7FFFFFFE) {
			drivePWM();
			return;
		}

//		/////   START HERE WE WILL CHANGE TO PID
//		int32_t curSpeed = getSpeed();
//
//		if (targetSpeed > 0){
//			if (curSpeed < targetSpeed) {
//				if (pwmSpeed < 255) {
//					pwmSpeed++;
//				}
//			} else if (curSpeed > targetSpeed) {
//				if (pwmSpeed > 0){
//					pwmSpeed--;
//				}
//			}
//		}
//		if (targetSpeed < 0) {
//			if (curSpeed > targetSpeed) {
//				if (pwmSpeed > -255) {
//					pwmSpeed--;
//				}
//			} else if (curSpeed < targetSpeed) {
//				if (pwmSpeed < 0) {
//					pwmSpeed++;
//				}
//			}
//		}
//		///////   END HERE WE WILL CHANGE FOR PID

		pidInput = getSpeed();
		pidSetpoint = targetSpeed;
		motorPID.Compute();
		pwmSpeed = pidOutput;

		lastLoop = thisLoop;
		drivePWM();
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
	drivePWM();
	targetSpeed = 0x7FFFFFFE;
}


int32_t Motor::getSpeed(){
	return encoder.getSpeed();
}


void Motor::setSpeed(int32_t aTarget){
	targetSpeed = aTarget;
	motorPID.SetMode(AUTOMATIC);
}

int16_t Motor::getPwmSpeed(){
	return pwmSpeed;
}

int32_t Motor::getTargetSpeed(){
	return targetSpeed;
}
