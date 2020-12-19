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
#include "EncoderInterface.h"
#include <PID_v1.h>

#define MOTOR_FORWARD 2
#define MOTOR_REVERSE 1
#define MOTOR_STOP 0


class Motor {

	uint8_t directionPin1;
	uint8_t directionPin2;
	uint8_t enablePin;
	uint8_t feedbackPin;


	uint8_t invertForward;  //HIGH / true for left LOW / false for right

	int32_t targetSpeed = 0;
	int32_t currentSpeed = 0;

	int16_t pwmSpeed = 0;

	uint8_t minPWM = 127;

	PID motorPID;
	double pidInput = 0.0;
	double pidOutput = 0.0;
	double pidSetpoint = 0.0;

	void drivePWM();



public:

	EncoderInterface encoder;

//	Motor(uint8_t aDirpin, uint8_t aEnabpin, boolean aInvert) : directionPin(aDirpin), enablePin(aEnabpin), invertForward(aInvert), encoder(NULL){};

	Motor(uint8_t aDirpin1, uint8_t aDirpin2, uint8_t aPwmpin,
			uint8_t aFeedbackPin, boolean aInvert) :
			directionPin1(aDirpin1), directionPin2(aDirpin2), enablePin(
					aPwmpin), feedbackPin(aFeedbackPin), invertForward(aInvert), motorPID(
					&pidInput, &pidOutput, &pidSetpoint, 6.0, 1.0, 0.0, DIRECT) {};

	void init();

	void setDirection(uint8_t);

	void driveForward();
	void driveBackward();
	void drive(int16_t);
	void stop();
	void coast();

	int32_t getSpeed();
	int16_t getPwmSpeed();
	int32_t getTargetSpeed();
	uint16_t getFeedback();

	void setMinPWM(uint8_t);
	uint8_t getMinPWM();

	void setSpeed(int32_t);
	void setTuningByString(char*);

	void loop();

};




#endif /* MOTOR_H_ */
