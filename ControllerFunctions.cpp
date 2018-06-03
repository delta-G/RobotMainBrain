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

#include "ControllerFunctions.h"


XboxHandler* xbox_ptr;   /// This is also defined in CommandFunctions.  MUST be FIXED BEFORE COMPILE

Stream* outStream;

Stream* servoStream;

Motor* leftMotor_ptr;
Motor* rightMotor_ptr;

boolean started = false;

unsigned int updateInterval = 20;

int controlMode;


float leftOut = 0.0;
float rightOut = 0.0;


void initializeControllerFunctions(Motor* aLeftMotor, Motor* aRightMotor, Stream* aOutStream, Stream* aServStream, XboxHandler* aXbox){

	leftMotor_ptr = aLeftMotor;
	rightMotor_ptr = aRightMotor;
	outStream = aOutStream;
	servoStream = aServStream;
	xbox_ptr = aXbox;

}


void mainControllerLoop() {

	static unsigned long previousRunTime = millis();
	unsigned long currentRunTime = millis();

	if (xbox_ptr->newDataAvailable()) {

		// START
		if (xbox_ptr->isClicked(START)) {
			if (!started) {

				runStartup();
			}
		}

		//BACK
		if (xbox_ptr->isClicked(BACK)) {
			if (started) {

				returnControl();
			}
		}

		//  Timed Section

		if (currentRunTime - previousRunTime >= updateInterval) {
			previousRunTime = currentRunTime;

			if (xbox_ptr->isClicked(Y)) {
				controlMode++;
				controlMode %= NUMBER_OF_MODES;

				if (controlMode == DRIVE) {
					outStream->println("<Drive Mode>");
				} else if (controlMode == ARM) {
					outStream->println("<Arm Mode>");
				} else if (controlMode == MINE) {
					outStream->println("<Mine Mode>");
				}
			}

			switch (controlMode){

			case DRIVE:
				driveWithTwoSticksAlg2();
				leftMotor_ptr->loop();
				rightMotor_ptr->loop();
				break;
			case ARM:
				driveByDpad();
				break;

			}
		}
	}
}


void runStartup(){
	// Need to let Main Brain know we have control this way
	started = true;
	outStream->println("<RMB Start Iface>");
}

void returnControl(){
	// Need to let Main Brain know he is back in control
	started = false;
	outStream->println("<RMB Iface End>");
}

void driveWithTwoSticks() {

	int16_t leftVal = xbox_ptr->getHatValue(LeftHatY);
	int16_t rightVal = xbox_ptr->getHatValue(RightHatY);

	int16_t leftOutput = 0;
	int16_t rightOutput = 0;

	if (abs(leftVal) > DEFAULT_DEADZONE) {
		leftOutput = map(leftVal, -32768, 32767, -255, 255);
		if (abs(leftOutput) < 127) {
			leftOutput = 0;
		}
	}
	if (abs(rightVal) > DEFAULT_DEADZONE) {
		rightOutput = map(rightVal, -32768, 32767, -255, 255);
		if (abs(rightOutput) < 127) {
			rightOutput = 0;
		}
	}

	leftMotor_ptr->drive(leftOutput);
	rightMotor_ptr->drive(rightOutput);
}


void driveWithTwoSticksAlg2() {

	int16_t leftVal = xbox_ptr->getHatValue(LeftHatY);
	int16_t rightVal = xbox_ptr->getHatValue(RightHatY);

	int16_t leftOutput = 0;
	int16_t rightOutput = 0;

	if (abs(leftVal) > DEFAULT_DEADZONE) {
		leftOutput = map(leftVal, -32768, 32767, -100, 100);
		if (abs(leftOutput) < 30) {
			leftOutput = 0;
		}
	}
	if (abs(rightVal) > DEFAULT_DEADZONE) {
		rightOutput = map(rightVal, -32768, 32767, -100, 100);
		if (abs(rightOutput) < 30) {
			rightOutput = 0;
		}
	}

	leftMotor_ptr->setSpeed(leftOutput);
	rightMotor_ptr->setSpeed(rightOutput);
}



void driveByDpad() {

	if (xbox_ptr->isPressed(UP)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			leftMotor_ptr->driveForward();
			rightMotor_ptr->stop();
		} else if (xbox_ptr->isPressed(LEFT)) {
			leftMotor_ptr->stop();
			rightMotor_ptr->driveForward();
		} else {
			leftMotor_ptr->driveForward();
			rightMotor_ptr->driveForward();
		}
	} else if (xbox_ptr->isPressed(DOWN)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			rightMotor_ptr->driveBackward();
			leftMotor_ptr->stop();
		} else if (xbox_ptr->isPressed(LEFT)) {
			rightMotor_ptr->stop();
			leftMotor_ptr->driveBackward();
		} else {
			leftMotor_ptr->driveBackward();
			rightMotor_ptr->driveBackward();
		}
	} else if (xbox_ptr->isPressed(LEFT)) {
		leftMotor_ptr->driveBackward();
		rightMotor_ptr->driveForward();
	} else if (xbox_ptr->isPressed(RIGHT)) {
		rightMotor_ptr->driveBackward();
		leftMotor_ptr->driveForward();
	} else {
		rightMotor_ptr->stop();
		leftMotor_ptr->stop();
	}
}

void driveWithOneStick() {

	int16_t xVal = xbox_ptr->getHatValue(RightHatX);
	int16_t yVal = xbox_ptr->getHatValue(RightHatY);

	float s = 0.707107;  // at pi/4 sin == cos =~= 0.707107

	//  rotation matrix
	float xRot = (xVal * s) - (yVal * s);
	float yRot = (xVal * s) + (yVal * s);
	//  Now left motor lies along y axis and right motor along x axis.

	leftOut = (yRot / 32768) * 255;
	if (abs(leftOut) < 127) {
		leftOut = 0;
	}
	if (leftOut > 255) leftOut = 255;
	if (leftOut < -255) leftOut = -255;


	rightOut = (xRot / 32768) * -255;
	if (abs(rightOut) < 127) {
		rightOut = 0;
	}
	if (rightOut > 255) rightOut = 255;
		if (rightOut < -255) rightOut = -255;

	leftMotor_ptr->drive(leftOut);
	rightMotor_ptr->drive(rightOut);

}



void driveWithOneStickAlg2() {

	int16_t xVal = xbox_ptr->getHatValue(RightHatX);
	int16_t yVal = xbox_ptr->getHatValue(RightHatY);

	float s = 0.707107;  // at pi/4 sin == cos =~= 0.707107

	//  rotation matrix
	float xRot = (xVal * s) - (yVal * s);
	float yRot = (xVal * s) + (yVal * s);
	//  Now left motor lies along y axis and right motor along x axis.

	leftOut = (yRot / 32768.0) * 255.0;
	rightOut = (xRot / 32768.0) * -255.0;

	if (abs(rightOut) > abs(leftOut)) {
		leftOut = 255 * (leftOut / rightOut);
		rightOut = (rightOut < 0) ? -255 : 255;
	} else if (abs(leftOut) > abs(rightOut)) {
		rightOut = 255 * (rightOut / leftOut);
		leftOut = (leftOut < 0) ? -255 : 255;
	} else {
		rightOut = (rightOut < 0) ? -255 : (rightOut > 0) ? 255 : 0;
		leftOut = (leftOut < 0) ? -255 : (leftOut > 0) ? 255 : 0;
	}

	///  Some bounds Checking
	if (abs(leftOut) < 127) {
		leftOut = 0;
	}
	if (leftOut > 255) {
		leftOut = 255;
	}
	if (leftOut < -255) {
		leftOut = -255;
	}

	if (abs(rightOut) < 127) {
		rightOut = 0;
	}
	if (rightOut > 255) {
		rightOut = 255;
	}
	if (rightOut < -255) {
		rightOut = -255;
	}

	///  Write to the motors.
	leftMotor_ptr->drive((int16_t)leftOut);
	rightMotor_ptr->drive((int16_t)rightOut);

}
