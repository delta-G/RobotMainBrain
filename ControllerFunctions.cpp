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


XboxHandler* xbox_ptr;

Stream* outStream;

Stream* servoStream;

//  This should be a pointer to a Robot so I can control everything.
Robot* robot_ptr;

boolean started = false;

unsigned int updateInterval = 20;


float leftOut = 0.0;
float rightOut = 0.0;


void initializeControllerFunctions(Robot* aRobot, Stream* aOutStream, Stream* aServStream, XboxHandler* aXbox){

	robot_ptr = aRobot;
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

		//TODO::
		//  Does this section really need to be timed if the controller data is coming to it at some given rate already?
		//  IF anything it should be if newData, but why can't it run all the time.  Maybe some of these functions
		//  need to constant update someday.

		if (currentRunTime - previousRunTime >= updateInterval) {
			previousRunTime = currentRunTime;

			if (xbox_ptr->isClicked(Y)) {
				robot_ptr->advanceDriveMode();
			}

			switch (robot_ptr->getDriveMode()){

			case DRIVE:
				driveWithTwoSticks();
				break;
			case ARM:
				driveByDpad();
				break;
			case MINE:
				driveWithOneStickAlg2();
				int panVal = xbox_ptr->getHatValue(LeftHatX);
				int tiltVal = xbox_ptr->getHatValue(LeftHatY);
				Serial1.print("<A,S6,J");
				Serial1.print(-panVal);
				Serial1.print(">");
				Serial1.print("<A,S7,J");
				Serial1.print(-tiltVal);
				Serial1.print(">");

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

	robot_ptr->leftMotor.drive(leftOutput);
	robot_ptr->rightMotor.drive(rightOutput);
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

	robot_ptr->leftMotor.setSpeed(leftOutput);
	robot_ptr->rightMotor.setSpeed(rightOutput);
}

void driveByDpad() {

	if (xbox_ptr->isPressed(UP)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			robot_ptr->leftMotor.driveForward();
			robot_ptr->rightMotor.stop();
		} else if (xbox_ptr->isPressed(LEFT)) {
			robot_ptr->leftMotor.stop();
			robot_ptr->rightMotor.driveForward();
		} else {
			robot_ptr->leftMotor.driveForward();
			robot_ptr->rightMotor.driveForward();
		}
	} else if (xbox_ptr->isPressed(DOWN)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			robot_ptr->rightMotor.driveBackward();
			robot_ptr->leftMotor.stop();
		} else if (xbox_ptr->isPressed(LEFT)) {
			robot_ptr->rightMotor.stop();
			robot_ptr->leftMotor.driveBackward();
		} else {
			robot_ptr->leftMotor.driveBackward();
			robot_ptr->rightMotor.driveBackward();
		}
	} else if (xbox_ptr->isPressed(LEFT)) {
		robot_ptr->leftMotor.driveBackward();
		robot_ptr->rightMotor.driveForward();
	} else if (xbox_ptr->isPressed(RIGHT)) {
		robot_ptr->rightMotor.driveBackward();
		robot_ptr->leftMotor.driveForward();
	} else {
		robot_ptr->rightMotor.stop();
		robot_ptr->leftMotor.stop();
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

	robot_ptr->leftMotor.drive(leftOut);
	robot_ptr->rightMotor.drive(rightOut);

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
	robot_ptr->leftMotor.drive((int16_t)leftOut);
	robot_ptr->rightMotor.drive((int16_t)rightOut);

}
