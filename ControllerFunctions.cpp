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
				started = true;
				runStartup();
			}
		}

		//BACK
		if (xbox_ptr->isClicked(BACK)) {
			if (started) {
				started = false;
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
					outStream->println("Drive Mode");
				} else if (controlMode == ARM) {
					outStream->println("Arm Mode");
				} else if (controlMode == MINE) {
					outStream->println("Mine Mode");
				}
			}

			switch (controlMode){

			case DRIVE:
				driveWithTwoSticks();
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

	outStream->println("RMB Starting Interface");

}

void returnControl(){

	// Need to let Main Brain know he is back in control

	outStream->println("RMB Xbox Releasing");

}

void driveWithTwoSticks() {

	int16_t leftVal = xbox_ptr->getHatValue(LeftHatY);
	int16_t rightVal = xbox_ptr->getHatValue(RightHatY);

	if (leftVal > DEFAULT_DEADZONE) {
		leftMotor_ptr->driveForward();
	} else if (leftVal < -DEFAULT_DEADZONE) {
		leftMotor_ptr->driveBackward();
	} else {
		leftMotor_ptr->stop();
	}

	if (rightVal > DEFAULT_DEADZONE) {
		rightMotor_ptr->driveForward();
	} else if (rightVal < -DEFAULT_DEADZONE) {
		rightMotor_ptr->driveBackward();
	} else {
		rightMotor_ptr->stop();
	}

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


