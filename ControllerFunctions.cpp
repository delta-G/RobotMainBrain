/*
 * ControllerFunctions.cpp
 *
 *  Created on: Jun 17, 2017
 *      Author: david
 */

#include "ControllerFunctions.h"


XboxHandler xbox;

Stream* outStream;
Motor* leftMotor;
Motor* rightMotor;

boolean started = false;

unsigned int updateInterval = 20;

void mainControllerLoop() {

	static unsigned long previousRunTime = millis();
	unsigned long currentRunTime = millis();

	if (xbox.newDataAvailable()) {

		// START
		if (xbox.isClicked(START)) {
			if (!started) {
				started = true;
				runStartup();
			}
		}

		//BACK
		if (xbox.isClicked(BACK)) {
			if (started) {
				started = false;
				returnControl();
			}
		}

		//  Timed Section

		if (currentRunTime - previousRunTime >= updateInterval) {
			previousRunTime = currentRunTime;

			if (xbox.isClicked(Y)) {
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

			if (controlMode == DRIVE) {
				driveWithTwoSticks();
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

	int16_t leftVal = xbox.getHatValue(LeftHatY);
	int16_t rightVal = xbox.getHatValue(RightHatY);

	if (leftVal > DEFAULT_DEADZONE) {
		leftMotor->driveForward();
	} else if (leftVal < -DEFAULT_DEADZONE) {
		leftMotor->driveBackward();
	}

	if (rightVal > DEFAULT_DEADZONE) {
		rightMotor->driveForward();
	} else if (rightVal < -DEFAULT_DEADZONE) {
		rightMotor->driveBackward();
	}

}


