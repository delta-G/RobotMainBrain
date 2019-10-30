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

boolean armButtonModeActive = false;
boolean robotButtonModeActive = false;

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
		// XBOX
		if (xbox_ptr->isClicked(XBOX)) {
			// This button is for DiscoBot
			// it means python is using the controller
			return;
		}

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

		if (xbox_ptr->isClicked(Y)) {
			robot_ptr->advanceDriveMode();
		}

		switch (robot_ptr->getDriveMode()) {

		case DRIVE:{
			static boolean stickMode = true;
			if (xbox_ptr->isPressed(A)) {
				armButtonMode();
			} else if (xbox_ptr->isPressed(B)) {
				robotButtonMode();
			} else {
				if(xbox_ptr->isClicked(X)){
					stickMode = !stickMode;
				}
				armButtonModeActive = false;
				robotButtonModeActive = false;
				dpadPanAndTilt();
				if(stickMode){
				driveWithTwoSticks();
				} else {
					driveWithTwoBrakes();
				}
			}
		}
			break;
		case ARM:
			driveByDpad();
			break;
		case MINE: {
			if (xbox_ptr->isPressed(A)) {
				armButtonMode();
			} else if (xbox_ptr->isPressed(B)){
				robotButtonMode();
			} else {
				robotButtonModeActive = false;
				armButtonModeActive = false;
				driveWithOneStickAlg2(xbox_ptr->getHatValue(RightHatX),
									xbox_ptr->getHatValue(RightHatY));
				panTiltStick(-xbox_ptr->getHatValue(LeftHatX) , -xbox_ptr->getHatValue(LeftHatY));
			}

			break;
		}
		default: {
			break;
		}
		}

	}
}

void panTiltStick(int aPan, int aTilt) {
	static char followOrUse = 'J';
	int panVal = aPan;
	int tiltVal = aTilt;

	if (xbox_ptr->isClicked(L3)) {
		if (followOrUse == 'J') {
			followOrUse = 'F';
		} else if (followOrUse == 'F') {
			followOrUse = 'J';
		}
	}

	Serial1.print("<A,S6,");
	Serial1.print(followOrUse);
	Serial1.print(panVal);
	Serial1.print(",S7,");
	Serial1.print(followOrUse);
	Serial1.print(tiltVal);
	Serial1.print(">");
}

void robotButtonMode(){
	if(!robotButtonModeActive){
		xbox_ptr->clear();
		robotButtonModeActive = true;
	}
	if (xbox_ptr->isClicked(L1)){
		robot_ptr->headlight.toggle();
	}

	int throt = (int)robot_ptr->getThrottle() + (xbox_ptr->getHatValue(LeftHatY) / 3276);
	if(xbox_ptr->isPressed(UP)){
		throt = throt + 1;
	}
	if(xbox_ptr->isPressed(DOWN)){
		throt = throt - 1;
	}

	// check for rollover
	if(throt < 0){
		throt = 0;
	} else if (throt > 255){
		throt = 255;
	}
	robot_ptr->setThrottle(throt);
}

void armButtonMode() {
	if(!armButtonModeActive){
		xbox_ptr->clear();
		armButtonModeActive = true;
	}
	if (xbox_ptr->isClicked(UP)) {
		Serial1.print("<A,CV256>"); // Standing Up Looking Forward
	} else if (xbox_ptr->isClicked(LEFT)) {
		Serial1.print("<A,CV352>");   // Forward and low
	} else if (xbox_ptr->isClicked(DOWN)) {
		Serial1.print("<A,CV320>");   ///  Forward and as low as possible
	} else if (xbox_ptr->isClicked(RIGHT)) {
		Serial1.print("<A,CV288>");   ///  Sitting Scorpion Pose
	}
	if (xbox_ptr->isPressed(L1)) {
		Serial1.print("<A,S0,a200>");
	}
	else if (xbox_ptr->isPressed(R1)) {
		Serial1.print("<A,S0,a-200>");
	}
	else {
		Serial1.print("<A,S0,J0>");
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

void dpadPanAndTilt() {

	if (xbox_ptr->isPressed(UP)) {
		Serial1.print("<A,S7,a");
		Serial1.print(-200);
		Serial1.print(">");
	} else if (xbox_ptr->isPressed(DOWN)) {
		Serial1.print("<A,S7,a");
		Serial1.print(200);
		Serial1.print(">");
	}

	if (xbox_ptr->isPressed(RIGHT)) {
		Serial1.print("<A,S6,a");
		Serial1.print(-200);
		Serial1.print(">");
	} else if (xbox_ptr->isPressed(LEFT)) {
		Serial1.print("<A,S6,a");
		Serial1.print(200);
		Serial1.print(">");
	}
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

	robot_ptr->drive(leftOutput,rightOutput);
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

	robot_ptr->setSpeed(leftOutput,rightOutput);
}

void driveByDpad() {

	if (xbox_ptr->isPressed(UP)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			robot_ptr->drive(255,0);
		} else if (xbox_ptr->isPressed(LEFT)) {
			robot_ptr->drive(0,255);
		} else {
			robot_ptr->driveForward();
		}
	} else if (xbox_ptr->isPressed(DOWN)) {
		if (xbox_ptr->isPressed(RIGHT)) {
			robot_ptr->drive(-255,0);
		} else if (xbox_ptr->isPressed(LEFT)) {
			robot_ptr->drive(0,-255);
		} else {
			robot_ptr->driveBackward();
		}
	} else if (xbox_ptr->isPressed(LEFT)) {
		robot_ptr->spinLeft();
	} else if (xbox_ptr->isPressed(RIGHT)) {
		robot_ptr->spinRight();
	} else {
		robot_ptr->stop();
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

	robot_ptr->drive(leftOut, rightOut);

}

void driveWithOneStickAlg2(int aXval, int aYval) {

	int16_t xVal = aXval;
	int16_t yVal = aYval;

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
	robot_ptr->drive((int16_t)leftOut,(int16_t)rightOut);
}


void driveWithTwoBrakes(){

	int leftBrake = xbox_ptr->getTriggerValue(L2);
	int rightBrake = xbox_ptr->getTriggerValue(R2);

	robot_ptr->drive(255 - leftBrake, 255 - rightBrake);
}
