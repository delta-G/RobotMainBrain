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


#include "CommandFunctions.h"

extern XboxHandler xbox;
extern Motor leftMotor;
extern Motor rightMotor;

extern int32_t leftCounter;
extern int32_t rightCounter;

extern float leftOut;
extern float rightOut;

//  'E' reserved for ESP board
Command commands[] = {
		{ 'X', xboxCommand },
		{ 'R', requestFromBot },
		{ 'M', motorControl },
		{ 'm', motorControl },
		{ 't', testFunc },
		{ 'B', requestFromBot },
		{ 'H', headlightControl },
		{ 'V', videoControl },
		{ 'S', armControl },
		{ '#', armControl }
};


void testFunc(char* p) {

	int32_t speed = atol(p + 4);
	if (p[2] == 'L') {
		leftMotor.setSpeed(speed);
	}
	if (p[2] == 'R') {
		rightMotor.setSpeed(speed);
	}

}


//Command armCommands[] = {
//		{ 'C', enableArm }
//};

CommandParser cp(&Serial, &commands[0], NUM_ELEMENTS(commands));
//CommandParser cpArm(&Serial1, &armCommands[0], NUM_ELEMENTS(armCommands));

/*
 *
 * These functions will receive the full command with packet markers intact.
 * Offset into these char arrays with that in mind.  p[0] will be '<' and
 * the last char will be '>'
 *
 */

bool armEnabled = true;

void xboxCommand(char* p) {
	xbox.handleIncomingASCII(p);
}

void enableArm(char* p) {
	if (p[3] == '0'){
		Serial.print("<Arm Responding>");
		armEnabled = true;
	}
}

void videoControl(char* p) {

	switch(p[2]){

	case '0':
		digitalWrite(CAM_ENABLE, LOW);
		break;
	case '1':
		digitalWrite(CAM_ENABLE, HIGH);
		break;
	default:
		break;

	}

}

void headlightControl(char* p) {

	switch (p[2]) {

	case '0':
		digitalWrite(HEADLIGHT_PIN, LOW);
		break;
	case '1':
		digitalWrite(HEADLIGHT_PIN, HIGH);
		break;
	default:
		break;

	}

}

void requestFromBot(char* p) {
	switch(p[3]){

	case 'H':
	{
		if(p[4] == 'B'){
			Serial.print(HBOR_STRING);
		}
		break;
	}




	case 'C':
		Serial.print("<Cnts,");
		Serial.print(leftCounter);
		Serial.print(",");
		Serial.print(rightCounter);
		Serial.print(">");
		break;


	case 'S':
		Serial.print("<Spd,");
		Serial.print(leftMotor.getSpeed());
		Serial.print(",");
		Serial.print(rightMotor.getSpeed());
		Serial.print(">");
		break;

	case 's':
		Serial.print("<Out,");
		Serial.print(leftMotor.getPwmSpeed());
		Serial.print(",");
		Serial.print(rightMotor.getPwmSpeed());
		Serial.print(">");
		break;

	case 'M':
		Serial.print("<Cnts,");
		Serial.print(leftCounter);
		Serial.print(",");
		Serial.print(rightCounter);
		Serial.print(">");

		Serial.print("<Spd,");
		Serial.print(leftMotor.getSpeed());
		Serial.print(",");
		Serial.print(rightMotor.getSpeed());
		Serial.print(">");

		Serial.print("<Out,");
		Serial.print(leftMotor.getPwmSpeed());
		Serial.print(",");
		Serial.print(rightMotor.getPwmSpeed());
		Serial.print(">");

		break;


	case 'B':
	{
		int r = analogRead(BATTERY_PIN);
//		float v = (r * 20.75) / 1024;
		// Calibrated
		float v = (r * 0.020104) + 0.79690;
		Serial.print("<BAT,");
		Serial.print(r);
		Serial.print(",");
		Serial.print(v, 1);
		Serial.print(">");
		break;
	}

	default:
	{
		break;
	}

	}

}

void motorControl(char* p) {

	if (p[1] == 'M') {
		if (p[2] == 'R') {
			if (p[4] == '1') {
				rightMotor.driveForward();
//				Serial.println("<Right-DIR LOW>");
			} else if (p[4] == '-' && p[5] == '1') {
				rightMotor.driveBackward();
//				Serial.println("<Right-DIR HIGH>");
			} else {
				rightMotor.stop();
//				Serial.println("<Right Enable LOW>");
			}
		}
		else if (p[2] == 'L') {
			if (p[4] == '1') {
				leftMotor.driveForward();
//				Serial.println("<Left-DIR LOW>");
			} else if (p[4] == '-' && p[5] == '1') {
				leftMotor.driveBackward();
//				Serial.println("<Left-DIR HIGH>");
			} else {
				leftMotor.stop();
//				Serial.println("<Left Enable LOW>");
			}
		}
	}
	else if (p[1] == 'm') {
		int amt = atoi(p + 4);

		if (p[2] == 'R') {
			rightMotor.drive(amt);
		}
		else if (p[2] == 'L') {
			leftMotor.drive(amt);
		}
	}



}

void armControl(char* p) {

	if(armEnabled) {

		Serial1.print(p);

	}

}
