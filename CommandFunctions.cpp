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


//  'E' reserved for ESP board
Command commands[] = {
		{ 'X', xboxCommand },
		{ 'R', requestFromBot },
		{ 'M', motorControl },
		{ 'B', requestFromBot },
		{ 'H', headlightControl },
		{ 'V', videoControl },
		{ 'S', armControl },
		{ '#', armControl }
};

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
			Serial.print("<RMB HBoR>");
		}
		break;
	}

	case 'B':
	{
		int r = analogRead(BATTERY_PIN);
		float v = (r * 20.75) / 1024;
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
				digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, HIGH);
//				Serial.println("<Right-DIR LOW>");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, HIGH);
//				Serial.println("<Right-DIR HIGH>");
			} else {
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, LOW);
//				Serial.println("<Right Enable LOW>");
			}
		}
		else if (p[2] == 'L') {
			if (p[4] == '1') {
				digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, HIGH);
//				Serial.println("<Left-DIR LOW>");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, HIGH);
//				Serial.println("<Left-DIR HIGH>");
			} else {
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, LOW);
//				Serial.println("<Left Enable LOW>");
			}
		}
	}

}

void armControl(char* p) {

	if(armEnabled) {

		Serial1.print(p);

	}

}
