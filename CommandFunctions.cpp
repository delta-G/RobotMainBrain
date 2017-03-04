/*
 * CommandFunctions.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: david
 */
#include "CommandFunctions.h"

Command commands[] = {
		{ 'R', requestFromBot },
		{ 'M', motorControl },
		{ 'S', armControl },
		{ '#', armControl }
};

CommandParser cp(&Serial, &commands[0], NUM_ELEMENTS(commands));

/*
 *
 * These functions will receive the full command with packet markers intact
 * offset into these char arrays with that in mind.  p[0] will be '<' and
 * the last char will be '>'
 *
 */

void requestFromBot(char* p) {

}

void motorControl(char* p) {

	if (p[1] == 'M') {
		if (p[2] == 'R') {
			if (p[4] == '1') {
				digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, HIGH);
				Serial.println("Right-LHH");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, HIGH);
				Serial.println("Right-HLH");
			} else {
				digitalWrite(RIGHT_MOTOR_ENABLE_PIN, LOW);
				Serial.println("Right---L");
			}
		}
		else if (p[2] == 'L') {
			if (p[4] == '1') {
				digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, HIGH);
				Serial.println("Left-HLH");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, HIGH);
				Serial.println("Left-LHH");
			} else {
				digitalWrite(LEFT_MOTOR_ENABLE_PIN, LOW);
				Serial.println("Left---L");
			}
		}
	}

}

void armControl(char* p) {

}
