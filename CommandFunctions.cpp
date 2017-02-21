/*
 * CommandFunctions.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: david
 */
#include "CommandFunctions.h"

const int motor_R_Enable = 5;
const int motorOut1 = 6;
const int motorOut2 = 7;

const int motor_L_Enable = 10;
const int motorOut3 = 8;
const int motorOut4 = 9;

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
				digitalWrite(motorOut1, LOW);
				digitalWrite(motorOut2, HIGH);
				digitalWrite(motor_R_Enable, HIGH);
				Serial.println("Right-LHH");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(motorOut1, HIGH);
				digitalWrite(motorOut2, LOW);
				digitalWrite(motor_R_Enable, HIGH);
				Serial.println("Right-HLH");
			} else {
				digitalWrite(motor_R_Enable, LOW);
				Serial.println("Right---L");
			}
		}
		else if (p[2] == 'L') {
			if (p[4] == '1') {
				digitalWrite(motorOut3, HIGH);
				digitalWrite(motorOut4, LOW);
				digitalWrite(motor_L_Enable, HIGH);
				Serial.println("Left-HLH");
			} else if (p[4] == '-' && p[5] == '1') {
				digitalWrite(motorOut3, LOW);
				digitalWrite(motorOut4, HIGH);
				digitalWrite(motor_L_Enable, HIGH);
				Serial.println("Left-LHH");
			} else {
				digitalWrite(motor_L_Enable, LOW);
				Serial.println("Left---L");
			}
		}
	}

}

void armControl(char* p) {

}
