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

extern Robot robot;

extern int32_t leftCounter;
extern int32_t rightCounter;

//  'E' reserved for ESP board
Command commands[] = {
		{ 'X', xboxCommand },
		{ 'R', requestFromBot },
		{ 'T', setThrottle },
		{ 'M', motorControl },
		{ 'm', motorControl },
		{ 't', testFunc },
		{ 'B', requestFromBot },
		{ 'H', headlightControl },
		{ 'V', videoControl },
		{ 'S', armControl },
		{ 'A', armControl },
		{ '#', armControl },
		{ 'U', ultrasonicControl },
		{ 'D', autoDrive }
};

void autoDrive(char* p){
	int32_t dist = atol(p+2);
	robot.driveForward(dist);
}


void testFunc(char* p) {

	int32_t speed = atol(p + 4);
	if (p[2] == 'L') {
		robot.leftMotor.setSpeed(speed);
	}
	if (p[2] == 'R') {
		robot.rightMotor.setSpeed(speed);
	}

}


void ultrasonicControl(char *p) {
	switch (p[3]) {

	case 'S':
		robot.sonar.startPing();
		break;
	case 'P':
		if (p[4] == 's') {
			robot.sonar.gimbal.setPanSpeed(atoi((const char*) (p + 5)));
		} else {
			robot.sonar.gimbal.setPan(atoi((const char*) (p + 4)));
		}
		break;
	case 'T':
		if (p[4] == 's') {
			robot.sonar.gimbal.setTiltSpeed(atoi((const char*) (p + 5)));
		} else {
			robot.sonar.gimbal.setTilt(atoi((const char*) (p + 4)));
		}
		break;
	case 'X':
		robot.sonar.stopPing();
		break;
	case 'W':
		robot.sonar.startSweep();
		break;
	case 'C':
		if (p[4] == '0') {
			robot.sonar.setContinuous(false);
		} else {
			robot.sonar.setContinuous(true);
			robot.sonar.startSweep();
		}
		break;
	case 'D':
		if (p[4] == 's') {
			robot.sonar.setSweepDelay(atoi((const char*) (p + 5)));
		} else if (p[4] == 'h') {
			robot.sonar.setHoldDelay(atoi((const char*) (p + 5)));
		}
		break;
	case 'Z':
		robot.sonar.parkSensor();
		break;
	default:
		break;

	}
}


//Command armCommands[] = {
//		{ 'C', enableArm }
//};

CommandParser cp(&commands[0], NUM_ELEMENTS(commands), true);
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
	xbox.handleIncomingASCII(p + 2);

	if((robot.armResponding)&&(robot.getDriveMode() == ARM)){
		Serial1.print(p);
	}
	robot.regularResponse();

}

void xboxCommandRaw(char* p) {
	if(p[1] == 0x14 && p[2] == 16){
		// It's a real raw xbox command
		uint8_t temp[14];
		memcpy(temp, p+1, 14);
		temp[1] = 0x0D;  // xboxHandler Expects this
		xbox.handleIncoming(temp);
		if((robot.armResponding)&&(robot.getDriveMode() == ARM)){
			for(uint8_t i=0; i<16; i++){
				Serial1.write(p[i]);
			}
		}
		robot.regularResponse();
	}
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
//		digitalWrite(CAM_ENABLE, LOW);
		robot.camera.disable();
		break;
	case '1':
//		digitalWrite(CAM_ENABLE, HIGH);
		robot.camera.enable();
		break;
	default:
		break;

	}

}

void headlightControl(char* p) {

	switch (p[2]) {

	case '0':
//		digitalWrite(HEADLIGHT_PIN, LOW);
		robot.headlight.disable();
		break;
	case '1':
//		digitalWrite(HEADLIGHT_PIN, HIGH);
		robot.headlight.enable();
		break;
	default:
		break;

	}

}

void requestFromBot(char* p) {
	switch (p[3]) {

	case 'P':
		robot.sonar.startPing();
		break;

	case 'H':

		if (p[4] == 'B') {
			Serial.print(HBOR_STRING);
		}
		break;
	case 'G': {
		char gitbuf[9];
		strncpy(gitbuf, GIT_HASH, 8);
		gitbuf[8] = 0;
		Serial.print("<RMBGIT-");
		Serial.print(gitbuf);
		Serial.print(">");
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
		Serial.print(robot.leftMotor.getSpeed());
		Serial.print(",");
		Serial.print(robot.rightMotor.getSpeed());
		Serial.print(">");
		break;

	case 's':
		Serial.print("<Out,");
		Serial.print(robot.leftMotor.getPwmSpeed());
		Serial.print(",");
		Serial.print(robot.rightMotor.getPwmSpeed());
		Serial.print(">");
		break;

	case 'M':
		Serial.print("<Cnts,");
		Serial.print(leftCounter);
		Serial.print(",");
		Serial.print(rightCounter);
		Serial.print(">");

		Serial.print("<Spd,");
		Serial.print(robot.leftMotor.getSpeed());
		Serial.print(",");
		Serial.print(robot.rightMotor.getSpeed());
		Serial.print(">");

		Serial.print("<Out,");
		Serial.print(robot.leftMotor.getPwmSpeed());
		Serial.print(",");
		Serial.print(robot.rightMotor.getPwmSpeed());
		Serial.print(">");

		break;


	case 'B':
	{
//		int r = analogRead(BATTERY_PIN);
//		float v = (r * 20.75) / 1024;
		// Calibrated
		float v = robot.battery.getVoltage();
		int r = (v - 0.79690) / 0.020104;

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


void setThrottle(char* p) {
	// p[0] == '<'
	// p[1] == 'T'
	int setpt = atoi(p + 2);
	robot.setThrottle(setpt);
}


void motorControl(char* p) {

	if (p[1] == 'M') {
		if (p[2] == 'R') {
			if (p[4] == '1') {
				robot.rightMotor.driveForward();
			} else if (p[4] == '-' && p[5] == '1') {
				robot.rightMotor.driveBackward();
			} else {
				robot.rightMotor.stop();
			}
		}
		else if (p[2] == 'L') {
			if (p[4] == '1') {
				robot.leftMotor.driveForward();
			} else if (p[4] == '-' && p[5] == '1') {
				robot.leftMotor.driveBackward();
			} else {
				robot.leftMotor.stop();
			}
		}
	}
	else if (p[1] == 'm') {
		int amt = atoi(p + 4);

		if (p[2] == 'R') {
			robot.rightMotor.drive(amt);
		}
		else if (p[2] == 'L') {
			robot.leftMotor.drive(amt);
		}
	}



}

void armControl(char* p) {

	if(robot.armResponding) {

		Serial1.print(p);

	}

}
