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
extern boolean commandTimeoutOverride;
extern boolean useSpeedBasedAlgs;

extern boolean runSpeedReport;


extern int32_t leftCounter;
extern int32_t rightCounter;

//  'E' reserved for ESP board
Command commands[] = {
		// 'p' and 'P' and 'l' and 'E' are used by radios
		{ 'X', xboxCommand },
		{ 'R', requestFromBot },
		{ 'C', configureBot },
		{ 'T', setThrottle },
		{ 'Q', powerControl},
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


CommandParser cp(&commands[0], NUM_ELEMENTS(commands), true);

/*
 *
 * These functions will receive the full command with packet markers intact.
 * Offset into these char arrays with that in mind.  p[0] will be '<' and
 * the last char will be '>'
 *
 */
void powerControl(char* p){
	switch(p[2]){
	case 'R':
		if(p[3] == '0'){
			robot.comPower.disable();
		} else if (p[3] == '1'){
			robot.comPower.enable();
		}
		break;
	case 'A':
		if(p[3] == '0'){
			robot.arm.disable();
		} else if (p[3] == '1'){
			robot.silenceHeartbeat();
			delay(10);
			robot.arm.enable();
			delay(10);
			robot.restartHeartbeat();
		}
		break;
	case 'a':
		if(p[3] == '0'){
			robot.auxPower.disable();
		} else if (p[3] == '1'){
			robot.auxPower.enable();
		}
		break;
	case 'M':
		if(p[3] == '0'){
			robot.motorPower.disable();
		} else if (p[3] == '1'){
			robot.motorPower.enable();
		}
		break;
	case 'm':
		if(p[3] == '0'){
			robot.motorController.disable();
		} else if (p[3] == '1'){
			robot.motorController.enable();
		}
		break;
	case 'V':
		if(p[3] == '0'){
			robot.v12Power.disable();
		} else if (p[3] == '1'){
			robot.v12Power.enable();
		}
		break;
	case 'S':
		if(p[3] == '0'){
			robot.sonarPower.disable();
		} else if (p[3] == '1'){
			robot.sonarPower.enable();
		}
		break;
	}
}

void autoDrive(char* p){

	switch(p[2]){
	case 'L':{
		int32_t dist = atol(p+3);
		robot.driveTicks(dist,0);
		break;
	}
	case 'R':{
		int32_t dist = atol(p+3);
		robot.driveTicks(0,dist);
		break;
	}
	case 'D':{
		int32_t distL = atol(p+3);
		char* commaPointer = strchr(p, ',');
		int32_t distR = atol(commaPointer + 1);
		robot.driveTicks(distL,distR);
		break;
	}
	case 'W':
		robot.startRandomWalk();
		break;
	case '0'...'9':{
		int32_t dist = atol(p+2);
		robot.driveTicks(dist,dist);
		break;
	}
	}
}


void testFunc(char* p) {

	Serial.print(F("<RMB_RESPONDING>"));

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


void configureBot(char *p) {

	switch (p[2]) {
	case 'A':
		if (p[3] == '1') {
			useSpeedBasedAlgs = true;
		} else if (p[3] == '0') {
			useSpeedBasedAlgs = false;
		}
		break;
	case 'P': {  // set CS0[2:0] to set PWM frequency on Timer0
		byte val = TCCR0B;
		val &= !7;  // clear bottom three bits (CS0[2:0])
		if (p[3] == '1') {
			val |= (1<<CS02);
		}
		if (p[4] == '1') {
			val |= (1<<CS01);
		}
		if (p[5] == '1') {
			val |= (1<<CS00);
		}
		TCCR0B = val;
		break;
	}
	case 'O':  // override
		if (p[3] == '1') {
			commandTimeoutOverride = true;
		} else if (p[3] == '0') {
			commandTimeoutOverride = false;
		}
		break;
	case 'm': {  // set minPWM
		int setpt = atoi(p+3);
		robot.setMinPWM(setpt);
		break;
	}

	default:
		break;
	}
}


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
		memcpy(robot.lastRawCommand, p+1, XBOX_RAW_BUFFER_SIZE);
		robot.regularResponse();
	}
}

void enableArm(char* p) {
	if (p[3] == '0'){
		Serial.print(F("<Arm Responding>"));
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
	case 'r':
		if (p[4] == '1') {
			runSpeedReport = true;
		} else if (p[4] == '0') {
			runSpeedReport = false;
		}
		break;
	case 'R':
		robot.regularResponse();
		break;
	case 'E':  // Echo
		Serial.print(F("<"));
		Serial.print(p+5);
		break;
	case 'P':
		robot.sonar.startPing();
		break;
	case 'F': // flush radio
		Serial.print(F("<FFF>"));
		break;
	case 'H':
		if (p[4] == 'B') {
			Serial.print(F(HBOR_STRING));
		}
		break;
	case 'G': {
		char gitbuf[9];
		strncpy(gitbuf, GIT_HASH, 8);
		gitbuf[8] = 0;
		Serial.print(F("<RMBGIT-"));
		Serial.print(gitbuf);
		Serial.print(F(">"));
		break;
	}

	case 'C':
		Serial.print(F("<Cnts,"));
		Serial.print(leftCounter);
		Serial.print(F(","));
		Serial.print(rightCounter);
		Serial.print(F(">"));
		break;


	case 'S':
		Serial.print(F("<Spd,"));
		Serial.print(robot.leftMotor.getSpeed());
		Serial.print(F(","));
		Serial.print(robot.rightMotor.getSpeed());
		Serial.print(F(">"));
		break;

	case 's':
		Serial.print(F("<Out,"));
		Serial.print(robot.leftMotor.getPwmSpeed());
		Serial.print(F(","));
		Serial.print(robot.rightMotor.getPwmSpeed());
		Serial.print(F(">"));
		break;

	case 'M':
		Serial.print(F("<Cnts,"));
		Serial.print(leftCounter);
		Serial.print(F(","));
		Serial.print(rightCounter);
		Serial.print(F(">"));

		Serial.print(F("<Spd,"));
		Serial.print(robot.leftMotor.getSpeed());
		Serial.print(F(","));
		Serial.print(robot.rightMotor.getSpeed());
		Serial.print(F(">"));

		Serial.print(F("<Out,"));
		Serial.print(robot.leftMotor.getPwmSpeed());
		Serial.print(F(","));
		Serial.print(robot.rightMotor.getPwmSpeed());
		Serial.print(F(">"));

		break;


	case 'B':
	{
//		int r = analogRead(BATTERY_PIN);
//		float v = (r * 20.75) / 1024;
		// Calibrated
		float v = robot.battery.getVoltage();
		int r = (v - 0.79690) / 0.020104;

		Serial.print(F("<BAT,"));
		Serial.print(r);
		Serial.print(F(","));
		Serial.print(v, 1);
		Serial.print(F(">"));
		break;
	}
	case 'V':
		robot.readSupplyVoltages();
		break;
	case 'v':
		robot.reportSupplyVoltages();
		break;

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
	if (p[1] == 'S') {
		if(robot.checkMotorStatus()){
			Serial.print(F("<Motor Fail Bit Error>"));
		}
	}

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
		else if (p[2] == 'P') {
			robot.leftMotor.setTuningByString(p+4);
			robot.rightMotor.setTuningByString(p+4);
		}
	}
	else if (p[1] == 'm') {
		long amt = atol(p + 4);

		if (p[2] == 'R') {
			robot.rightMotor.setSpeed(amt);
		}
		else if (p[2] == 'L') {
			robot.leftMotor.setSpeed(amt);
		}
	}
}

void armControl(char* p) {

	if(robot.armResponding) {

		Serial1.print(p);

	}

}
