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

#include "RobotMainBrain.h"

enum States { BOOTING, BOOT_ARM, CONNECT_COM, CONNECT_WAIT, RUNNING, NUM_RMB_STATES } currentState;

extern CommandParser cp;



unsigned int heartbeatInterval = 200;

XboxHandler xbox;

Motor leftMotor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_ENABLE_PIN, true);
Motor rightMotor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_ENABLE_PIN, false);

float batteryVoltage = 0.0;

void setup() {


	// Start with everything off

	pinMode(COM_POWER_ENABLE, OUTPUT);
	digitalWrite(COM_POWER_ENABLE, HIGH);

	pinMode(ARM_ENABLE, OUTPUT);
	digitalWrite(ARM_ENABLE, LOW);

	pinMode(CAM_ENABLE, OUTPUT);
	digitalWrite(CAM_ENABLE, LOW);

	pinMode(HEADLIGHT_PIN, OUTPUT);
	digitalWrite(HEADLIGHT_PIN, LOW);

	leftMotor.init();
	rightMotor.init();

	pinMode(HEARTBEAT_PIN, OUTPUT);
	for (int i = 0; i < 3; i++) {
		digitalWrite(HEARTBEAT_PIN, HIGH);
		delay(100);
		digitalWrite(HEARTBEAT_PIN, LOW);
		delay(100);
	}

	analogReference(INTERNAL1V1);

	initializeControllerFunctions(&leftMotor, &rightMotor, &Serial, &Serial1,
				&xbox);

	setupPCint();

}


void loop() {

	heartBeat();

	switch (currentState) {

	//   3 seconds at bootup for power to stabilize etc.
	case BOOTING:

		if (millis() > 3000) {
			currentState = BOOT_ARM;
		}
		break;

	case BOOT_ARM: {
		static boolean enteredArmState = false;
		static unsigned long armStartTime = 0;
		static boolean startedArmCom = false;
		if (!enteredArmState) {
			digitalWrite(ARM_ENABLE, HIGH);
			heartbeatInterval = 500;
			enteredArmState = true;
			armStartTime = millis();
		}

		if (!startedArmCom && (millis() - armStartTime >= 1000)) {
			//  Begin Serial on Arm Controller
			Serial1.begin(ARM_BOARD_BAUD);
			startedArmCom = true;
		}

		if (millis() - armStartTime >= 7000) {
			currentState = CONNECT_COM;
		}

		break;
	}

	case CONNECT_COM:
	{
		static boolean enteredComState = false;
		static unsigned long comStartedTime = 0;
		if(!enteredComState){
			Serial.begin(ROBOT_COM_BAUD);
			enteredComState = true;
			comStartedTime = millis();
		}

		if ((millis() - comStartedTime > 250)) {
			Serial.print(RMB_STARTUP_STRING);
			char gitbuf[9];
			strncpy(gitbuf, GIT_HASH, 8);
			gitbuf[8] = 0;
			Serial.print("<RMBGIT-");
			Serial.print(gitbuf);
			Serial.print(">");
			currentState = CONNECT_WAIT;
			heartbeatInterval = 2000;
		}

		//  TODO:  Why does cp need to run here?  And why not in the next case while waiting for ESP?
		cp.run();

		break;
	}

	//**TODO:  This case is broken!!!

	//  This needs some bounds checking on the array or you're asking for trouble.

	case CONNECT_WAIT:
	{
//		static boolean enteredState = false;
		static boolean started = false;
//		static unsigned long enteredTime = 0;

		static char waitBuf[15] = { 0 };
		static boolean waitRec = false;
		static uint8_t windx = 0;
		if (Serial.available()) {
			char c = Serial.read();
			if (c == '<') {
				waitRec = true;
				waitBuf[0] = 0;
				windx = 0;
			}
			if (waitRec) {
				waitBuf[windx] = c;
				waitBuf[++windx] = 0;
				if (c == '>') {
					if (started) {
						if (strcmp(waitBuf, COM_CONNECT_STRING)) {
							currentState = RUNNING;
						}
					}
					else {
						if (strcmp(waitBuf, COM_START_STRING)) {
							windx = 0;
							waitBuf[windx] = 0;
							started = true;   // next packet should be connection
							waitRec = false;  // wait for another SOP
						}
					}
				}
			}

		}

		break;
	}

	case RUNNING:
		monitorBattery();
		cp.run();

		mainControllerLoop();

		break;

	default:
		//  Freak out we shouldn't be here
		heartbeatInterval = 50;

		break;
	}
}



#define NUMBER_BATTERY_READINGS_TO_AVERAGE 30
void monitorBattery() {

	static uint16_t readings[NUMBER_BATTERY_READINGS_TO_AVERAGE] = { 0 };
	static uint8_t index = 0;
	static uint32_t total = 0;
	static uint16_t average = 0;

	//  Doesn't keep track of number of reads so it
	//  fills up the array as fast as it can and the
	//  first time it gets it full it slows down.
	static uint16_t readInterval = 10;
	static uint32_t pm = millis();
	uint32_t cm = millis();

	if (cm - pm >= readInterval) {
		pm = cm;

		total -= readings[index];
		readings[index] = analogRead(BATTERY_PIN);
		total += readings[index];
		++index;
		if(index == NUMBER_BATTERY_READINGS_TO_AVERAGE){
			// Slow down the read interval once the array fills up.
			readInterval = 1000;
		}
		index %= NUMBER_BATTERY_READINGS_TO_AVERAGE;

		average = total / NUMBER_BATTERY_READINGS_TO_AVERAGE;

//	float v = (r * 20.75) / 1024;

//		batteryVoltage = (average * 0.0202636719);
		batteryVoltage = (average * 0.020105) + 0.796904;  //Calibrated
		// 207.5 / 1024


		// TODO:  This should be stored somewhere and printed out in an orderly fashion.
		//  This should probably only happen once when the battery has changed and kept
		//  it's value for some time.
		Serial.print("<BAT,");
		Serial.print(average);
		Serial.print(",");
		Serial.print(batteryVoltage, 1);
		Serial.print(">");

	}
}


void heartBeat() {
	static boolean heartState = false;
	static uint8_t counter = 0;

	static unsigned long preMil = millis();
	unsigned long curMil = millis();

	if (curMil - preMil >= heartbeatInterval) {
		preMil = curMil;
		heartState = !heartState;
		digitalWrite(HEARTBEAT_PIN, heartState);
		counter++;
		// Send HB to controller every 5 seconds or so
		// It doesn't get scared until it loses it for at least 10
		if(counter == (6000 / heartbeatInterval)){
			Serial.print(HEARTBEAT_STRING);
			counter = 0;
		}

	}

}
