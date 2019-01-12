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

enum BootStates {
	BOOTING,
	BOOT_ARM,
	BOOTING_ARM,
	STARTING_ARM_COM,
	CONNECT_COM,
	CONNECTING_COM,
	CONNECT_WAIT,
	RUNNING,
	NUM_RMB_STATES
} bootState;

extern CommandParser cp;

unsigned int heartbeatInterval = 200;

XboxHandler xbox;

Robot robot;

void setup() {

	// Start with everything off

	pinMode(COM_POWER_ENABLE, OUTPUT);
	digitalWrite(COM_POWER_ENABLE, HIGH);  //except this, this is on

	pinMode(ARM_ENABLE, OUTPUT);
	digitalWrite(ARM_ENABLE, LOW);

	pinMode(CAM_ENABLE, OUTPUT);
	digitalWrite(CAM_ENABLE, LOW);

	pinMode(HEADLIGHT_PIN, OUTPUT);
	digitalWrite(HEADLIGHT_PIN, LOW);

	robot.leftMotor.init();
	robot.rightMotor.init();

	pinMode(HEARTBEAT_PIN, OUTPUT);
	for (int i = 0; i < 3; i++) {
		digitalWrite(HEARTBEAT_PIN, HIGH);
		delay(100);
		digitalWrite(HEARTBEAT_PIN, LOW);
		delay(100);
	}

	analogReference(INTERNAL1V1);

	initializeControllerFunctions(&robot, &Serial, &Serial1, &xbox);

	setupPCint();  // for the encoders

	//  loop here while everything boots up.
	while (bootState < RUNNING) {
		heartBeat();
		bootup();
	}

}

void bootup() {

	static unsigned long armStartTime = 0;
	static unsigned long comStartedTime = 0;

	switch (bootState) {

	//   3 seconds at bootup for power to stabilize etc.
	case BOOTING:

		if (millis() > 3000) {
			bootState = BOOT_ARM;
		}
		break;

	case BOOT_ARM: {
//		digitalWrite(ARM_ENABLE, HIGH);
		robot.arm.enable();
		heartbeatInterval = 500;
		armStartTime = millis();
		bootState = STARTING_ARM_COM;
		break;
	}

	case STARTING_ARM_COM: {
		if (millis() - armStartTime >= 1000) {
			Serial1.begin(ARM_BOARD_BAUD);
			bootState = BOOTING_ARM;
		}

		break;
	}

	case BOOTING_ARM: {

		if (millis() - armStartTime >= 7000) {
			bootState = CONNECT_COM;
		}

		break;
	}

	case CONNECT_COM: {
		Serial.begin(ROBOT_COM_BAUD);
		comStartedTime = millis();
		bootState = CONNECTING_COM;
		break;
	}

	case CONNECTING_COM: {
		if ((millis() - comStartedTime > 250)) {
			Serial.print(RMB_STARTUP_STRING);
			char gitbuf[9];
			strncpy(gitbuf, GIT_HASH, 8);
			gitbuf[8] = 0;
			Serial.print("<RMBGIT-");
			Serial.print(gitbuf);
			Serial.print(">");
			bootState = CONNECT_WAIT;
			heartbeatInterval = 2000;
		}

		break;
	}

	case CONNECT_WAIT: {
		static boolean started = false;

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
				if (windx < 14) {
					waitBuf[windx] = c;
					waitBuf[++windx] = 0;
				}
				if (c == '>') {
					if (started) {
						if (strcmp(waitBuf, COM_CONNECT_STRING) == 0) {
							bootState = RUNNING;
						}
					} else {
						if (strcmp(waitBuf, COM_START_STRING) == 0) {
							windx = 0;
							waitBuf[windx] = 0;
							started = true;  // next packet should be connection
							waitRec = false;  // wait for another SOP
						}
						//  BackDoor for testing
						else if (strcmp(waitBuf, "<GO>") == 0) {
							bootState = RUNNING;  // this will break the while loop calling us
						}
					}
				}
			}

		}

		break;
	}

	default:
		//  Freak out we shouldn't be here
		heartbeatInterval = 50;
		break;
	}

}

void loop() {

	heartBeat();


	robot.mainLoop();
	cp.run();
	mainControllerLoop();

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
		if (counter == (6000 / heartbeatInterval)) {
			Serial.print(HEARTBEAT_STRING);
			counter = 0;
		}

	}

}
