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

extern CommandParser cp;

unsigned int heartbeatInterval = 500;

XboxHandler xbox;

Motor leftMotor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_ENABLE_PIN, true );
Motor rightMotor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_ENABLE_PIN, false );

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

	// give a second for power to stabilize

	heartbeatInterval = 200;
	unsigned long delayStart = millis();
	while (millis() - delayStart <= 5000) {
		heartBeat();
	}

	// Turn on the Arm and give it time to do it's thing

	digitalWrite(ARM_ENABLE, HIGH);
	heartbeatInterval = 500;
	delayStart = millis();
	while (millis() - delayStart <= 10000) {
		heartBeat();
	}

	// Turn on the COM board

	digitalWrite(COM_POWER_ENABLE, HIGH);

	delay(500);

	initializeControllerFunctions(&leftMotor, &rightMotor, &Serial, &Serial1,
			&xbox);





	analogReference(INTERNAL1V1);



	Serial.begin(115200);

	heartbeatInterval = 1000;

	delay(1000);

	Serial.print("<RobotMainBrain Active>");

	// Connect to Arm Controller

	Serial1.begin(115200);
	delay(250);

	heartbeatInterval = 2000;

}

void loop() {
	heartBeat();
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
		if(counter == (6000 / heartbeatInterval)){
			Serial.print("<RMB HB>");
			counter = 0;
		}

	}

}
