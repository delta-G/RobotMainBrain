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
	POWERUP,
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



StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, parseCommand);
StreamParser armParser(&Serial1, START_OF_PACKET, END_OF_PACKET, armParserCallback);

unsigned long lastCommandTime;

unsigned long commandTimeout = 3000;
boolean commandTimeoutOverride = false;
boolean useSpeedBasedAlgs = false;

boolean runSpeedReport = false;

void debugBlink(uint8_t numTimes) {
	for (uint8_t i = 0; i < numTimes; i++) {
		digitalWrite(HEARTBEAT_PIN, HIGH);
		delay(100);
		digitalWrite(HEARTBEAT_PIN, LOW);
		delay(100);
	}
}

void parseCommand(char *aCommand) {
	if (strcmp(aCommand, "<LOST_COM>") == 0) {
		robot.allStop();
	} else {
		cp.parseCommandString(aCommand);
		lastCommandTime = millis();
	}
}


void setup() {
	pinMode(4, OUTPUT); // Stop SPI from going slave while we set up


	pinMode(SCK, OUTPUT); // @suppress("Invalid arguments")
	pinMode(MISO, INPUT); // @suppress("Invalid arguments")
	pinMode(MOSI, OUTPUT); // @suppress("Invalid arguments")

	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	SPSR = 0;
	SPSR = (1 << SPI2X);

	robot.init();

	pinMode(HEARTBEAT_PIN, OUTPUT);
	for (int i = 0; i < 3; i++) {
		digitalWrite(HEARTBEAT_PIN, HIGH);
		delay(100);
		digitalWrite(HEARTBEAT_PIN, LOW);
		delay(100);
	}

	analogReference(INTERNAL1V1);

	initializeControllerFunctions(&Serial, &Serial1, &xbox);

	setupPCint();  // for the encoders

	//  loop here while everything boots up.
	while (bootState < RUNNING) {
//		heartBeat();
		bootup();
	}

	armParser.setRawCallback(armParserRawCallback);
	parser.setRawCallback(rawDataCallback);

}

void bootup() {

	static unsigned long armStartTime = 0;
	static unsigned long comStartedTime = 0;

	switch (bootState) {

	case POWERUP:
//		robot.motorPower.enable();
		robot.comPower.enable();
//		robot.motorController.enable();
		bootState = BOOTING;
		debugBlink(1);
		break;

		//   3 seconds at bootup for power to stabilize etc.
	case BOOTING:

		if (millis() > RMB_BOOT_INIT_WAIT) {
			bootState = BOOT_ARM;
			// For some reason if the heartbeat light is on when the arm boots then RMB will lock up.  Must be a power thing somewere.
			robot.silenceHeartbeat();
			debugBlink(1);
		}
		break;

	case BOOT_ARM: {
		if (armStartTime == 0) {
			robot.arm.enable();
			heartbeatInterval = 500;
			armStartTime = millis();
		} else {
			if (millis() - armStartTime >= ARM_BOOT_INIT_WAIT) {
				Serial1.begin(ARM_BOARD_BAUD);
				bootState = BOOTING_ARM;
				robot.restartHeartbeat();
				debugBlink(2);
			}
		}
		break;
	}

	case BOOTING_ARM: {

		armParser.run();

		if(robot.armPresent && robot.armResponding){
//			debugBlink(3);
			bootState = CONNECT_COM;
		}
		// Timeout in case arm is not there
		if (millis() - armStartTime >= ARM_BOOT_TIMEOUT) {
			Serial1.end();
			robot.armPresent = false;
			robot.armResponding = false;
			bootState = CONNECT_COM;
		}

		break;
	}

	case CONNECT_COM: {
		debugBlink(3);
		Serial.begin(ROBOT_COM_BAUD);
		comStartedTime = millis();
		bootState = CONNECTING_COM;
		debugBlink(1);
		break;
	}

	case CONNECTING_COM: {
		if ((millis() - comStartedTime > 250)) {
			Serial.print(F(RMB_STARTUP_STRING));
			char gitbuf[9];
			strncpy(gitbuf, GIT_HASH, 8);
			gitbuf[8] = 0;
			Serial.print("<RMBGIT-");
			Serial.print(gitbuf);
			Serial.print(">");
			bootState = CONNECT_WAIT;
		}

		break;
	}

	case CONNECT_WAIT: {
		/*I suppose I could hijack cp for this if I ever need to save a few bytes*/
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
							heartbeatInterval = 2000;
						}
					} else {
						if (strcmp(waitBuf, COM_START_STRING) == 0) {
							windx = 0;
							waitBuf[windx] = 0;
							started = true;  // next packet should be connection
							waitRec = false;  // wait for another SOP
							heartbeatInterval = 1000;
						}

						//  BackDoor for testing
						else if (strcmp(waitBuf, "<GO>") == 0) {
							bootState = RUNNING; // this will break the while loop calling us
							heartbeatInterval = 2000;
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

	robot.mainLoop();  //Things the robot does on his own...
	armParser.run();       // Read serial coming from Arm
	parser.run();          // Reads in commands from Serial
	mainControllerLoop();  // Interacts with Xbox controller.

	if((millis() - lastCommandTime >= commandTimeout) && (!commandTimeoutOverride)){
		robot.allStop();
	}

	if(runSpeedReport){
		reportSpeeds();
	}

}


void heartBeat() {
	static boolean heartState = false;

	static unsigned long preMil = millis();
	unsigned long curMil = millis();

	if (curMil - preMil >= heartbeatInterval) {
		preMil = curMil;
		heartState = !heartState;
		if (!robot.heartSilenced) {
			digitalWrite(HEARTBEAT_PIN, heartState);
		}
	}
}


void rawDataCallback(char* p){
	if(p[1] == 0x14 && p[2] == 16){
		xboxCommandRaw(p);
		lastCommandTime = millis();
	}
	else {
		sendError(ECODE_BAD_RAW_INPUT);
	}
}


void armParserCallback(char* aCommand){
	if(strcmp(aCommand, ARM_INIT_COMPLETE) == 0){
		robot.armPresent = true;
		Serial1.print(F(RMB_ARM_TEST_STRING));
	}
	else if(strcmp(aCommand, ARM_CONNECT_RESPONSE) == 0){
		robot.armResponding = true;
	}
	else if(strcmp(aCommand, ARM_NO_NEW_DATA) == 0){
		// if no new arm data, call regular response again and get a dump or voltages
//		robot.regularResponse();
		robot.redundantArmReport();
	}
	else if(strcmp(aCommand, ARM_MOVEMENT_DONE) == 0){
		if(robot.movementDoneWait){
			robot.movementDoneWait = false;
		}
	}
	else {
		if(bootState == RUNNING){
			Serial.print(aCommand);
		}
	}
}


void armParserRawCallback(char* p) {
	// If it is a good response
	if(p[0]=='<' && p[2]==ARM_DUMP_SIZE && p[ARM_DUMP_SIZE-1]=='>'){
		//  This will get sent out with next regular response.
		robot.saveArmReport((uint8_t*)p);
	}
}


