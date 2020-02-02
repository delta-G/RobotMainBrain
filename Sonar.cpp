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

#include "Sonar.h"


void Sonar::begin(){
	ping.begin();
//	ping.sendPing();
	gimbal.init();
	state = NOT_RUNNING;
}

void Sonar::startPing(){
	ping.sendPing();
	if(state == NOT_RUNNING){
		state = HOLDING;
	}
}

void Sonar::stopPing(){
	state = NOT_RUNNING;
}

void Sonar::startSweep() {
	if (state != SWEEPING) {
		sweepState = STARTING;
		state = SWEEPING;
		sweep();
	}
}

void Sonar::sweep() {

	static uint32_t delayStart = 0;

	switch (sweepState) {
	case STARTING:
		gimbal.setPanAngle(0);
		sweepIndex = 0;
		sweepState = MOVING;
		break;
	case MOVING:
		if (!gimbal.isMoving()) {
			sweepState = DELAYING;
			delayStart = millis();
		}
		break;
	case DELAYING:
		if (millis() - delayStart >= 20) {
			sweepState = PINGING;
			startPing();
		}
		break;
	case PINGING:
		if (ping.hasNewData()) {
			distances[sweepIndex] = ping.getDistanceMM();
			sweepIndex++;
			if (sweepIndex < 13) {
				gimbal.setPanAngle(((float) sweepIndex / 12.0) * 3.141592);
				sweepState = MOVING;
			} else {
				gimbal.setPanAngle(1.570796);
				sweepState = STARTING;
				state = NOT_RUNNING;
				dumpSweep = true;
			}
		}
		break;
	}

}

void Sonar::loop() {
	gimbal.run();
	if (state == SWEEPING) {
		sweep();
	} else {
		if (ping.hasNewData()) {
			distance = ping.getDistanceMM();
			curpan = gimbal.getPan();
			curtilt = gimbal.getTilt();
			if (state == HOLDING) {
				ping.sendPing();
			}
		}
	}
}

int16_t Sonar::getDistance(){
	return distance;
}

uint8_t* Sonar::dataDump() {
	uint8_t dataSize = 10;
	static uint8_t data[30];

	if (dumpSweep) {
		dumpSweep = false;
		dataSize = 30;
		data[0] = '<';
		data[1] = 0x13;
		data[2] = dataSize;
		for(int i=0; i<13; i++){
			data[(2*i)+3]=((distances[i] >> 8) & 0xFF);
			data[(2*i)+4]=(distances[i] & 0xFF);
		}
		data[29] = '>';

	} else {
		dataSize = 10;
		data[0] = '<';
		data[1] = 0x13;
		data[2] = dataSize;
		data[3] = ((distance >> 8) & 0xFF);
		data[4] = (distance & 0xFF);
		data[5] = ((curpan >> 8) & 0xFF);
		data[6] = (curpan & 0xFF);
		data[7] = ((curtilt >> 8) & 0xFF);
		data[8] = (curtilt & 0xFF);
		data[9] = '>';
	}
	for (int i = 0; i < dataSize; i++) {
		Serial.write(data[i]);
	}

	return data;
}



