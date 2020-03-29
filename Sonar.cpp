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

void Sonar::setSweepCallback(void (*aCallback)()){
	sweepCallback = aCallback;
}

void Sonar::setPingCallback(void (*aCallback)()){
	sweepCallback = aCallback;
}

void Sonar::begin(){
	ping.begin();
	gimbal.init();
	state = NOT_RUNNING;
	gimbal.setPanSpeed(1750);
	gimbal.setTiltSpeed(1000);
}

void Sonar::startPing() {
	ping.sendPing();
	if (state == NOT_RUNNING) {
		state = HOLDING;
	}
}

void Sonar::stopPing(){
	state = NOT_RUNNING;
}

void Sonar::startSweep() {
	if ((state != SWEEP_FORW)&&(state != SWEEP_BACK)) {
		sweepState = STARTING;
		state = SWEEP_FORW;
		sweep();
	}
}

void Sonar::sweep() {

	static uint32_t delayStart = 0;

	switch (sweepState) {
	case STARTING:
		if(state == SWEEP_FORW){
			gimbal.setPanAngle(minSweepAngle);
			sweepIndex = 0;

		} else if(state == SWEEP_BACK){
			gimbal.setPanAngle(maxSweepAngle);
			sweepIndex = 12;
		}
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

			if(state == SWEEP_FORW){
				if (sweepIndex < 12) {
					sweepIndex++;
					gimbal.setPanAngle(((float) sweepIndex / 12.0) * 3.141592);
					sweepState = MOVING;
				} else {
					if (continuousSweep) {
						sweepState = SWEEP_DELAY;
						delayStart = millis();
					} else {
						state = NOT_RUNNING;
						gimbal.setPanAngle(1.570796);
						sweepState = STARTING;
					}
					dumpSweep = true;
					scanDone = true;
					newDump = true;
					if(sweepCallback != NULL){
						sweepCallback();
					}
				}
			} else if (state == SWEEP_BACK) {
				if (sweepIndex > 0) {
					sweepIndex--;
					gimbal.setPanAngle(((float) sweepIndex / 12.0) * 3.141592);
					sweepState = MOVING;
				} else {
					if (continuousSweep) {
						sweepState = SWEEP_DELAY;
						delayStart = millis();
					} else {
						state = NOT_RUNNING;
						gimbal.setPanAngle(1.570796);
						sweepState = STARTING;
					}
					dumpSweep = true;
					scanDone = true;
					newDump = true;
					if(sweepCallback != NULL){
						sweepCallback();
					}
				}
			}
		}
		break;
	case SWEEP_DELAY:
		if(millis() - delayStart >= sweepDelay){
			if(state == SWEEP_FORW){
				state = SWEEP_BACK;
			}
			else if(state == SWEEP_BACK){
				state = SWEEP_FORW;
			}
			sweepState = PINGING;
			startPing();
		}
		break;
	}
}

void Sonar::setContinuous(bool aBool){
	continuousSweep = aBool;
}

void Sonar::setSweepDelay(uint16_t aDelay){
	sweepDelay = aDelay;
}

void Sonar::setHoldDelay(uint16_t aDelay){
	holdDelay = aDelay;
}

void Sonar::loop() {
	static uint32_t delayStart = millis();
	gimbal.run();
	if (state == SWEEP_FORW || state == SWEEP_BACK) {
		sweep();
	}
	else if(state == HOLD_DELAY){
		if(millis() - delayStart >= holdDelay){
			state = HOLDING;
			startPing();
		}
	}
	else {
		if (ping.hasNewData()) {
			distance = ping.getDistanceMM();
			curpan = gimbal.getPan();
			curtilt = gimbal.getTilt();
			if (state == HOLDING) {
				state = HOLD_DELAY;
				delayStart = millis();
			}
			else {
				state = NOT_RUNNING;
			}
			if(pingCallback != NULL){
				pingCallback();
			}
			newDump = true;
		}
	}
}

void Sonar::parkSensor(){
	gimbal.setPan(1450);
	gimbal.setTilt(550);
}

int16_t Sonar::getDistance(){
	return distance;
}

int16_t Sonar::getDistance(uint8_t aIndex){
	return distances[aIndex];
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
	newDump = false;
	return data;
}


boolean Sonar::scanFinished(){
	boolean retval = scanDone;
	scanDone = false;
	return retval;
}

boolean Sonar::hasNewDump(){
	return newDump;
}



void Sonar::setMinAngle(float aStart){minSweepAngle = aStart;}
void Sonar::setMaxAngle(float aEnd){maxSweepAngle = aEnd;}
float Sonar::getMinAngle(){return minSweepAngle;}
float Sonar::getMaxAngle(){return maxSweepAngle;}


bool Sonar::isHolding(){
	return (state == HOLDING || state == HOLD_DELAY);
}


