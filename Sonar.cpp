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
}

void Sonar::startPing(){
//	Serial.print("<SONAR_PING>");
	ping.sendPing();
//	digitalWrite(13, !digitalRead(13));
}


void Sonar::loop(){
	gimbal.run();
	if(ping.hasNewData()){
//		Serial.print("<NEW_SONAR>");
		distance = ping.getDistanceMM();
		ping.sendPing();
	}
}

int16_t Sonar::getDistance(){
	return distance;
}

uint8_t* Sonar::dataDump(){

	uint8_t dataSize = 10;
	static uint8_t data[10];

	uint16_t curpan = getPan();
	uint16_t curtilt = getTilt();

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

	for(int i=0; i<dataSize; i++){
		Serial.write(data[i]);
	}

	return data;
}


void Sonar::setPan(uint16_t aTargetMicros) {
	gimbal.getPanJoint()->setTarget(aTargetMicros);
}

void Sonar::setTilt(uint16_t aTargetMicros) {
	gimbal.getTiltJoint()->setTarget(aTargetMicros);
}

uint16_t Sonar::getPan(){
	return gimbal.getPanJoint()->getPosition();
}

uint16_t Sonar::getTilt(){
	return gimbal.getTiltJoint()->getPosition();
}

void Sonar::setPanSpeed(uint16_t aPanSpeed){
	gimbal.getPanJoint()->setSpeed(aPanSpeed);
}

void Sonar::setTiltSpeed(uint16_t aTiltSpeed){
	gimbal.getTiltJoint()->setSpeed(aTiltSpeed);
}
