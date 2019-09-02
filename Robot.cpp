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

#include "Robot.h"



boolean Switchable::isEnabled(){
	return enabled;
}

void Switchable::enable() {
	digitalWrite(pin, inverted ? LOW : HIGH);
	enabled = true;
}

void Switchable::disable() {
	digitalWrite(pin, inverted ? HIGH : LOW);
	enabled = false;
}


void Robot::mainLoop(){
	battery.monitor();
	leftMotor.loop();
	rightMotor.loop();
}

uint8_t* Robot::dataDump() {

	Serial.print(HBOR_STRING);

	static uint8_t data[15];

	data[0] = '<';
	data[1] = 0x13;
	data[2] =  15;
	data[3] = (byte) (battery.getVoltage() * 10);
	data[4] = (byte) ((leftMotor.encoder.getTicks() >> 8) & 0xFF);
	data[5] = (byte) (leftMotor.encoder.getTicks() & 0xFF);
	data[6] = (byte) ((leftMotor.getSpeed() >> 8) & 0xFF);
	data[7] = (byte) (leftMotor.getSpeed() & 0xFF);
	data[8] = (byte) (leftMotor.getPwmSpeed() & 0xFF);
	data[9] = (byte) ((rightMotor.encoder.getTicks() >> 8) & 0xFF);
	data[10] = (byte) (rightMotor.encoder.getTicks() & 0xFF);
	data[11] = (byte) ((rightMotor.getSpeed() >> 8) & 0xFF);
	data[12] = (byte) (rightMotor.getSpeed() & 0xFF);
	data[13] = (byte) (rightMotor.getPwmSpeed() & 0xFF);
	data[14] = '>';

	for(int i=0; i<15; i++){
		Serial.write(data[i]);
	}

	return data;

}



void Robot::setDriveMode(DriveModeEnum aDriveMode){
	driveMode = aDriveMode;
	if(driveMode >= NUMBER_OF_MODES){
		driveMode = (DriveModeEnum)((int)driveMode % NUMBER_OF_MODES);
	}
	switch (driveMode){
	case DRIVE:
		Serial1.print("<A,CMD>");
		break;
	case ARM:
		Serial1.print("<A,CMA>");
		break;
	case MINE:
		Serial1.print("<A,CMM>");
		break;
	}
}

DriveModeEnum Robot::getDriveMode(){
	return driveMode;
}

DriveModeEnum Robot::advanceDriveMode(){
	DriveModeEnum dm = driveMode;
	dm = (DriveModeEnum)((int)dm + 1);
	setDriveMode(dm);
	return driveMode;
}

