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

void Switchable::enable(boolean aState){
	digitalWrite(pin, inverted? !aState : aState);
}



void Robot::monitorBattery() {

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
		if (index == NUMBER_BATTERY_READINGS_TO_AVERAGE) {
			// Slow down the read interval once the array fills up.
			readInterval = 1000;
		}
		index %= NUMBER_BATTERY_READINGS_TO_AVERAGE;

		average = total / NUMBER_BATTERY_READINGS_TO_AVERAGE;

		//	float v = (r * 20.75) / 1024;

		//		batteryVoltage = (average * 0.0202636719);   //  Theoretical
		batteryVoltage = (average * 0.020105) + 0.796904;  //Calibrated
		// 207.5 / 1024

	}

}


float Robot::getBatteryVoltage(){
	return batteryVoltage;
}
