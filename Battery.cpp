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

#include "Battery.h"

Battery::Battery(uint8_t aPin, uint16_t aReadInterval) {
	index = 0;
	total = 0;
	average = 0;
	readInterval = 0;

	readInterval = aReadInterval;
	previousReadMillis = millis();

	pin = aPin;

	voltage = 0.0;
}

void Battery::initReadings() {
	// Fill reading array with an initial value.
	int val = analogRead(pin);
	val = analogRead(pin);  // read twice since we're probably at startup
	for (int i = 0; i < NUMBER_BATTERY_READINGS_TO_AVERAGE; i++) {
		readings[i] = val;
		total += val;
	}
}

void Battery::monitor() {

	uint32_t cm = millis();

	if (cm - previousReadMillis >= readInterval) {
		previousReadMillis = cm;

		total -= readings[index];
		readings[index] = analogRead(pin);
		total += readings[index];
		++index;
		index %= NUMBER_BATTERY_READINGS_TO_AVERAGE;
		average = total / NUMBER_BATTERY_READINGS_TO_AVERAGE;

		//	float v = (r * 20.75) / 1024;
		//		batteryVoltage = (average * 0.0202636719);   //  Theoretical
		voltage = (average * 0.020105) + 0.796904;  //Calibrated
	}

}

float Battery::getVoltage() {
	return voltage;
}
