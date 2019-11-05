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

#ifndef BATTERY_H_
#define BATTERY_H_

#include "Arduino.h"
#include "Defines.h"

class Battery {

private:

	uint16_t readings[NUMBER_BATTERY_READINGS_TO_AVERAGE] = { 0 };
	uint8_t index;
	uint32_t total;
	uint16_t average;

	uint16_t readInterval;
	uint32_t previousReadMillis;

	uint8_t pin;

	float voltage;

public:

	Battery(uint8_t aPin, uint16_t aReadInterval);

	void initReadings();
	void monitor();

	float getVoltage();

};

#endif /* BATTERY_H_ */
