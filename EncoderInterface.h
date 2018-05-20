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

#ifndef ENCODERINTERFACE_H_
#define ENCODERINTERFACE_H_

#include "Arduino.h"


//////*  TODO  **

/*
 *
 * Use scope to figure this timing out.  It shouldn't be arbitrary
 *
 *
 */



#define NUMBER_OF_SPEEDS 16
#define MINIMUM_RUN_TIME 20000  // microseconds


class EncoderInterface {


	int32_t (*getCount)();
	float speeds[NUMBER_OF_SPEEDS];
	uint8_t index;
	int32_t lastCount;

	uint32_t lastRunTime = 0;



	float averageSpeed;




public:

	EncoderInterface(int32_t(*aGet)()) : getCount(aGet), lastCount(0), index(0), averageSpeed(0){clearSpeeds();}

	void run();
	void clearSpeeds();

	float getAverageSpeed();
	float getSpeed();



};




#endif /* ENCODERINTERFACE_H_ */
