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

#include "EncoderInterface.h"


void EncoderInterface::run() {

	// only run if we've got a good getCount pointer
	if (getCount) {

		uint32_t curTime = micros();
		int32_t curCount = getCount();

		//  so we get the same number on quick consecutive calls
		if (curTime - lastRunTime >= MINIMUM_RUN_TIME) {
			// speed in counts per microsecond
			float speed = (curCount - lastCount) / (curTime - lastRunTime);

			speeds[index] = speed;

			float sum;

			for (uint8_t i = 0; i < NUMBER_OF_SPEEDS; i++) {
				sum += speeds[i];
			}

			averageSpeed = sum / NUMBER_OF_SPEEDS;

			index++;
			lastCount = curCount;
			lastRunTime = curTime;
		}
	}
}


void EncoderInterface::clearSpeeds(){
	for(int i = 0; i < NUMBER_OF_SPEEDS; i++){
		speeds[i] = 0;
	}
}

float EncoderInterface::getAverageSpeed(){
	run();
	return averageSpeed;
}

float EncoderInterface::getSpeed(){
	run();
	// get the last speed in the circular buffer.
	return speeds[(index == 0)? NUMBER_OF_SPEEDS : index - 1];
}







