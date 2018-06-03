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


class EncoderInterface {

private:

	volatile uint32_t lastTickMicros;
	volatile uint32_t lastDeltaMicros;
	volatile int32_t ticks;
	volatile boolean forward;



public:

	EncoderInterface():lastTickMicros(0), lastDeltaMicros(0), ticks(0), forward(false){}

	void tick(boolean);
	int32_t getSpeed();
	int32_t getTicks();


};




#endif /* ENCODERINTERFACE_H_ */
