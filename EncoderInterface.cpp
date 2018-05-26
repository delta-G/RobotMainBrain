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




void EncoderInterface::tick(boolean aForward){

	uint32_t tickTime = micros();
	uint32_t delta = tickTime - lastTickMicros;
	lastTickMicros = tickTime;

	speed = 1000000ul / delta;   // ticks per 1,000,000 micros (1 second)

	if(!aForward){
		speed = 0 - speed;
		ticks--;
	}
	else {
		ticks++;
	}
}



int32_t EncoderInterface::getSpeed(){
	int32_t retval = 0;
	cli();
	retval = speed;
	sei();
	return retval;
}

int32_t EncoderInterface::getTicks(){
	int32_t retval = 0;
	cli();
	retval = ticks;
	sei();
	return retval;
}



