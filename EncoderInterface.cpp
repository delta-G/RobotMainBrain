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



//*****   ONLY CALL FROM ISR   NOT SAFE OTHERWISE
void EncoderInterface::tick(boolean aForward){

	uint32_t tickTime = micros();
	lastDeltaMicros = tickTime - lastTickMicros;
	lastTickMicros = tickTime;

	if(!aForward){
		ticks--;
	}
	else {
		ticks++;
	}
	forward = aForward;
}



int32_t EncoderInterface::getSpeed(){
	int32_t retval = 0;
	uint32_t ct = micros();
	cli();
	uint32_t lsDel = lastDeltaMicros;
	uint32_t lsTic = lastTickMicros;
	boolean forw = forward;
	sei();

	if(ct - lsTic >= lsDel){
		//  been too long, calculate based on time since last tick (slowing down)
		retval = (1000000ul / (ct - lsTic));  // gets 0 after 1 second
	}
	// last tick was very recent, speed numbers are good.
	// calculate based on the delta of last two ticks
	else {
		retval = 1000000ul / lsDel;
	}

	if (!forw){
		retval = 0 - retval;
	}

	return retval;
}


int32_t EncoderInterface::getTicks(){
	int32_t retval = 0;
	cli();
	retval = ticks;
	sei();
	return retval;
}



