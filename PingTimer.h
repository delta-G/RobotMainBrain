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

#ifndef PINGTIMER_H_
#define PINGTIMER_H_

#include "Arduino.h"

#define PING_PIN_MASK (1 << 6)
#define PING_PIN_PORT PORTD
#define PING_PIN_DIRECTION DDRD

class PingTimer {
private:
	volatile int timerVal;
	volatile boolean overflowed;
	volatile boolean newData;

	void initTimer();

public:
	PingTimer();
	void begin();

	void sendPing();
	void echoHandler();
	void overflowHandler();

};



#endif /* PINGTIMER_H_ */
