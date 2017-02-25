/*
 * PingTimer.h
 *
 *  Created on: Feb 24, 2017
 *      Author: david
 */

#ifndef PINGTIMER_H_
#define PINGTIMER_H_

#include "Arduino.h"

#define PING_PIN_MASK (1 << 6)
#define PING_PIN_PORT PORTD
#define PING_PIN_DIRECTION DDRD

class PingTimer {

	volatile int timerVal;
	volatile boolean overflowed;
	volatile boolean newData;

	PingTimer();
	void initTimer();

	void sendPing();
	void echoHandler();
	void overflowHandler();

};



#endif /* PINGTIMER_H_ */
