/*
 * PingTimer.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: david
 */

#include "PingTimer.h"



void PingTimer::initTimer(){

	TCCR1A = 0;  // no PWM
	// Setup Input Capture
	// Noise Cancel On, Falling Edge , No prescaler, 4ms range.
	TCCR1B = (1 << ICNC1) | (0 << ICES1) | (1 << CS10);

	// make sure analog comparator isn't selected as capture input
	ACSR &= ~(1 << ACIC);

	//  Turn off interrupts Input Capture and Timer Overflow
	TIMSK1 = 0;

}


PingTimer::PingTimer(){
	initTimer();
	newData = false;
	overflowed = false;
	timerVal = 0;
}



void PingTimer::sendPing(){

	// make pin output and low
	DDRD |= PING_PIN_MASK;
	PING_PIN_PORT &= ~PING_PIN_MASK;
	delayMicroseconds(4);
	// send high pulse
	PING_PIN_PORT |= PING_PIN_MASK;
	delayMicroseconds(10);
	//end high pulse
	PING_PIN_PORT &= ~PING_PIN_MASK;
	//make pin input and pull-up on
	DDRD &= ~PING_PIN_MASK;
	PING_PIN_PORT |= PING_PIN_MASK;

	//  Turn on interrupts Input Capture and Timer Overflow
	TIMSK1 = (1 << ICIE1) | (1 << TOIE1);

	//  Reset Input Capture and Timer Overflow Interrupts
	TIFR1 = (1 << ICF1) | (1 << TOV1);

	while(!(PING_PIN_PORT & PING_PIN_MASK));  // wait to start timer
	sei();
	TCNT1H = 0;
	TCNT1L = 0;
	cli();

	overflowed = false;
	newData = false;

}


void PingTimer::echoHandler(){
	// Read the ICR1 Register
	uint8_t low = ICR1L;
	uint8_t high = ICR1H;
	timerVal = ((high << 8) | low) - 4;  // Noise canceler adds 4 clock cycles

	//  turn off the interrupts
	TIMSK1 = 0;
	newData = true;
}

void PingTimer::overflowHandler(){
	//turn off ICR and TOV interrupts
	TIMSK1 = 0;
	newData = true;
	overflowed = true;
}












