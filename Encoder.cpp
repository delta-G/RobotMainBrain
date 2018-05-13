/*

Encoder.cpp  --  runs on 1284P and handles motor encoders of my robot
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

#include "Encoder.h"


#define LEFT_INT_MASK 0x80
#define LEFT_B_MASK 0x40
#define RIGHT_INT_MASK 0x20
#define RIGHT_B_MASK 0x10

volatile uint8_t lastPortRead = 0;

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;




ISR(PCINT2_vect){

	uint8_t portRead = PINC;

	uint8_t whoFired = portRead ^ lastPortRead;
	uint8_t pinsDiff = portRead ^ (portRead << 1); // Will have bits 5 or 7 set if pins on right motor or left motor are diff.  Cleared if same.

	if (whoFired & LEFT_INT_MASK) {
		if (pinsDiff & LEFT_INT_MASK) {
			leftCounter++;
		} else {
			leftCounter--;
		}
	}
	if (whoFired & RIGHT_INT_MASK) {
		if (pinsDiff & RIGHT_INT_MASK) {
			rightCounter++;
		} else {
			rightCounter--;
		}
	}

	lastPortRead = portRead;

}


void setupPCint(){

	PCICR = PCICR | (1 << PCIE2);   //  Turn on pin change interrupt 2
	PCIFR = (1 << PCIF2);   // Clear any pending interrupt flag for pin change interrupt 2
	PCMSK2 = (1 << PCINT23) | (1 << PCINT21);  // Set interrupts for pins 21 and 23 at bits 5 and 7.  Bits 4 and 6 will be the other pins.

}






