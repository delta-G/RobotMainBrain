/*
 * XboxHandler.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: david
 */

#include "XboxHandler.h"



void XboxHandler::handleIncoming(char* aPacket){

	if(aPacket[0] == '<' && aPacket[1] == 'X' && aPacket[17] == '>'){
		memcpy(readUnion.rawBuffer, aPacket + 3, 14);
		newData = true;

		buttonClickState = readUnion.values.buttonState & ~oldButtonState;
		oldButtonState = readUnion.values.buttonState;

		if(((uint8_t)oldTriggerState = 0) && readUnion.values.rightTrigger != 0){
			R2Clicked = true;
		}
		if ((oldTriggerState >> 8 = 0) && readUnion.values.leftTrigger != 0) {
			L2Clicked = true;
		}

		oldTriggerState = (((uint16_t) readUnion.values.leftTrigger) << 8) | readUnion.values.rightTrigger;

	}
}


boolean XboxHandler::isClicked(ButtonMaskEnum aButton) {

	uint16_t retval = (buttonClickState & aButton); // aButton is 0 for L2 and R2 since they're analog

	// aButton is 0 for L2 and R2 since they're analog

	if (aButton == L2) {
		retval = L2Clicked;
		L2Clicked = false;
	} else if (aButton == R2) {
		retval = R2Clicked;
		R2Clicked = false;
	} else {
//		retval = (buttonClickState & aButton);

		buttonClickState &= ~retval;  // clear that click

	}

	return (boolean) retval;

}


boolean XboxHandler::isPressed(ButtonMaskEnum aButton) {

	if (!(aButton == L2 || aButton == R2)) {
		return (boolean) (readUnion.values.buttonState & aButton);
	}

	if(aButton == L2){
		return (boolean)readUnion.values.leftTrigger;
	}
	return (boolean)readUnion.values.rightTrigger;

}

