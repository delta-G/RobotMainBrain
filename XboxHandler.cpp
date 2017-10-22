/*
 * XboxHandler.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: david
 */

#include "XboxHandler.h"

XboxHandler::XboxHandler(){
	memset(readUnion.rawBuffer, 0, 14);
	oldButtonState = 0;
	oldTriggerState=0;
	buttonClickState=0;
	memset((uint8_t*)oldHatState, 0, 8);
	L2Clicked= false;
	R2Clicked= false;
	newData = false;
}

boolean XboxHandler::newDataAvailable(){
	boolean retval = newData;
	newData = false;
	return retval;
}


void XboxHandler::handleIncoming(char* aPacket){

	Serial.print("<PACKET>");

	//save old hat state
	memcpy (oldHatState, readUnion.values.hatValues, 8);

	if(aPacket[0] == '<' && aPacket[1] == 'C' && aPacket[17] == '>'){
		Serial.print("<C");
		for(int i = 3; i < 17; i++){
			Serial.print(',');
			Serial.print(aPacket[i], HEX);
		}
		Serial.print('>');
		memcpy(readUnion.rawBuffer, aPacket + 3, 14);
		newData = true;

		//  Use OR Equal to preserve clicks that haven been read yet
		buttonClickState |= readUnion.values.buttonState & ~oldButtonState;
		oldButtonState = readUnion.values.buttonState;

		if(((uint8_t)oldTriggerState == 0) && readUnion.values.rightTrigger != 0){
			R2Clicked = true;
		}
		if ((oldTriggerState >> 8 == 0) && readUnion.values.leftTrigger != 0) {
			L2Clicked = true;
		}

		oldTriggerState = (((uint16_t) readUnion.values.leftTrigger) << 8) | readUnion.values.rightTrigger;

		newData = true;

	}
}

void XboxHandler::handleIncomingASCII(char* aPacket){

	Serial.print("<ASCIIPACK>");

	//save old hat state
	memcpy (oldHatState, readUnion.values.hatValues, 8);

	if(aPacket[0] == '<' && aPacket[1] == 'X' && aPacket[17] == '>'){

		uint8_t rawBuf[14];

		for ( uint8_t i = 0; i < 14; i++){
			char temp[2] = {aPacket[3+(2*i)], aPacket[4+(2*i)]};
			rawBuf[i] = strtoul(temp, NULL, 16);
		}

		char temp[2][25];
		sprintf(temp[0], "<X,%04X%08lX>", *((uint16_t*)rawBuf), *((uint32_t*)(rawBuf+2)));
		sprintf(temp[1], "<x,%08lX%08lX>", *((uint32_t*)(rawBuf+6)), *((uint32_t*)(rawBuf+10)));
		Serial.print(temp[0]);
		Serial.print(temp[1]);


		memcpy(readUnion.rawBuffer, rawBuf, 14);

		//  Use OR Equal to preserve clicks that haven been read yet
		buttonClickState |= readUnion.values.buttonState & ~oldButtonState;
		oldButtonState = readUnion.values.buttonState;

		if(((uint8_t)oldTriggerState == 0) && readUnion.values.rightTrigger != 0){
			R2Clicked = true;
		}
		if ((oldTriggerState >> 8 == 0) && readUnion.values.leftTrigger != 0) {
			L2Clicked = true;
		}

		oldTriggerState = (((uint16_t) readUnion.values.leftTrigger) << 8) | readUnion.values.rightTrigger;

		newData = true;

	}
}


boolean XboxHandler::isClicked(ButtonMaskEnum aButton) {

	uint16_t retval = (buttonClickState & aButton);

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


int16_t XboxHandler::getHatValue(HatEnum aHat) {
	int16_t retval = readUnion.values.hatValues[aHat];
	if (abs(retval) < DEFAULT_DEADBAND) {
		retval = 0;
	}
	return retval;

}
