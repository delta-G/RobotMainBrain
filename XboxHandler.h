/*
 * XboxHandler.h
 *
 *  Created on: Jun 16, 2017
 *      Author: david
 */

#ifndef XBOXHANDLER_H_
#define XBOXHANDLER_H_

#include "Arduino.h"

#include "ControllerEnums.h"

#define NUMBER_HATS 4
#define NUMBER_BUTTONS 18

#define DEFAULT_DEADBAND 1025


class XboxHandler {

private:

	union {

		uint8_t rawBuffer[14];
		struct {

			uint16_t checkBytes;
			uint16_t buttonState;
			uint8_t leftTrigger;
			uint8_t rightTrigger;
			int16_t hatValues[4];

		} values;


	} readUnion;

	uint16_t oldButtonState;
	uint16_t oldTriggerState;
	uint16_t buttonClickState;
	uint16_t oldHatState[4];

	bool L2Clicked;
	bool R2Clicked;

	bool newData;




public:

	void handleIncoming(char*);

	boolean isClicked(ButtonMaskEnum);
	boolean isPressed(ButtonMaskEnum);
	int16_t getHatValue(HatEnum);

	boolean newDataAvailable() { return newData; }


};


#endif /* XBOXHANDLER_H_ */
