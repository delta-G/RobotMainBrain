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

#ifndef XBOXHANDLER_H_
#define XBOXHANDLER_H_

#include "Arduino.h"
#include "Defines.h"

#include "ControllerEnums.h"

#define NUMBER_HATS 4
#define NUMBER_BUTTONS 18

#define DEFAULT_DEADBAND 1025


class XboxHandler {

private:

	ControllerUnion readUnion;  // ControllerUnion is in RobotSharedDefines

	uint16_t oldButtonState;
	uint16_t oldTriggerState;
	uint16_t buttonClickState;
	uint16_t oldHatState[4];

	bool L2Clicked;
	bool R2Clicked;

	bool newData;




public:

	XboxHandler();

	void handleIncoming(char*);
	void handleIncomingASCII(char*);

	boolean isClicked(ButtonMaskEnum);
	boolean isPressed(ButtonMaskEnum);
	int16_t getHatValue(HatEnum);
	uint8_t getTriggerValue(ButtonMaskEnum);

	boolean newDataAvailable();


};


#endif /* XBOXHANDLER_H_ */
