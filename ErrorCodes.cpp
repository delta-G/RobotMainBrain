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


#include "ErrorCodes.h"


void sendError(uint8_t aErrorCode){
	Serial.print("<Z");
	// Discobot's parser will halt on anything larger than 127 in an ascii command
	// if we ever need more than 128 error codes then we will have to revisit this.
	if(aErrorCode < 128){
		Serial.write(aErrorCode);
	} else {
		Serial.write(ECODE_BAD_ERROR_CODE);
	}
	Serial.print(">");
}
