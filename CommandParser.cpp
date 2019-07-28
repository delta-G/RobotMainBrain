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

#include "CommandParser.h"

//void CommandParser::run() {
//
//	if (stream->available()) {
//		char c = stream->read();
//
//		if (c == START_OF_PACKET) {
//			index = 0;
//			inputBuffer[index] = 0;
//			receiving = true;
//		}
//		if (receiving) {
//			inputBuffer[index] = c;
//			inputBuffer[++index] = 0;
//			if(index >= MAX_COMMAND_LENGTH){
//				index--;
//			}
//
//			if (c == END_OF_PACKET) {
//				receiving = false;
//				parseCommandString();
//			}
//
//		}
//
//	}
//
//}

void CommandParser::parseCommandString(char* aCommand) {
	if (aCommand[0] == START_OF_PACKET) {
		for(int i = 0; i < numCommands; i++) {
			if (commands[i].match(aCommand + 1)) {
				commands[i].run(aCommand);
				return;
			}
		}

	}
}



