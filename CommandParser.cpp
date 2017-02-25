/*
 * CommandParser.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: david
 */

#include "CommandParser.h"

void CommandParser::run() {

	if (stream->available()) {
		char c = stream->read();

		if (c == START_OP) {
			index = 0;
			inputBuffer[index] = 0;
			receiving = true;
		}
		if (receiving) {
			inputBuffer[index] = c;
			inputBuffer[++index] = 0;

			if (c == END_OP) {
				receiving = false;
				parseCommandString();
			}

		}

	}

}

void CommandParser::parseCommandString() {
	if (inputBuffer[0] == START_OP) {
		for(int i = 0; i < numCommands; i++) {
			if (commands[i].match(inputBuffer + 1)) {
				commands[i].run(inputBuffer);
				return;
			}
		}

	}
}



