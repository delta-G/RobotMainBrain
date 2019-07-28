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

#ifndef COMMANDPARSER_H_
#define COMMANDPARSER_H_

#include <Arduino.h>

#ifndef COM_PARSER_MAX_COMMAND_LENGTH
#define COM_PARSER_MAX_COMMAND_LENGTH 50
#endif

typedef void (*commandFunc)(char*);

struct Command {
	char matchChar;
	commandFunc function;

	Command(char m, commandFunc f):matchChar(m), function(f){};
	void run(char* com){function(com);}
	boolean match(char* com){
		return ((*com == matchChar) || (matchChar == '#' && *com >= '0' && *com <= '9'));
	}
};

class CommandParser {

private:
	char inputBuffer[COM_PARSER_MAX_COMMAND_LENGTH + 1];
	uint8_t index = 0;
	boolean receiving = false;

	Command* commands;
	uint8_t numCommands;

	boolean hasStartMarker;


public:

	void parseCommandString(char* aCommand);

	CommandParser(Command* c, uint8_t n):commands(c), numCommands(n), hasStartMarker(false){};
	CommandParser(Command* c, uint8_t n, boolean m):commands(c), numCommands(n), hasStartMarker(m){};
};



#endif /* COMMANDPARSER_H_ */
