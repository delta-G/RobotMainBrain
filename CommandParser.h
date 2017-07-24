/*
 * CommandParser.h
 *
 *  Created on: Dec 3, 2016
 *      Author: david
 */

#ifndef COMMANDPARSER_H_
#define COMMANDPARSER_H_

#include <Arduino.h>

#define MAX_COMMAND_LENGTH 50
#define START_OP '<'
#define END_OP '>'

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
	char inputBuffer[MAX_COMMAND_LENGTH + 1];
	uint8_t index = 0;
	boolean receiving = false;

	Stream* stream;
	Command* commands;
	uint8_t numCommands;


public:

	void run();
	void parseCommandString();

	CommandParser(Stream* s, Command* c, uint8_t n):stream(s), commands(c), numCommands(n){};




};



#endif /* COMMANDPARSER_H_ */
