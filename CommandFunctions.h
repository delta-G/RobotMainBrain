/*
 * CommandFunctions.h
 *
 *  Created on: Dec 3, 2016
 *      Author: david
 */

#ifndef COMMANDFUNCTIONS_H_
#define COMMANDFUNCTIONS_H_

#include <Arduino.h>
#include "Defines.h"
#include "CommandParser.h"
//#include "Joint.h"

#ifndef NUM_ELEMENTS
#define NUM_ELEMENTS(x) ((sizeof(x)) / (sizeof(x[0])))
#endif

void enableArm(char*);

void requestFromBot(char*);
void motorControl(char*);
void armControl(char*);



#endif /* COMMANDFUNCTIONS_H_ */
