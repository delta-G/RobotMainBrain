/*
 * ControllerFunctions.h
 *
 *  Created on: Jun 17, 2017
 *      Author: david
 */

#ifndef CONTROLLERFUNCTIONS_H_
#define CONTROLLERFUNCTIONS_H_

#include "Arduino.h"

#include "XboxHandler.h"

#define DEFAULT_DEADZONE 1025

enum ModeEnum {
	DRIVE,
	ARM,
	MINE,
	NUMBER_OF_MODES
} controlMode;

void mainControllerLoop();
void runStartup();
void returnControl();
void driveWithTwoSticks();


#endif /* CONTROLLERFUNCTIONS_H_ */
