/*
 * ControllerFunctions.h
 *
 *  Created on: Jun 17, 2017
 *      Author: david
 */

#ifndef CONTROLLERFUNCTIONS_H_
#define CONTROLLERFUNCTIONS_H_

#include "Arduino.h"
#include "Defines.h"

#include "Motor.h"

#include "XboxHandler.h"

#define DEFAULT_DEADZONE 1025

enum ModeEnum {
	DRIVE,
	ARM,
	MINE,
	NUMBER_OF_MODES
};

void initializeControllerFunctions(Motor*, Motor*, Stream*, Stream*, XboxHandler*);

void mainControllerLoop();
void runStartup();
void returnControl();
void driveWithTwoSticks();
void driveByDpad();


#endif /* CONTROLLERFUNCTIONS_H_ */
