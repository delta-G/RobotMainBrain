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

#ifndef CONTROLLERFUNCTIONS_H_
#define CONTROLLERFUNCTIONS_H_

#include "Arduino.h"
#include "Defines.h"
#include "Robot.h"

#include "Motor.h"

#include <XboxHandler.h>

#define DEFAULT_DEADZONE 1025

//enum ModeEnum {
//	DRIVE,
//	ARM,
//	MINE,
//	NUMBER_OF_MODES
//};

void initializeControllerFunctions(Robot*, Stream*, Stream*, XboxHandler*);

void mainControllerLoop();
void runStartup();
void returnControl();
void driveWithTwoSticks();
void dpadPanAndTilt();
void driveWithTwoSticksAlg2();
void driveByDpad();
void driveWithOneStick();
void driveWithOneStickAlg2(int, int);

void armButtonMode();

#endif /* CONTROLLERFUNCTIONS_H_ */
