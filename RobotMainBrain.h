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


#ifndef _RobotMainBrain_H_
#define _RobotMainBrain_H_
#include "Arduino.h"

#include "Defines.h"

#include "Robot.h"

#include <StreamParser.h>

#include "CommandParser.h"
#include "CommandFunctions.h"

#include "ControllerFunctions.h"

#include "Motor.h"

#include "githash.h"

#include "Encoder.h"

void setup();
void bootup();
void loop();
void heartBeat();
//void monitorBattery();






#endif /* _RobotMainBrain_H_ */
