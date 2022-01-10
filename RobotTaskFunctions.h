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

#ifndef ROBOTTASKFUNCTIONS_H_
#define ROBOTTASKFUNCTIONS_H_

#include "Arduino.h"
#include "Robot.h"

//  TaskFunc will return false if it wants to be called again on the next loop and true once it has completed it's task
//  It will take a uint8_t as the state with 0 being entry, 1 being looping, and 2 being exit.
//  The actual task loop may have other states in the looping area, but this allows us to use 1 function for entry through exit of a task.
//  return true in state 0 and it never attaches to the callback.
//  return false in the exit condition and it will restart the whole process.

namespace Task {

enum TaskFuncStates {TF_ENTRY, TF_LOOPING, TF_EXIT};
typedef boolean (*TaskFunc)(TaskFuncStates);


boolean startTask(TaskFunc);

boolean taskLoop();

boolean exitTask(TaskFunc);



boolean turnOffArm(TaskFuncStates);
boolean turnOnArm(TaskFuncStates);

}
#endif /* ROBOTTASKFUNCTIONS_H_ */
