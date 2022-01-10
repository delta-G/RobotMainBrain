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



#include "RobotTaskFunctions.h"

namespace Task{

TaskFunc currentTask = NULL;


boolean taskLoop(){
	boolean retval = true;
	if(currentTask != NULL){
		retval = currentTask(TF_LOOPING);
		//  if task returns true then run the exit state.  If it's still true then detach
		//  the exit state may restart the task and then return false so we allow it to change retval
		if(retval){
			retval = exitTask(currentTask);
		}
	}
	return retval;
}


boolean startTask(TaskFunc aFunc){
	// if aFunc doesn't exit as done = true on the first go then setup to keep calling
	boolean retval = aFunc(TF_ENTRY);
	if(!retval){
		currentTask = aFunc;
		robot.setTaskLoopCallback(taskLoop);
	}
	return retval;
}


boolean exitTask(TaskFunc aFunc){
	// run the exit state of the function and see if it wants to restart
	boolean retval = aFunc(TF_EXIT);
	if (retval){
		currentTask = NULL;
	} else {
		retval = currentTask(TF_ENTRY);
	}
	return retval;
}







/************************************************
 * **********************************************
 * ***********************************************
 * ********T ASK FUNCTIONS  *********************
 * ********************************************
 * ******************************************
 * ***********************************************
 */


boolean turnOffArm(TaskFuncStates aState){

	boolean retval = false;  // assume recall, change to true to exit and true again in exit state

	switch (aState){

	case TF_ENTRY:
		// Park arm and set flag to wait for movement
		robot.movementDoneWait = true;
		Serial1.print("<A,CQ>");
		break;

	case TF_LOOPING:
		if(!robot.movementDoneWait){
			// When parking is done
			retval = true;  //  Will move us to EXIT STATE
		}
		break;

	case TF_EXIT:
		Serial1.end();
		pinMode(10, INPUT);
		pinMode(11, INPUT);
		digitalWrite(10, LOW);
		digitalWrite(11, LOW);
		robot.arm.disable();
		retval = true;
		break;
	}
	return retval;
}

boolean turnOnArm(TaskFuncStates aState){

	boolean retval = false;  // assume recall, change to true to exit and true again in exit state
	static uint32_t startTime = 0;
	static uint8_t internalState = 0;

	switch (aState){

	case TF_ENTRY:
		robot.silenceHeartbeat();
		robot.arm.enable();
		robot.restartHeartbeat();
		startTime = millis();
		internalState = 0;
		break;

	case TF_LOOPING:
		switch(internalState){
		case 0:
			if(millis() - startTime >= ARM_BOOT_INIT_WAIT){
				Serial1.begin(ARM_BOARD_BAUD);
				internalState = 1;
				startTime = millis();
			}
			break;
		case 1:
			if(robot.armPresent && robot.armResponding){
				retval = true;
			}
			if(millis() - startTime >= ARM_BOOT_TIMEOUT){
				Serial1.end();
				pinMode(10, INPUT);
				pinMode(11, INPUT);
				digitalWrite(10, LOW);
				digitalWrite(11, LOW);
				sendError(ECODE_ARM_BOOT_TIMEOUT);
			}
			break;
		}
		break;

	case TF_EXIT:
		Serial1.print("<A,RR>");
		retval = true;
		break;
	}
	return retval;
}

}



//boolean templateToCopyAndPaste(TaskFuncStates aState){
//
//	boolean retval = false;  // assume recall, change to true to exit and true again in exit state
//
//	switch (aState){
//
//	case TF_ENTRY:
//
//		break;
//
//	case TF_LOOPING:
//
//		break;
//
//	case TF_EXIT:
//
//      retval = true;  // unless you want to call this whole process again. //
//
//		break;
//	}
//	return retval;
//}

