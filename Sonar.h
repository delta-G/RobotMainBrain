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

#ifndef SONAR_H_
#define SONAR_H_


#include "Arduino.h"
#include <PingTimer.h>
#include <Gimbal.h>
#include <Joint.h>


//  Let's keep a reading for every 15 degrees.  That gives us 12 sections over our
//  180 degree range.  Plus the starting "fencepost" and we have 13 readings.
// They are at, 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180

enum sweepStates {
	STARTING,
	MOVING,
	DELAYING,
	PINGING,
	SWEEP_DELAY
};

enum SonarStates {
	NOT_READY,
	NOT_RUNNING,
	HOLDING,
	HOLD_DELAY,
	SWEEP_FORW,
	SWEEP_BACK,
	NUM_SONAR_STATES
};

class Sonar {

private:
	SonarStates state = NOT_READY;
	sweepStates sweepState = STARTING;
	uint8_t sweepIndex = 0;
	boolean dumpSweep = false;
	boolean scanDone = false;

	boolean newDump = false;

	// Last set of readings taken
	int16_t distance = -2;
	uint16_t curpan = 0;
	uint16_t curtilt = 0;

	int16_t distances[13];
	Joint panJoint;
	Joint tiltJoint;

	boolean continuousSweep = false;
	uint16_t sweepDelay = 1000;

	uint16_t holdDelay = 200;

	float minAngle = 0.0;
	float maxAngle = 3.14159;

	void (*sweepCallback)() = 0;
	void (*pingCallback)() = 0;



public:

	GimbalClass gimbal;
	//  Joint (name, pin, starting pos, length, min us, min angle, max us, max angle)
	Sonar() : panJoint(19, 1500, 0, 544, 0, 2400, 3.1415), tiltJoint(18, 1500, 0, 544, 0, 2400, 3.1415), sweepCallback(NULL), pingCallback(NULL), gimbal(&panJoint, &tiltJoint){};

	void setSweepCallback(void (*aCallback)());
	void setPingCallback(void (*aCallback)());

	void begin();
	void loop();
	void startPing();
	void stopPing();



	void sweep();
	void startSweep();
	void setContinuous(bool);
	bool getContinuous(){return continuousSweep;}
	bool isHolding();
	void setSweepDelay(uint16_t);
	void setHoldDelay(uint16_t);

	void parkSensor();

	uint8_t* dataDump();

	int16_t getDistance();
	int16_t getDistance(uint8_t);

	boolean scanFinished();
	boolean hasNewDump();

	void setMinAngle(float);
	void setMaxAngle(float);
	float getMinAngle();
	float getMaxAngle();



};




#endif /* SONAR_H_ */
