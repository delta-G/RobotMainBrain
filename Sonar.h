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
	SWEEP_FORW,
	SWEEP_BACK,
	NUM_SONAR_STATES
};

class Sonar {

private:
	SonarStates state;
	sweepStates sweepState;
	uint8_t sweepIndex;
	boolean dumpSweep = false;

	// Last set of readings taken
	int16_t distance;
	uint16_t curpan;
	uint16_t curtilt;
	//  Joint (name, pin, starting pos, length, min us, min angle, max us, max angle)

	int16_t distances[13];
	Joint panJoint;
	Joint tiltJoint;

	boolean continuousSweep = false;
	uint16_t sweepDelay = 1000;



public:

	GimbalClass gimbal;

	Sonar() : state(NOT_READY), sweepState(STARTING), sweepIndex(0), distance(1234), curpan(0), curtilt(0),
			panJoint(19, 1500, 0, 544, 0, 2400, 3.1415), tiltJoint(18, 1500, 0, 544, 0, 2400, 3.1415),
			gimbal(&panJoint, &tiltJoint){};

	void begin();
	void loop();
	void startPing();
	void stopPing();

	void sweep();
	void startSweep();
	void setContinuous(bool);
	void setSweepDelay(uint16_t);

	uint8_t* dataDump();

	int16_t getDistance();

};




#endif /* SONAR_H_ */
