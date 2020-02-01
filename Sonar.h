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

class Sonar {

private:

	int16_t distance;
	//  Joint (name, pin, starting pos, length, min us, min angle, max us, max angle)
	Joint panJoint;
	Joint tiltJoint;

	GimbalClass gimbal;

public:

	Sonar() : distance(1234),panJoint(19, 1500, 0, 544, 0, 2400, 3.1415), tiltJoint(18, 1500, 0, 544, 0, 2400, 3.1415), gimbal(&panJoint, &tiltJoint){};

	void begin();
	void loop();
	void startPing();

	uint8_t* dataDump();

	int16_t getDistance();
	uint16_t getPan();
	uint16_t getTilt();
	void setPan(uint16_t);
	void setTilt(uint16_t);

	void setPanSpeed(uint16_t);
	void setTiltSpeed(uint16_t);

};




#endif /* SONAR_H_ */
