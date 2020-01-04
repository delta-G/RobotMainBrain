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

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Arduino.h"
#include "Defines.h"
#include "Motor.h"
#include "Battery.h"


class Switchable {

private:

	boolean enabled;
	boolean inverted;
	uint8_t pin;

public:

	Switchable(uint8_t aPin):enabled(false), inverted(false), pin(aPin){};
	Switchable(uint8_t aPin, boolean aInv):enabled(false), inverted(aInv), pin(aPin){};

	boolean isEnabled();
	void enable();
	void disable();
	void toggle();

};




class Robot {

private:

	DriveModeEnum driveMode = DRIVE;

public:

	Switchable camera;
	Switchable arm;
	Switchable headlight;
	Switchable comPower;

	Motor leftMotor;
	Motor rightMotor;

	Battery battery;

	uint8_t throttle;

	boolean armPresent;
	boolean armResponding;

	Robot():camera(CAM_ENABLE),
			arm(ARM_ENABLE),
			headlight(HEADLIGHT_PIN),
			comPower(COM_POWER_ENABLE),
			leftMotor(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_ENABLE_PIN, true),
			rightMotor(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_ENABLE_PIN, false),
	        battery(BATTERY_PIN, 1000),
			throttle(255){};

	void init();
	void mainLoop();
	uint8_t* dataDump();
	uint8_t getStatusByte();
	void regularResponse();
	void allStop();

	void setThrottle(uint8_t);
	uint8_t getThrottle();

	void stop();
	void driveForward();
	void driveBackward();
	void spinLeft();
	void spinRight();
	void drive(int16_t, int16_t);
	void setSpeed(int32_t, int32_t);

	void setDriveMode(DriveModeEnum);
	DriveModeEnum getDriveMode();
	DriveModeEnum advanceDriveMode();
};








#endif /* ROBOT_H_ */
