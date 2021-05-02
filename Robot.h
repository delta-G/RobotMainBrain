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
#include "Sonar.h"

#include <MCP23S08.h>
#include <MCP3008.h>

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
	void init(uint8_t);

};


enum RandomWalkStates {rws_STARTING, rws_SCANNING, rws_TURNING, rws_DELAY, rws_MOVING};

class Robot {

private:

	DriveModeEnum driveMode = DRIVE;

	RandomWalkStates randomWalkState = rws_STARTING;
	boolean randomWalking = false;

public:

	boolean heartSilenced;

	MCP23S08 xpander;
	MCP23S08 powerXpander;

	MCP3008 powerADC;

	Switchable camera;
	Switchable arm;
	Switchable headlight;
	Switchable comPower;

	Switchable motorPower;
	Switchable motorController;
	Switchable v12Power;
	Switchable auxPower;
	Switchable sonarPower;

	Motor leftMotor;
	Motor rightMotor;

//	Battery battery;

	boolean voltageReportNeeded = false;
	uint32_t lastVoltageReportMillis;

	uint16_t voltages[6];
	uint16_t voltagePins[6] = {BATTERY_ADC_PIN, V12_ADC_PIN, AUX_ADC_PIN, MAIN5_ADC_PIN, RADIO_ADC_PIN, 0};
	float voltageCals[6] = {BATTERY_ADC_CAL_FACTOR, V12_ADC_CAL_FACTOR, AUX_ADC_CAL_FACTOR, MAIN5_ADC_CAL_FACTOR, RADIO_ADC_CAL_FACTOR, 1};

	Sonar sonar;

	uint8_t throttle = 255;  // maxPWM
	uint8_t minPWM = 127;

	boolean armPresent = false;
	boolean armResponding = false;

	int32_t leftTarget = 0;
	boolean runLeftToTarget = false;
	int32_t rightTarget = 0;
	boolean runRightToTarget = false;

	uint8_t lastRawCommand[XBOX_RAW_BUFFER_SIZE];
	uint8_t armDumpBuffer[ARM_DUMP_SIZE];
	boolean newArmData = false;


	Robot():heartSilenced(true),
			xpander(XPANDER_CS_PIN, MB_XPANDER_HW_ADDY),
			powerXpander(XPANDER_CS_PIN, POWER_XPANDER_HW_ADDY),
			powerADC(POWER_ADC_CS_PIN),
			camera(CAM_ENABLE_PIN),
			arm(ARM_ENABLE_PIN, true),
			headlight(HEADLIGHT_PIN),
			comPower(COM_POWER_ENABLE_PIN),
			motorPower(MOTOR_POWER_ENABLE_PIN),
			motorController(MOTOR_CONTROLLER_ENABLE_PIN),
			v12Power(V12_ENABLE),
			auxPower(AUX_POWER_ENABLE_PIN),
			sonarPower(SONAR_ENABLE_PIN),
			leftMotor(LEFT_MOTOR_DIRECTION_PIN_1, LEFT_MOTOR_DIRECTION_PIN_2, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_FEEDBACK_PIN, true),
			rightMotor(RIGHT_MOTOR_DIRECTION_PIN_1, RIGHT_MOTOR_DIRECTION_PIN_2, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_FEEDBACK_PIN, false),
			sonar(){};

	void silenceHeartbeat();
	void restartHeartbeat();

	void init();
	void mainLoop();
	void autoLoop();

	void startRandomWalk();
	void randomWalk();

	void readSupplyVoltages();
	uint8_t* reportSupplyVoltages();
	uint8_t* dataDump();
	uint8_t getStatusByte1();
	uint8_t getStatusByte2();
	void regularResponse();
	void allStop();

	void setThrottle(uint8_t);
	uint8_t getThrottle();
	uint8_t getMinPWM();
	void setMinPWM(uint8_t);

	boolean checkMotorStatus();

	void stop();
	void driveForward();
	void driveTicks(int32_t aLeft, int32_t aRight);
	void driveBackward();
	void spinLeft();
	void spinRight();
	void drive(int16_t, int16_t);
	void setSpeed(int32_t, int32_t);

	void saveLastRawCommand(uint8_t*);
	void saveArmReport(uint8_t*);

	void setDriveMode(DriveModeEnum);
	DriveModeEnum getDriveMode();
	DriveModeEnum advanceDriveMode();
};


extern Robot robot;





#endif /* ROBOT_H_ */
