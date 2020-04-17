/*
 * SpeedReport.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: david
 */

#include "SpeedReport.h"

#include "Robot.h"

void reportSpeeds(){

	static unsigned long pm = millis();
	unsigned long cm = millis();

	if (cm - pm >= 100){
		Serial.print("<SR,");
		Serial.print(robot.leftMotor.getSpeed());
		Serial.print(',');
		Serial.print(robot.rightMotor.getSpeed());
		Serial.print(',');

		Serial.print(robot.leftMotor.getPwmSpeed());
		Serial.print(',');
		Serial.print(robot.rightMotor.getPwmSpeed());
		Serial.print(',');


		Serial.print(robot.leftMotor.getTargetSpeed());
		Serial.print(',');
		Serial.print(robot.rightMotor.getTargetSpeed());
		Serial.print('>');

	}


}
