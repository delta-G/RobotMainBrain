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

#include "Robot.h"

Robot robot;

boolean Switchable::isEnabled() {
	return enabled;
}

void Switchable::enable() {
	if (pin < 100) {
		digitalWrite(pin, inverted ? LOW : HIGH);
	} else if (pin < 108) {
		robot.xpander.writePin(pin - 100, inverted ? LOW : HIGH);
	} else if (pin < 116) {
		robot.powerXpander.writePin(pin - 108, inverted ? LOW : HIGH);
	}

	enabled = true;
}

void Switchable::disable() {
	if (pin < 100) {
			digitalWrite(pin, inverted ? HIGH : LOW);
		} else if (pin < 108) {
			robot.xpander.writePin(pin - 100, inverted ? HIGH : LOW);
		} else if (pin < 116) {
			robot.powerXpander.writePin(pin - 108, inverted ? HIGH : LOW);
		}
	enabled = false;
}

void Switchable::toggle(){
	if(enabled){
		disable();
	} else {
		enable();
	}
}

void Switchable::init(uint8_t aState) {
	if (pin < 100) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, aState);
	} else if (pin < 108) {
		robot.xpander.setPinMode(pin - 100, OUTPUT);
		robot.xpander.writePin(pin - 100, aState);
	} else if (pin < 116) {
		robot.powerXpander.setPinMode(pin - 108, OUTPUT);
		robot.powerXpander.writePin(pin - 108, aState);
	}
	if(aState != inverted){
		enabled = true;
	} else {
		enabled = false;
	}
}



void Robot::init() {

	pinMode(XPANDER_RESET_PIN, OUTPUT);
	digitalWrite(XPANDER_RESET_PIN, HIGH);
	xpander.init();
	powerXpander.init();
	powerADC.init();

	xpander.writeRegister(GPINTENA, ((1<<LEFT_MOTOR_SF_PIN)|(1<<RIGHT_MOTOR_SF_PIN)));
	xpander.writeRegister(INTCONA, 0);


	camera.init(LOW);
	arm.init(HIGH);
	headlight.init(LOW);
	comPower.init(LOW);

	motorPower.init(LOW);
	motorController.init(LOW);
	v12Power.init(LOW);
	auxPower.init(LOW);
	sonarPower.init(LOW);

	leftMotor.init();
	rightMotor.init();
	sonar.begin();
//	battery.initReadings();
}

void Robot::mainLoop(){
	if(driveMode == AUTO){
		autoLoop();
	}
	readSupplyVoltages();
	leftMotor.loop();
	rightMotor.loop();
	sonar.loop();
}


void Robot::autoLoop() {

	int16_t leftOut = 0;
	int16_t rightOut = 0;

	if(randomWalking){
		randomWalk();
	}

	if (runLeftToTarget || runRightToTarget) {

		if (runLeftToTarget) {
			if (leftTarget > leftMotor.encoder.getTicks()) {
				leftOut = 255;
			} else if (leftTarget < leftMotor.encoder.getTicks()) {
				leftOut = -255;
			} else {
				runLeftToTarget = false;
				leftOut = 0;
			}
		}
		if (runRightToTarget) {
			if (rightTarget > rightMotor.encoder.getTicks()) {
				rightOut = 255;
			} else if (rightTarget < rightMotor.encoder.getTicks()) {
				rightOut = -255;
			} else {
				runRightToTarget = false;
				rightOut = 0;
			}
		}

		drive(leftOut, rightOut);
	}
}

void Robot::startRandomWalk(){
	randomWalkState = rws_STARTING;
	randomWalking = true;
	setDriveMode(AUTO);
}


void Robot::randomWalk() {

	static uint32_t delayStart = 0;

	switch (randomWalkState) {
	case rws_STARTING:
		sonar.startSweep();
		randomWalkState = rws_SCANNING;
		break;
	case rws_SCANNING:
		if (sonar.scanFinished()) {
			// Find which direction has greatest distance
			if (sonar.getDistance(0) >= sonar.getDistance(12)) {
				if (sonar.getDistance(6) > sonar.getDistance(0)) {
					randomWalkState = rws_TURNING;
				} else {
					// turn right
					driveTicks(80, -80);
					randomWalkState = rws_TURNING;
				}
			} else {
				if (sonar.getDistance(6) > sonar.getDistance(12)) {
					randomWalkState = rws_TURNING;
				} else {
					//turn left
					driveTicks(-80, 80);
					randomWalkState = rws_TURNING;
				}
			}
		}
		break;
	case rws_TURNING:
		if (!(runLeftToTarget || runRightToTarget)) {
			sonar.gimbal.setPanAngle(1.5708);
			sonar.startPing();
			delayStart = millis();
			randomWalkState = rws_DELAY;
		}
		break;
	case rws_DELAY:
		if (millis() - delayStart >= 500) {
			randomWalkState = rws_MOVING;
		}
		break;
	case rws_MOVING:
		if (sonar.getDistance() < 300) {
			stop();
			sonar.startSweep();
			randomWalkState = rws_SCANNING;
		} else {
			driveForward();
		}
		break;
	}

}

void Robot::silenceHeartbeat(){
	heartSilenced = true;
	digitalWrite(HEARTBEAT_PIN, LOW);
}

void Robot::restartHeartbeat(){
	heartSilenced = false;
}

void Robot::allStop(){

	stop();
	Serial1.print("<CX>");  // stops the arm

}

void Robot::readSupplyVoltages(){

//	uint16_t battery = powerADC.read(BATTERY_ADC_PIN) * BATTERY_ADC_CAL_FACTOR;
//	uint16_t V12 = powerADC.read(V12_ADC_PIN) * V12_ADC_CAL_FACTOR;
//	uint16_t aux = powerADC.read(AUX_ADC_PIN) * AUX_ADC_CAL_FACTOR;
//	uint16_t main5 = powerADC.read(MAIN5_ADC_PIN) * MAIN5_ADC_CAL_FACTOR;
//	uint16_t radio = powerADC.read(RADIO_ADC_PIN) * RADIO_ADC_CAL_FACTOR;
//	uint16_t motor = (analogRead(0) + 29.64) / 0.05132;

	static unsigned long pm = millis();
	unsigned long cm = millis();
	if (cm - pm >= VOLTAGE_READING_INTERVAL) {
		pm = cm;
		for (int i = 0; i < 6; i++) {
			uint16_t volts;

			if (i < 5) {
				// first five are on the ADC chip
				volts = powerADC.read(voltagePins[i]) * voltageCals[i];
			} else {
				// the motor voltage reads on an analog pin on the 1284 chip
				volts = (analogRead(0) + 29.64) / 0.05132;
			}
			// if voltage has changed by more than 10% we need to report to DiscoBot
			if (fabs((float) voltages[i] - (float) volts)
					> (voltages[i] * 0.10)) {
				voltageReportNeeded = true;
			}
			voltages[i] = volts;
		}
	}
	if (millis() - lastVoltageReportMillis >= VOLTAGE_REPORTING_INTERVAL) {
		voltageReportNeeded = true;
	}
}


uint8_t* Robot::reportSupplyVoltages(){
	lastVoltageReportMillis = millis();
	static uint8_t data[16];
	data[0] = '<';
	data[1] = 0x13;
	data[2] = 16;
	data[3] = (voltages[VOLT_ENUM_BATTERY] >> 8);
	data[4] = (voltages[VOLT_ENUM_BATTERY] & 0xFF);
	data[5] = (voltages[VOLT_ENUM_MOTOR] >> 8);
	data[6] = (voltages[VOLT_ENUM_MOTOR] & 0xFF);
	data[7] = (voltages[VOLT_ENUM_MAIN5] >> 8);
	data[8] = (voltages[VOLT_ENUM_MAIN5] & 0xFF);
	data[9] = (voltages[VOLT_ENUM_RADIO] >> 8);
	data[10] = (voltages[VOLT_ENUM_RADIO] & 0xFF);
	data[11] = (voltages[VOLT_ENUM_AUX] >> 8);
	data[12] = (voltages[VOLT_ENUM_AUX] & 0xFF);
	data[13] = (voltages[VOLT_ENUM_V12] >> 8);
	data[14] = (voltages[VOLT_ENUM_V12] & 0xFF);
	data[15] = '>';

	for (int i = 0; i < 16; i++) {
		Serial.write(data[i]);
	}

	return data;

}

uint8_t* Robot::dataDump() {

	Serial.print(HBOR_STRING);

	static uint8_t data[ROBOT_DATA_DUMP_SIZE];

	data[0] = '<';
	data[1] = 0x13;
	data[2] =  ROBOT_DATA_DUMP_SIZE;
	data[3] = getStatusByte1();
	data[4] = getStatusByte2();
	data[5] = throttle;
	data[6] = (byte) /*(battery.getVoltage() * 10)*/ 1;  // THIS SPACE CURRENTLY OPEN
	data[7] = (byte) ((leftMotor.encoder.getTicks() >> 8) & 0xFF);
	data[8] = (byte) (leftMotor.encoder.getTicks() & 0xFF);
	data[9] = (byte) ((leftMotor.getSpeed() >> 8) & 0xFF);
	data[10] = (byte) (leftMotor.getSpeed() & 0xFF);
	data[11] = (byte) (abs(leftMotor.getPwmSpeed()) & 0xFF);
	data[12] = (byte) ((rightMotor.encoder.getTicks() >> 8) & 0xFF);
	data[13] = (byte) (rightMotor.encoder.getTicks() & 0xFF);
	data[14] = (byte) ((rightMotor.getSpeed() >> 8) & 0xFF);
	data[15] = (byte) (rightMotor.getSpeed() & 0xFF);
	data[16] = (byte) (abs(rightMotor.getPwmSpeed()) & 0xFF);
	data[17] = (byte) 0;  // bot snr
	data[18] = (byte) 0;  // bot rssi
	data[19] = (byte) 0;  // base snr
	data[20] = (byte) 0;  // base rssi

	data[21] = '>';

	for(int i=0; i<ROBOT_DATA_DUMP_SIZE; i++){
		Serial.write(data[i]);
	}

	return data;

}

uint8_t Robot::getStatusByte1(){

	uint8_t retval = 0;

	switch(driveMode){
	case DRIVE:
		retval |= 0x01;
		break;
	case ARM:
		retval |= 0x02;
		break;
	case MINE:
		retval |= 0x03;
		break;
	case AUTO:
		retval |= 0x00;
		break;
	default:
		break;
	}
	if (armPresent) {
		retval |= 0x04;
	}
	if (armResponding) {
		retval |= 0x08;
	}

	if (camera.isEnabled()) {
		retval |= 0x10;
	}
	if (headlight.isEnabled()) {
		retval |= 0x20;
	}
	if (arm.isEnabled()) {
		retval |= 0x40;
	}
	if (comPower.isEnabled()) {
		retval |= 0x80;
	}


	return retval;

}

uint8_t Robot::getStatusByte2(){
	uint8_t retval = 0;

	if(motorPower.isEnabled()){
		retval |= 0x01;
	}
	if(motorController.isEnabled()){
		retval |= 0x02;
	}
	if(v12Power.isEnabled()){
		retval |= 0x04;
	}
	if(auxPower.isEnabled()){
		retval |= 0x08;
	}
	if(sonarPower.isEnabled()){
		retval |= 0x10;
	}

	return retval;
}


void Robot::regularResponse(){

	static uint8_t counter = 0;

	switch (counter++) {

	case 1:
		if (sonar.hasNewDump()) {
			sonar.dataDump();
			break;
		} else {
			counter++; /* no break */
		}
		/* no break */

	case 2: {
		boolean gotNew = newArmData;
		newArmData = false;
		if (gotNew) {
			for (int i = 0; i < ARM_DUMP_SIZE; i++) {
				Serial.write(armDumpBuffer[i]);
			}
		}
		if (armResponding) {
			if (gotNew && armDumpBuffer[4] == 't') {
				Serial1.print("<A,Rp>");
			} else {
				Serial1.print("<A,RR>");
			}
			break;
		} else {
			counter++; /* no break */
		}
	}
		/* no break */
	case 3:
		if (voltageReportNeeded) {
			reportSupplyVoltages();
			break;
		} else {
			counter++; /* no break */
		}
		/* no break */
	case 0:
		dataDump();
		break;
	}


	if (counter >= 4){
		counter = 0;
	}
}

boolean Robot::checkMotorStatus(){
	uint8_t readByte = xpander.readIO();
	// bits 3 and 4 are the two status flags.  Lets return true if either is low
	return !((readByte & (1<<3)) && (readByte & (1<<4)));
}

void Robot::setThrottle(uint8_t aLevel){
	throttle = aLevel;
}

uint8_t Robot::getThrottle(){
	return throttle;
}

void Robot::setMinPWM(uint8_t aVal){
	minPWM = aVal;
	leftMotor.setMinPWM(aVal);
	rightMotor.setMinPWM(aVal);
}

uint8_t Robot::getMinPWM(){
	return minPWM;
}

void Robot::stop() {
	leftMotor.stop();
	rightMotor.stop();
	runLeftToTarget = false;
	runRightToTarget = false;
}

void Robot::driveForward() {
	drive(255,255);
}

void Robot::driveTicks(int32_t aLeft, int32_t aRight) {
	// Use 0 to leave one motor alone and run the other.
	if (aLeft != 0) {
		int32_t leftCurrent = leftMotor.encoder.getTicks();
		leftTarget = leftCurrent + aLeft;
		runLeftToTarget = true;
	}
	if (aRight != 0) {
		int32_t rightCurrent = rightMotor.encoder.getTicks();
		rightTarget = rightCurrent + aRight;
		runRightToTarget = true;
	}
	setDriveMode(AUTO);

}

void Robot::driveBackward() {
	drive(-255,-255);
}
void Robot::spinLeft() {
	drive(-255,255);
}
void Robot::spinRight() {
	drive(255,-255);
}
void Robot::drive(int16_t aLeft, int16_t aRight) {
	float ratio = throttle / 255.0;
	leftMotor.drive(aLeft * ratio);
	rightMotor.drive(aRight * ratio);
}
void Robot::setSpeed(int32_t aLeft, int32_t aRight) {
	leftMotor.setSpeed(aLeft);
	rightMotor.setSpeed(aRight);
}


void Robot::saveLastRawCommand(uint8_t* p){
	memcpy(robot.lastRawCommand, p, XBOX_RAW_BUFFER_SIZE);
}

void Robot::saveArmReport(uint8_t* p){
	memcpy(robot.armDumpBuffer, p, ARM_DUMP_SIZE);
	newArmData = true;
}

void Robot::setDriveMode(DriveModeEnum aDriveMode) {
	driveMode = aDriveMode;
	if (driveMode >= NUMBER_OF_MODES) {
		driveMode = (DriveModeEnum) ((int) driveMode % NUMBER_OF_MODES);
	}
	if (driveMode != AUTO){
		randomWalking = false;
		runLeftToTarget = false;
		runRightToTarget = false;
	}
	if (armResponding) {
		switch (driveMode) {
		case DRIVE:
			Serial1.print("<A,CMD>");
			break;
		case ARM:
			Serial1.print("<A,CMA>");
			break;
		case MINE:
			Serial1.print("<A,CMM>");
			break;
		case AUTO:
			Serial1.print("<A,CMT>");
			break;
		default:
			break;
		}
	}
}

DriveModeEnum Robot::getDriveMode(){
	return driveMode;
}

DriveModeEnum Robot::advanceDriveMode(){
	DriveModeEnum dm = driveMode;
	dm = (DriveModeEnum)((int)dm + 1);
	setDriveMode(dm);
	return driveMode;
}

