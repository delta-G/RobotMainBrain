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
	battery.initReadings();
}

void Robot::mainLoop(){
	if(driveMode == AUTO){
		autoLoop();
	}
	battery.monitor();
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

void Robot::allStop(){

	stop();
	Serial1.print("<CX>");  // stops the arm

}

void Robot::readSupplyVoltages(){

	char data[40];
	uint16_t battery = powerADC.read(BATTERY_ADC_PIN);
	uint16_t V12 = powerADC.read(V12_ADC_PIN);
	uint16_t aux = powerADC.read(AUX_ADC_PIN);
	uint16_t main5 = powerADC.read(MAIN5_ADC_PIN);
	uint16_t radio = powerADC.read(RADIO_ADC_PIN);
	uint16_t motor = analogRead(0);

	snprintf(data, 32, "<VR,%i,%i,%i,%i,%i,%i>", battery, V12, aux, main5, radio, motor);
	Serial.print(data);
}

uint8_t* Robot::dataDump() {

	Serial.print(HBOR_STRING);

	static uint8_t data[ROBOT_DATA_DUMP_SIZE];

	data[0] = '<';
	data[1] = 0x13;
	data[2] =  ROBOT_DATA_DUMP_SIZE;
	data[3] = getStatusByte();
	data[4] = throttle;
	data[5] = (byte) (battery.getVoltage() * 10);
	data[6] = (byte) ((leftMotor.encoder.getTicks() >> 8) & 0xFF);
	data[7] = (byte) (leftMotor.encoder.getTicks() & 0xFF);
	data[8] = (byte) ((leftMotor.getSpeed() >> 8) & 0xFF);
	data[9] = (byte) (leftMotor.getSpeed() & 0xFF);
	data[10] = (byte) (abs(leftMotor.getPwmSpeed()) & 0xFF);
	data[11] = (byte) ((rightMotor.encoder.getTicks() >> 8) & 0xFF);
	data[12] = (byte) (rightMotor.encoder.getTicks() & 0xFF);
	data[13] = (byte) ((rightMotor.getSpeed() >> 8) & 0xFF);
	data[14] = (byte) (rightMotor.getSpeed() & 0xFF);
	data[15] = (byte) (abs(rightMotor.getPwmSpeed()) & 0xFF);
	data[16] = (byte) 0;  // bot snr
	data[17] = (byte) 0;  // bot rssi
	data[18] = (byte) 0;  // base snr
	data[19] = (byte) 0;  // base rssi

	data[20] = '>';

	for(int i=0; i<ROBOT_DATA_DUMP_SIZE; i++){
		Serial.write(data[i]);
	}

	return data;

}

uint8_t Robot::getStatusByte(){

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

/*  TODO:
 * This is all messed up.  It wastes cycles if the arm isn't there
 * and we need code so that if the arm isn't moving we keep sending
 * the robot dump instead of the same data over and over from the arm
 */
void Robot::regularResponse(){

	static uint8_t counter = 0;

	switch (counter++) {
	case 0:
		dataDump();
		break;
	case 1:
		if (sonar.hasNewDump()) {
			sonar.dataDump();
			break;
		} else {
			counter++; /* no break */
		}
		/* no break */
	case 2:
		if (armResponding) {
			Serial1.print("<A,Rp>");
		}
		break;
	case 3:
		if (armResponding) {
			Serial1.print("<A,Rt>");
		}
		break;
	case 4:
		if (armResponding) {
			Serial1.print("<A,Rs>");
		}
		break;
	}

	if (counter >= 5){
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

