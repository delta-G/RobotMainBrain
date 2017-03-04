#include "RobotMainBrain.h"

extern CommandParser cp;

void setup()
{

	Serial.begin(19200);

	pinMode(RIGHT_MOTOR_DIRECTION_PIN, OUTPUT);
	digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);

	pinMode(LEFT_MOTOR_DIRECTION_PIN, OUTPUT);
	digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);

	pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
	digitalWrite(RIGHT_MOTOR_ENABLE_PIN, LOW);

	pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
	digitalWrite(LEFT_MOTOR_ENABLE_PIN, LOW);

	delay(1000);

	Serial.print("<RobotMainBrain Active>");

}


void loop()
{
	heartBeat();
	cp.run();

}


void heartBeat(){

	static unsigned long preMil = millis();
	unsigned long curMil = millis();

	if (curMil - preMil >= 5000){
		preMil = curMil;
		Serial.print("<RMB HB>");
	}

}
