#include "RobotMainBrain.h"

extern CommandParser cp;

unsigned int heartbeatInterval = 500;

void setup() {

	pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
	digitalWrite(RIGHT_MOTOR_ENABLE_PIN, LOW);

	pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
	digitalWrite(LEFT_MOTOR_ENABLE_PIN, LOW);

	pinMode(RIGHT_MOTOR_DIRECTION_PIN, OUTPUT);
	digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);

	pinMode(LEFT_MOTOR_DIRECTION_PIN, OUTPUT);
	digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);



	pinMode(HEARTBEAT_PIN, OUTPUT);
	for (int i = 0; i < 3; i++) {
		digitalWrite(HEARTBEAT_PIN, HIGH);
		delay(100);
		digitalWrite(HEARTBEAT_PIN, LOW);
		delay(100);
	}

	heartbeatInterval = 200;
	unsigned long delayStart = millis();
	while(millis() - delayStart <= 5000){
		heartBeat();
	}

	Serial.begin(115200);

	heartbeatInterval = 500;

	delay(1000);

	Serial.print("<RobotMainBrain Active>");

}

void loop() {
	heartBeat();
	cp.run();

}

void heartBeat() {
	static boolean heartState = false;
	static uint8_t counter = 0;

	static unsigned long preMil = millis();
	unsigned long curMil = millis();

	if (curMil - preMil >= heartbeatInterval) {
		preMil = curMil;
		heartState = !heartState;
		digitalWrite(HEARTBEAT_PIN, heartState);
		counter++;
		if(counter == (5000 / heartbeatInterval)){
			Serial.print("<RMB HB>");
			counter = 0;
		}

	}

}
