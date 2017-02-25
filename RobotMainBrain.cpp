#include "RobotMainBrain.h"

extern CommandParser cp;

void setup()
{

	Serial.begin(19200);



}


void loop()
{

	cp.run();

}
