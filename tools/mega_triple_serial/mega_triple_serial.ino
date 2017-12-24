#include "Arduino.h"
//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(9600);
	Serial1.begin(9600);
	Serial2.begin(9600);
}

void loop()
{
	static int i = 0;
	++i;
//	Serial.print("Serial0");
//	Serial.println(i);

	Serial1.print("Serial1");
	Serial1.println(i);
	Serial2.print("Serial2");
	Serial2.println(i);
}
