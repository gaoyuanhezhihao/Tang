#include "Arduino.h"
#include "control.h"
#include "sensor.h"
#include "serial.h"


#define SYSTEM_CLOCK 16000000
#define DEFAULT_PWM 800



//enum X_Side{
//	Left,
//	Right,
//};

void setup() {

	init_serial();
	init_control(SYSTEM_CLOCK, DEFAULT_PWM);
	init_sensor();

	stop_car();
	Serial.println("xy_robot");
}




void loop() {
	check_serial();
//	Serial1.println("serial1...");
//	delay(1000);
//	move_y('u', 2000);
//	move_y('d', 2000);
//	move_x('l', 1000);
//	move_x('r', 1000);
}
