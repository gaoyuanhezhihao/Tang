/*
 * sensor.c
 *
 *  Created on: May 4, 2017
 *      Author: root
 */

#include "Arduino.h"
#include "sensor.h"
#include "control.h"

#define XL_PORT 2
#define XR_PORT 3
#define YU_PORT 21
#define YD_PORT 20

void init_sensor() {
	  pinMode(XL_PORT, INPUT);
	  pinMode(XR_PORT, INPUT);
	  pinMode(YU_PORT, INPUT);
	  pinMode(YD_PORT, INPUT);

	  attachInterrupt(0, xl_itrp_handler, FALLING);
	  attachInterrupt(1, xr_itrp_handler, FALLING);
	  attachInterrupt(2, yu_itrp_handler, FALLING);
	  attachInterrupt(3, yd_itrp_handler, FALLING);
}


void xl_itrp_handler() {
	Serial.println("port xl interrupt");
	stop_x();
}

void xr_itrp_handler() {
	Serial.println("port xr interrupt");
	stop_x();
}

void yu_itrp_handler() {
	Serial.println("port yu interrupt");
	stop_y();
}

void yd_itrp_handler() {
	Serial.println("port yd interrupt");
	stop_y();
}
