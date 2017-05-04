/*
 * control.h
 *
 *  Created on: May 2, 2017
 *      Author: root
 */
#include "Arduino.h"
#include "control.h"


#define DIR_Y 8
#define ENA_Y 9
#define PWM_Y 5

#define DIR_X 10
#define ENA_X 11
#define PWM_X 6

void init_control(unsigned int sys_clk, unsigned int pwm) {
	pinMode(DIR_Y, OUTPUT);
	pinMode(ENA_Y, OUTPUT);
	pinMode(DIR_X, OUTPUT);
	pinMode(ENA_X, OUTPUT);

	TCCR3A = TCCR3B = 0;
	TCCR3A = _BV(COM3A1) | _BV(WGM31); //non-inverting
	TCCR3B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
	ICR3 = sys_clk / pwm;
	//ICR3 = 1990;
	OCR3A = sys_clk / pwm/2;

	pinMode(PWM_Y, OUTPUT);

	TCCR4A = TCCR4B = 0;
	TCCR4A = _BV(COM3A1) | _BV(WGM31); //non-inverting
	TCCR4B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
	ICR4 = sys_clk / pwm;
	OCR4A = sys_clk / pwm/2;
	pinMode(PWM_X, OUTPUT);
}

void stop_car() {
    digitalWrite(ENA_X, 0);
    digitalWrite(ENA_Y, 0);
    Serial.println("stop");
}

void up_y() {
    Serial.println("up_y");
    digitalWrite(DIR_Y, 1);
    digitalWrite(ENA_Y, 1);
}

void down_y(){
    Serial.println("down_y");
    digitalWrite(DIR_Y, 0);
    digitalWrite(ENA_Y, 1);
}

void stop_y(){
    Serial.println("stop_y");
    digitalWrite(ENA_Y, 0);
}

void left_x(){
    Serial.println("left_x");
    digitalWrite(DIR_X, 1);
    digitalWrite(ENA_X, 1);
}

void right_x(){
	Serial.println("right_x");
    digitalWrite(DIR_X, 0);
    digitalWrite(ENA_X, 1);
}

void stop_x() {
	Serial.println("stop_x");
    digitalWrite(ENA_X, 0);

}
void move_y(const char dir, const int time) {
	if('u' == dir) {
		up_y();
	}else {
		down_y();
	}

	delay(time);
	stop_y();
}

void move_x(const char dir, const int time) {
	if('l' == dir) {
		left_x();
	}else {
		right_x();
	}

	delay(time);
	stop_x();
}

