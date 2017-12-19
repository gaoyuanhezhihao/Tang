/*
 * motion.c
 *
 *  Created on: Dec 14, 2017
 *      Author: hzh
 */

#include <Arduino.h>
#include "main.h"
#include "optical_cnt.h"

#define DIR_L 8
#define ENA_L 9
#define PWM_L 5

#define DIR_R 10
#define ENA_R 11
#define PWM_R 6

#define SYSTEM_CLOCK 16000000
#define DEFAULT_PWM 1600


unsigned int pwm_now;
/*	function */

void init_motion(){
	  pinMode(DIR_L, OUTPUT);
	  pinMode(ENA_L, OUTPUT);
	  pinMode(DIR_R, OUTPUT);
	  pinMode(ENA_R, OUTPUT);
	  TCCR3A = TCCR3B = 0;
	  TCCR3A = _BV(COM3A1) | _BV(WGM31); //non-inverting
	  TCCR3B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
	  ICR3 = SYSTEM_CLOCK / DEFAULT_PWM;
	  //ICR3 = 1990;
	  OCR3A = SYSTEM_CLOCK / DEFAULT_PWM/2;

	  pinMode(PWM_L, OUTPUT);

	  TCCR4A = TCCR4B = 0;
	  TCCR4A = _BV(COM3A1) | _BV(WGM31); //non-inverting
	  TCCR4B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
	  ICR4 = SYSTEM_CLOCK / DEFAULT_PWM;
	  OCR4A = SYSTEM_CLOCK / DEFAULT_PWM/2;
	  pinMode(PWM_R, OUTPUT);
	  stop();
}

void brake(int mil_seconds) {
  pinMode(PWM_L, INPUT);
  pinMode(PWM_R, INPUT);
  delay(mil_seconds);
}
void start_car(){
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
}

void _forward() {
    DEBUG_LN("try to go forward");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void stop() {
    state = 's';
    digitalWrite(ENA_R, 0);
    digitalWrite(ENA_L, 0);
    DEBUG_LN("stop");
}

void _left() {
    DEBUG_LN("turn left");
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void _right() {
    DEBUG_LN("try to turn right");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 1);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void _backward() {
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 1);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
    DEBUG_LN("go backward");
}

void set_both_pwm(unsigned int pwm) {
    ICR3  = SYSTEM_CLOCK / pwm;
    OCR3A = SYSTEM_CLOCK / pwm /2;
    ICR4 = SYSTEM_CLOCK / pwm;
    OCR4A = SYSTEM_CLOCK / pwm /2;
}
void turn(const char side) {
	stop();
	state = side;
	count_steps();

	switch(side){
	case 'l':
		_left();
		break;
	case 'r':
		_right();
		break;
	default:
		DEBUG_LN("Error, turn @side is not in ['l', 'r'] ");
		break;
	}

}
void step_turn(const char side, const unsigned int steps){
	stop();
	state = side;
	count_down_steps(steps);

	switch(side){
	case 'L':
		_left();
		break;
	case 'R':
		_right();
		break;
	default:
		DEBUG_LN("Error, start_step_turn @side is not in ['x', 'y'] ");
		break;
	}

}


void go(const char dir) {
	stop();
	state = dir;
	count_cm();
	switch(dir) {
	case 'f':
		_forward();
		break;
	case 'b':
		_backward();
		break;
	default:
		DEBUG_LN("Error, go @dir is not in ['f', 'b']");
	}
}
void go_dist(const char dir, unsigned int dist_in_cm){
	  stop();
	  state = dir;
	  count_down_cm(dist_in_cm);
	  switch(dir) {
	  case 'F':
		  _forward();
		  break;
	  case 'B':
		  _backward();
		  break;
	  default:
		  DEBUG_LN("Error, go_dist @dir is not in ['F', 'B'] ");
		  break;
	  }
	  return;
}

void go_steps(const char dir, unsigned int steps) {
  stop();
  state = dir;
  count_down_steps(steps);
  switch(dir) {
  case 'I':
	  _forward();
	  break;
  case 'K':
	  _backward();
	  break;
  default:
	  DEBUG_LN("Error, go_dist @dir is not in ['I', 'K'] ");
	  break;
  }

  return;
}
//void step_start(char dir) {
//	const unsigned int STEPS = 20;
//	const unsigned int TIME_PER_STEP = 1000 /20;
//	const unsigned int step_pwm_intv = pwm_now /STEPS;
//	unsigned int pwm = 0;
//
//	_set_both_pwm(pwm);
//	start_car();
//	if('f' == dir) {
//	  _forward();
//	}else if('b' == dir){
//	  _backward();
//	}
//	while(pwm <= pwm_now - step_pwm_intv) {
//	  pwm += step_pwm_intv;
//	  delay(TIME_PER_STEP);
//	  _set_both_pwm(pwm);
//	}
//}
