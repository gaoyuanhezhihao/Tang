/*
 * optical_cnt.c
 *
 *  Created on: Dec 14, 2017
 *      Author: hzh
 */
#include <Arduino.h>
#include "main.h"

#define COUNTER_PORT 3
#define PULSE_PER_CM 66
#define DIST_GO_PWM 1000



//typedef void (*Itrp_Func)(void);
unsigned int _cm_needed = 0;
unsigned int _cm_cnt = 0;
unsigned int _step_cnt = 0;
unsigned int _steps_needed = 0;




unsigned int get_cm() {
	return _cm_cnt;
}
unsigned int get_step() {
	return _step_cnt;
}
void _disable_itrp(){
	detachInterrupt(0);
}
void init_optc_cnt(){
	pinMode(COUNTER_PORT, INPUT);
    _disable_itrp();
}
void set_itrp_handler(void (*new_itrp)()){
	attachInterrupt(0, new_itrp, CHANGE);
}
//void _enable_itrp() {
//	attachInterrupt(0, port0_interrupt_handler, CHANGE);
//}


void cnt_step_itrp(){
	++ _step_cnt;
}

void cnt_down_step_itrp(){
	++ _step_cnt;
    if(_step_cnt >= _steps_needed) {
    	_disable_itrp();
        Serial1.print(state);
        Serial1.println("_ok");
        stop();
    }
}

void cnt_cm_itrp(){
	++ _step_cnt;
	if(_step_cnt >= PULSE_PER_CM) {
		++ _cm_cnt;
		_step_cnt -= PULSE_PER_CM;
	}
}

void cnt_down_cm_itrp(){
	++ _step_cnt;
	if(_step_cnt >= PULSE_PER_CM) {
		++ _cm_cnt;
		_step_cnt -= PULSE_PER_CM;
	}
    if(_cm_cnt >= _cm_needed) {
    	_disable_itrp();
        Serial1.print(state);
        Serial1.println("_ok");
    	stop();
    }
}


void clear_cnt(){
	_disable_itrp();

	_step_cnt = 0;
	_cm_cnt = 0;

}

void count_steps() {
	clear_cnt();
	set_itrp_handler(cnt_step_itrp);
}

void count_down_steps(unsigned int steps){
	clear_cnt();
	_steps_needed = steps;
	set_itrp_handler(cnt_down_step_itrp);
}

void count_cm() {
	clear_cnt();
	set_itrp_handler(cnt_cm_itrp);
}
void count_down_cm(unsigned int dist_in_cm){
	clear_cnt();
	set_itrp_handler(cnt_down_cm_itrp);
	_cm_needed = dist_in_cm;
}
