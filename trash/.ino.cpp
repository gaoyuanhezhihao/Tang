#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-01-01 19:12:26

#include "Arduino.h"
#include <Arduino.h>
void setup() ;
void enable_itrp() ;
void disable_itrp();
void start_step_turn(char *rcv_ch);
void port0_interrupt_handler() ;
void change_pwm(char *rcv_ch, unsigned int *p_pwm,char channel) ;
void set_both_pwm(unsigned int pwm) ;
void brake(int mil_seconds) ;
void start_car();
void go_forward() ;
void stop_car() ;
void turn_left() ;
void turn_right() ;
void go_backward() ;
void process_msg(char rcv_ch[3]) ;
int change_mode(char order) ;
int process_normal(char rcv_ch[3]) ;
int process_map(char rcv_ch[3]) ;
int process_step_turn(char rcv_ch[3]) ;
int process_dist_go(char rcv_ch[3]) ;
void step_start(char dir) ;
void go_dist(char * rcv_ch);
void filter_msg(char *msg) ;
void loop() ;

#include "four_wheels.ino"


#endif
