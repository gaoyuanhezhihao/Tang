#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-12-19 22:26:51

#include "Arduino.h"
#include <Arduino.h>
#include "motion.h"
#include "optical_cnt.h"
#include "main.h"
#include <Arduino.h>
#include "main.h"
#include "optical_cnt.h"
#include <Arduino.h>
#include "main.h"
void setup() ;
void process_msg(const char rcv_ch[3]) ;
void check_report();
void loop() ;
void init_motion();
void brake(int mil_seconds) ;
void start_car();
void _forward() ;
void stop() ;
void _left() ;
void _right() ;
void _backward() ;
void set_both_pwm(unsigned int pwm) ;
void turn(const char side) ;
void step_turn(const char side, const unsigned int steps);
void go(const char dir) ;
void go_dist(const char dir, unsigned int dist_in_cm);
void go_steps(const char dir, unsigned int steps) ;
unsigned int get_cm() ;
unsigned int get_step() ;
void _disable_itrp();
void init_optc_cnt();
void set_itrp_handler(void (*new_itrp)());
void cnt_step_itrp();
void cnt_down_step_itrp();
void cnt_cm_itrp();
void cnt_down_cm_itrp();
void clear_cnt();
void count_steps() ;
void count_down_steps(unsigned int steps);
void count_cm() ;
void count_down_cm(unsigned int dist_in_cm);

#include "car_v2.ino"

#include "motion.ino"
#include "optical_cnt.ino"

#endif
