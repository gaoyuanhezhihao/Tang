#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-05-04 20:39:06

#include "Arduino.h"
#include "Arduino.h"
#include "control.h"
#include "Arduino.h"
#include "sensor.h"
#include "control.h"
#include "Arduino.h"
#include "control.h"
#include "sensor.h"
#include "serial.h"
void init_control(unsigned long sys_clk, unsigned int pwm) ;
void stop_car() ;
void up_y() ;
void down_y();
void stop_y();
void left_x();
void right_x();
void stop_x() ;
void move_y(const char dir, const int time) ;
void move_x(const char dir, const int time) ;
void move_x_org() ;
void move_y_org() ;
void init_sensor() ;
void xl_itrp_handler() ;
void xr_itrp_handler() ;
void yu_itrp_handler() ;
void yd_itrp_handler() ;
void debug_print(const char * msg) ;
void debug_println(const char * msg) ;
void init_serial() ;
void process_msg(char rcv_ch[3]) ;
void check_serial() ;
void setup() ;
void loop() ;

#include "xy_robot.ino"

#include "control.ino"
#include "sensor.ino"
#include "serial.ino"

#endif
