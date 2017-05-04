/*
 * control.h
 *
 *  Created on: May 2, 2017
 *      Author: root
 */

#ifndef CONTROL_H_
#define CONTROL_H_



void stop_car();
void up_y();
void down_y();
void stop_y();
void left_x();
void right_x();
void stop_x() ;
void move_y(const char dir, const int time) ;
void move_x(const char dir, const int time) ;
void init_control(unsigned int sys_clk, unsigned int pwm) ;
#endif /* CONTROL_H_ */
