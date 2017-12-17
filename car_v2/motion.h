/*
 * motion.h
 *
 *  Created on: Dec 14, 2017
 *      Author: hzh
 */

#ifndef MOTION_H_
#define MOTION_H_


void init_motion();
void go_steps(const char dir, unsigned int steps);
void go_dist(const char dir, unsigned int dist_in_cm);
void go(const char dir);
void step_turn(const char side, const unsigned int steps);
void turn(const char side);
void stop();
void set_both_pwm(unsigned int pwm);


#endif /* MOTION_H_ */
