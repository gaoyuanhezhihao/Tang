/*
 * optical_cnt.h
 *
 *  Created on: Dec 14, 2017
 *      Author: hzh
 */

#ifndef OPTICAL_CNT_H_
#define OPTICAL_CNT_H_

void count_down_steps(unsigned int steps);
void count_down_cm(unsigned int dist_in_cm);
void count_cm();
void count_steps();
void init_optc_cnt();
unsigned int get_step();
unsigned int get_cm();
#endif /* OPTICAL_CNT_H_ */
