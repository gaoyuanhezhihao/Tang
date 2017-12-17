/*
 * main.h
 *
 *  Created on: Dec 14, 2017
 *      Author: hzh
 */

#ifndef MAIN_H_
#define MAIN_H_


//enum SYSTEM_MODE{
//  NORMAL_MODE,
//  STEP_TURN_MODE,
//  DIST_GO_MODE,
//  MAP_MODE
//};
//extern enum SYSTEM_MODE current_system_mode;
extern char state;
#define DEBUG_LN(s) (Serial.println(s))
#define DEBUG(s) (Serial.print(s))
#endif /* MAIN_H_ */
