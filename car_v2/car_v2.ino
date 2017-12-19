#include <Arduino.h>
#include "motion.h"
#include "optical_cnt.h"
#include "main.h"

//#define DEBUG_LN(s) (Serial.println(s))
//#define DEBUG(s) (Serial.print(s));


#define TOTAL_BYTES 4
#define TRUE 1
#define FALSE 0

#define Freq 5

const unsigned int GAP= 1000 / Freq;

char state = 's';
const char HEADER = 'H';


unsigned int dist_in_cm = 0;
unsigned int count_in_cm = 0;



void setup()
{
	Serial.begin(9600);
	Serial1.begin(9600);
	init_motion();
	init_optc_cnt();
}

void process_msg(const char rcv_ch[3]) {
	Serial1.print(rcv_ch[0]);
	Serial1.println("_ack");

	unsigned int _2bytes = rcv_ch[1] * 256;
	_2bytes += (unsigned char)rcv_ch[2];
	const char order = rcv_ch[0];
	switch(order) {
	case 'f':
	case 'b':
		go(order);
		break;
	case 'l':
	case 'r':
		turn(order);
		break;
	case 'F':
	case 'B':
		go_dist(order, _2bytes);
		break;
	case 'I':
	case 'K':
		go_steps(order, _2bytes);
		break;
	case 'L':
	case 'R':
		step_turn(order, _2bytes);
		break;
	case 's':
		stop();
		break;
	case 'p':
		set_both_pwm(_2bytes);
		break;
	default:
		Serial1.println("No such Order");
		break;
	}
	return;
}

void check_report(){
	static unsigned long last_time = 0;
	unsigned long now = millis();
	if(now < last_time) {
		/*overflow*/
		last_time = 0;
	}
	if(now - last_time > GAP) {
		Serial1.print("cm:");
		Serial1.println(get_cm());
		Serial1.print("step:");
		Serial1.println(get_step());
		last_time = now;
	}
}

void loop()
{
  char rcv_ch[3] = { 0 };
  if (Serial1.available() >= TOTAL_BYTES) {
    char tag = Serial1.read();
    if (tag == HEADER) {
      rcv_ch[0] = Serial1.read();
      DEBUG("rcv:");
      DEBUG_LN(rcv_ch[0]);
      rcv_ch[1] = Serial1.read();
      rcv_ch[2] = Serial1.read();
      process_msg(rcv_ch);

    }
  }
  check_report();
}

