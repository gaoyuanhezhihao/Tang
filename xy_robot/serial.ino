/*
 * serial.c
 *
 *  Created on: May 4, 2017
 *      Author: root
 */
#define TOTAL_BYTES 4
const char HEADER = 'H';

void debug_print(const char * msg) {
	Serial.print(msg);
}

void debug_println(const char * msg) {
	Serial.println(msg);
}

void init_serial() {
	Serial.begin(9600);
	Serial1.begin(9600);
}

void process_msg(char rcv_ch[3]) {
	unsigned int time = 0;
	time = rcv_ch[1] * 256;
	time += (unsigned char)rcv_ch[2];

	switch(rcv_ch[0]) {
	case 'l':
		move_x('l', time);
		break;
	case 'r':
		move_x('r', time);
		break;
	case 'u':
		move_y('u', time);
		break;
	case 'd':
		move_y('d', time);
		break;
	default:
		Serial1.println("Error! wrong order");
		break;
	}

}

void check_serial() {
	  char rcv_ch[3] = { 0 };
	  if (Serial1.available() >= TOTAL_BYTES) {
	    char tag = Serial1.read();
	    if (tag == HEADER) {
	      rcv_ch[0] = Serial1.read();
	      rcv_ch[1] = Serial1.read();
	      rcv_ch[2] = Serial1.read();
	      process_msg(rcv_ch);
	    }else {
	    	Serial1.println("Header fail");
	    }
	  }
}
