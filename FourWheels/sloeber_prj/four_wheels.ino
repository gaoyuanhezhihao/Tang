#include <Arduino.h>
#define DIR_L 8
#define ENA_L 9
#define PWM_L 5

#define DIR_R 10
#define ENA_R 11
#define PWM_R 6
#define TOTAL_BYTES 4
#define DEFAULT_PWM 2200
#define TURN_PWM 500
#define MAP_GO_PWM 1000
#define SYSTEM_CLOCK 16000000

#define COUNTER_PORT 3
#define STEPS_PER_CIRCLE 1000

#define TRUE 1
#define FALSE 0

#define SPECAIL_NUM 1
#define PULSE_PER_CM 66
#define DIST_GO_PWM 1000

//enum dist_go_states {
//  NOT_DIST_GO,
//  WAIT_D_ORDER,
//  DIST_MOVING,
//  DIST_GO_PAUSE
//};

enum SYSTEM_MODE{
  NORMAL_MODE,
  STEP_TURN_MODE,
  DIST_GO_MODE,
  MAP_MODE
};

SYSTEM_MODE current_system_mode = NORMAL_MODE;

unsigned int pwm_now = DEFAULT_PWM;
char state = 's';
char last_state = 's';
const char HEADER = 'H';
char stoped = 1;
unsigned int step_count = 0;
unsigned int steps_needed = 0;
char step_turn_mode = 'N';

unsigned int dist_in_cm = 0;
unsigned int count_in_cm = 0;

/*---map mode---*/
unsigned long map_dist_pulses = 0;

void setup()
{
	Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(DIR_L, OUTPUT);
  pinMode(ENA_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(ENA_R, OUTPUT);

  pinMode(COUNTER_PORT, INPUT);
  attachInterrupt(0, port0_interrupt_handler, CHANGE);

  TCCR3A = TCCR3B = 0;
  TCCR3A = _BV(COM3A1) | _BV(WGM31); //non-inverting
  TCCR3B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
  ICR3 = SYSTEM_CLOCK / DEFAULT_PWM;
  //ICR3 = 1990;
  OCR3A = SYSTEM_CLOCK / DEFAULT_PWM/2;

  pinMode(PWM_L, OUTPUT);

  TCCR4A = TCCR4B = 0;
  TCCR4A = _BV(COM3A1) | _BV(WGM31); //non-inverting
  TCCR4B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
  ICR4 = SYSTEM_CLOCK / DEFAULT_PWM;
  OCR4A = SYSTEM_CLOCK / DEFAULT_PWM/2;
  pinMode(PWM_R, OUTPUT);
}
void enable_itrp() {
	attachInterrupt(0, port0_interrupt_handler, CHANGE);
}
void disable_itrp(){
	detachInterrupt(0);
}
void start_step_turn(char *rcv_ch){
  switch(rcv_ch[0]) {
    case 'l':// turing left some steps.
      step_turn_mode = 'l';
      stop_car();
      step_count = 0;
      steps_needed = rcv_ch[1] * 256;
      steps_needed += (unsigned char)rcv_ch[2];
      Serial1.println("l_ack");
      Serial1.println(steps_needed);
      state = 'l';
      enable_itrp();
      turn_left();
      break;
    case 'r':// step right.
      step_turn_mode = 'r';
      stop_car();
      step_count = 0;
      steps_needed = rcv_ch[1] * 256;
      steps_needed += (unsigned char)rcv_ch[2];
      Serial1.println("r_ack");
      Serial1.println(steps_needed);
      state =  'r';
      enable_itrp();
      turn_right();
      break;
    default:
      break;
  }
}
void port0_interrupt_handler()
{
  if(current_system_mode == STEP_TURN_MODE) {
      ++step_count;
      if(step_count >= steps_needed) {
          disable_itrp();
          stop_car();
          Serial1.print(state);
          Serial1.println("_ok");
          current_system_mode = NORMAL_MODE;
      }
  }else if(current_system_mode == DIST_GO_MODE) {
      ++step_count;
      if(count_in_cm >= dist_in_cm) {
          stop_car();
          Serial1.println("D_ok");
      }
      if(step_count >= PULSE_PER_CM) {
          ++ count_in_cm;
          step_count -= PULSE_PER_CM;
      }
  }else if(MAP_MODE == current_system_mode) {
	  ++ map_dist_pulses;
  }
}
//void micro_move(char *rcv_ch) {
//  unsigned int time_to_move = rcv_ch[1] * 256;
//  time_to_move += (unsigned char)rcv_ch[2];
//  switch(rcv_ch[0]) {
//    case 'H':
//      turn_left();
//      delay(time_to_move);
//      stop_car();
//      Serial1.println("H_ok");
//      break;
//    case 'L':
//      turn_right();
//      delay(time_to_move);
//      stop_car();
//      Serial1.println("L_ok");
//      break;
//    case 'J':
//      go_backward();
//      delay(time_to_move);
//      stop_car();
//      Serial1.println("J_ok");
//      break;
//    case 'K':
//      go_forward();
//      delay(time_to_move);
//      stop_car();
//      Serial1.println("K_ok");
//      break;
//    default:
//      break;
//  }
//}
void change_pwm(char *rcv_ch, unsigned int *p_pwm,char channel)
{
  *p_pwm = rcv_ch[1] * 256;
  *p_pwm += (unsigned char)rcv_ch[2];
  Serial1.print("rcv:pwm=");
  Serial1.println(*p_pwm);
  pwm_now = *p_pwm;
  if (channel == 'l')
  {
    ICR3  = SYSTEM_CLOCK / *p_pwm;
    OCR3A = SYSTEM_CLOCK / *p_pwm /2;
  }
  else
  {
    ICR4 = SYSTEM_CLOCK / *p_pwm;
    OCR4A = SYSTEM_CLOCK / *p_pwm /2;
  }
}
void set_both_pwm(unsigned int pwm) {
    ICR3  = SYSTEM_CLOCK / pwm;
    OCR3A = SYSTEM_CLOCK / pwm /2;
    ICR4 = SYSTEM_CLOCK / pwm;
    OCR4A = SYSTEM_CLOCK / pwm /2;
}
// void change_state(char order)
// {
//   change_wheel_direction(order);
// }
void brake(int mil_seconds) {
  pinMode(PWM_L, INPUT);
  pinMode(PWM_R, INPUT);
  delay(mil_seconds);
}
void start_car(){
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
}

void go_forward() {
    Serial.println("try to go forward");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void stop_car() {
    last_state = state;
    state = 's';
    if(last_state == 'l' || last_state == 'r' || last_state == 'f' || last_state == 'b') {
      brake(100);
    }
    digitalWrite(ENA_R, 0);
    digitalWrite(ENA_L, 0);
    Serial.println("stop");
    stoped = 1;
}

void turn_left() {
    Serial.println("turn left");
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void turn_right() {
    Serial.println("try to turn right");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 1);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void go_backward() {
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 1);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
    Serial.println("go backward");
}
// void change_wheel_direction(char order)
// {
//   switch (order)
//   {
//   case 's':
//     stop_car();
//     Serial1.println("s_ack");
//     break;
//   case 'f':
//     go_forward();
//     Serial1.println("f_ack");
//     break;
//   case 'l':
//     turn_left();
//     Serial1.println("l_ack");
//     break;
//   case 'r':
//     turn_right();
//     Serial1.println("r_ack");
//     break;
//   case 'b':
//     Serial1.println("b_ack");
//     go_backward();
//     break;
//   default:
//     break;
//   }
// }
void process_msg(char rcv_ch[3])
{
  int msg_processed = FALSE;
  while(FALSE == msg_processed) {
    switch(current_system_mode) {
      case NORMAL_MODE:
        msg_processed = process_normal(rcv_ch);
        break;
      case STEP_TURN_MODE:
        msg_processed = process_step_turn(rcv_ch);
        break;
      case DIST_GO_MODE:
        msg_processed = process_dist_go(rcv_ch);
        break;
      case MAP_MODE:
    	  msg_processed = process_map(rcv_ch);
    	  break;
      default:
        break;
    }
    if(FALSE == msg_processed) {
      if (FALSE == change_mode(rcv_ch[0]) ) {
    	  Serial1.println("!Error: no such order");
    	  Serial.println("!Error: no such order");
    	  return;
      }
    }
  }


}

int change_mode(char order) {
	switch(order) {
	case 's':
	case 'f':
	case 'b':
	case 'p':
		current_system_mode = NORMAL_MODE;
		break;
	case 'l':
	case 'r':
		current_system_mode = STEP_TURN_MODE;
		break;
	case 'D':
	case 'B':
		current_system_mode = DIST_GO_MODE;
		break;
	case 'F':
	case 'K':
	case 'S':
		current_system_mode = MAP_MODE;
		break;
	default:
		return FALSE;
	}
	return TRUE;
}


int process_normal(char rcv_ch[3]) {
  static unsigned int pwm = 0;

  switch (rcv_ch[0])
  {
    case 'p':
      change_pwm(rcv_ch, &pwm,  'l');
      change_pwm(rcv_ch, &pwm, 'r');
      Serial1.println("p_ack");
      break;
    case 's':
      stop_car();
      Serial1.println("s_ack");
      break;
    case 'f':
    	state = 'f';
      go_forward();
      Serial1.println("f_ack");
      break;
    case 'b':
    	state = 'b';
      Serial1.println("b_ack");
      go_backward();
      break;
    default:
      return FALSE;
      break;
  }
  return TRUE;
}

int process_map(char rcv_ch[3]) {
	char order = rcv_ch[0];
	switch(order) {
	case 'F':
		Serial1.println("F_ack");
		map_dist_pulses = 0;
		current_system_mode = MAP_MODE;
		state = 'F';
		enable_itrp();
		set_both_pwm(MAP_GO_PWM);
		go_forward();
		break;
	case 'K':
		/* go backward*/
		Serial1.println("K_ack");
		map_dist_pulses = 0;
		current_system_mode = MAP_MODE;
		state = 'K';
		enable_itrp();
		set_both_pwm(MAP_GO_PWM);
		go_backward();
		break;
	case 'S':
		stop_car();
		Serial1.print('S');
		Serial1.println(map_dist_pulses / PULSE_PER_CM);
		break;
	default:
		return FALSE;
		break;
	}
	return TRUE;
}

int process_step_turn(char rcv_ch[3]) {
  if('l' == rcv_ch[0] || 'r' == rcv_ch[0]) {
	  set_both_pwm(TURN_PWM);
	  start_step_turn(rcv_ch);
	  return TRUE;
  }else {
    return FALSE;
  }
}

int process_dist_go(char rcv_ch[3]) {
  switch(rcv_ch[0]) {
  case 'D':
	  if('D' == state) {
		  break;
	  }else{
		  Serial1.println("D_ack");
		  go_dist(rcv_ch);
	  }
	  break;
  case 'B':
	  if('B' == state) {
		  break;
	  }else{
		  Serial1.println("B_ack");
		  go_dist(rcv_ch);
	  }
	  break;
  default:
	  return FALSE;
	  break;

  }
  return TRUE;
}
void step_start(char dir) {
	const unsigned int STEPS = 20;
	const unsigned int TIME_PER_STEP = 1000 /20;
	const unsigned int step_pwm_intv = pwm_now /STEPS;
	unsigned int pwm = 0;

  start_car();
  if('f' == dir) {
	  go_forward();
  }else if('b' == dir){
	  go_backward();
  }
  while(pwm <= pwm_now - step_pwm_intv) {
	  pwm += step_pwm_intv;
    delay(TIME_PER_STEP);
    set_both_pwm(pwm);
  }
}
void go_dist(char * rcv_ch){
  unsigned int _2bytes = rcv_ch[1] * 256;
  _2bytes += (unsigned char)rcv_ch[2];
  stop_car();
  current_system_mode = DIST_GO_MODE;
  step_count = 0;
  count_in_cm = 0;
  dist_in_cm = _2bytes;
  pwm_now = DIST_GO_PWM;
  enable_itrp();
  if('B' == rcv_ch[0]) {
	  step_start('b');
	  state = 'B';
  }else if('D' == rcv_ch[0]) {
	  step_start('f');
	  state = 'D';
  }
  return;
}

//int process_dist_go_waiting(char rcv_ch[3], unsigned int _2bytes) {
//      if('D' == rcv_ch[0]) {
//          if(0 != _2bytes) {
//            dist_in_std_lenght = _2bytes;
//            Serial1.println("D_ack");
//            step_count = 0;
//            count_in_cm = 0;
//            current_dist_go_states = DIST_MOVING;
//            go_forward();
//          } else {
//            exit_dist_go("zero data");
//            return FALSE;
//          }
//      } else if('B' == rcv_ch[0]) {
//          // go backwards.
//          if(0 != _2bytes) {
//            dist_in_std_lenght = _2bytes;
//            Serial1.println("B_ack");
//            step_count = 0;
//            count_in_cm = 0;
//            current_dist_go_states = DIST_MOVING;
//            go_backward();
//          } else {
//            exit_dist_go("zero data");
//            return FALSE;
//          }
//      } else if('d' == rcv_ch[0]) {
//          if(0 != _2bytes) {
//            std_length = _2bytes;
//            Serial1.println("d_ack");
//            current_dist_go_states = WAIT_D_ORDER;
//            stop_car();
//          }else {
//            exit_dist_go("zero data");
//            return FALSE;
//          }
//      }
//      else {
//        exit_dist_go("wrong order, WAIT_D_ORDER");
//        Serial1.println(rcv_ch[0]);
//        return FALSE;
//      }
//      return TRUE;
//}

// void exit_dist_go(char * err_msg) {
//     stop_car();
//     current_system_mode = NORMAL_MODE;
//     current_dist_go_states = NOT_DIST_GO;
//     Serial1.print("exit_dist_go");
//     Serial1.println(current_dist_go_states);
//     Serial1.print("-->");
//     Serial1.println(err_msg);
// }
void filter_msg(char *msg) {
//	Serial.println("filter_msg");
  char order = msg[0];
  switch (order) {
    case 'Q': /*ask state*/
    	Serial1.print("Q_ack");
    	Serial1.println(state);
    	break;
    default:
      process_msg(msg);
      break;
  }
  return;
}
void loop()
{
  char rcv_ch[3] = { 0 };
//Serial.println("hello");
  if (Serial1.available() >= TOTAL_BYTES) {
    char tag = Serial1.read();
//    Serial.print("recv:");
//    Serial.println(tag);
    if (tag == HEADER) {
      rcv_ch[0] = Serial1.read();
      Serial.print("rcv:");
      Serial.println(rcv_ch[0]);
      rcv_ch[1] = Serial1.read();
      rcv_ch[2] = Serial1.read();
      filter_msg(rcv_ch);
    }
  }
//  check_step_counter();
}
//void check_step_counter(){
//
//  if(turn_side != 'N') {
//    // we are in step turning mode.
//    if(step_count >= steps_needed) {
//      change_wheel_direction('s');
//      turn_side = 'N';
//    }
//  }
//}
