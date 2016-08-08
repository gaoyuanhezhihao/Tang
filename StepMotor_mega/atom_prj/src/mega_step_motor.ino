#define DIR_L 8
#define ENA_L 9
#define PWM_L 5

#define DIR_R 10
#define ENA_R 11
#define PWM_R 6
#define TOTAL_BYTES 4
#define BEST_PWM 2200
#define SYSTEM_CLOCK 16000000

#define COUNTER_PORT 3
#define STEPS_PER_CIRCLE 1000

#define TRUE 1
#define FALSE 0

enum dist_go_states {
  NOT_DIST_GO,
  WAIT_D_ORDER,
  DIST_MOVING,
  DIST_GO_PAUSE
};

enum SYSTEM_MODE{
  NORMAL_MODE,
  STEP_TURN_MODE,
  DIST_GO_MODE
};

SYSTEM_MODE current_system_mode = NORMAL_MODE;

char state = 's';
char last_state = 's';
const char HEADER = 'H';
char stoped = 1;
unsigned int step_count = 0;
unsigned int steps_needed = 0;
char step_turn_mode = 'N';

unsigned int std_length = 0;
unsigned int dist_in_std_lenght = 0;
unsigned int count_std_length = 0;
dist_go_states current_dist_go_states = NOT_DIST_GO;

void setup()
{
  Serial1.begin(9600);
  Serial1.println();
  pinMode(DIR_L, OUTPUT);
  pinMode(ENA_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(ENA_R, OUTPUT);

  pinMode(COUNTER_PORT, INPUT);
  attachInterrupt(0, port0_interrupt_handler, CHANGE);

  TCCR3A = TCCR3B = 0;
  TCCR3A = _BV(COM3A1) | _BV(WGM31); //non-inverting
  TCCR3B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
  ICR3 = SYSTEM_CLOCK / BEST_PWM;
  //ICR3 = 1990;
  OCR3A = SYSTEM_CLOCK / BEST_PWM/2;

  pinMode(PWM_L, OUTPUT);

  TCCR4A = TCCR4B = 0;
  TCCR4A = _BV(COM3A1) | _BV(WGM31); //non-inverting
  TCCR4B = _BV(CS30) | _BV(WGM32) | _BV(WGM33); //prescalar=1, fast pwm
  ICR4 = SYSTEM_CLOCK / BEST_PWM;
  OCR4A = SYSTEM_CLOCK / BEST_PWM/2;
  pinMode(PWM_R, OUTPUT);
}
void start_step_turn(char *rcv_ch){
  switch(rcv_ch[0]) {
    case 'y':// turing left some steps.
      step_turn_mode = 'y';
      // change_wheel_direction('s');
      stop_car();
      step_count = 0;
      steps_needed = rcv_ch[1] * 256;
      steps_needed += (unsigned char)rcv_ch[2];
      Serial1.println("y_ack");
      Serial1.println("try to step left ");
      Serial1.println(steps_needed);
      // change_wheel_direction('l');
      turn_left();
      break;

    case 'z':// step right.
      step_turn_mode = 'z';
      // change_wheel_direction('s');
      stop_car();
      step_count = 0;
      steps_needed = rcv_ch[1] * 256;
      steps_needed += (unsigned char)rcv_ch[2];
      Serial1.println("z_ack");
      Serial1.println("try to step right ");
      Serial1.println(steps_needed);
      // change_wheel_direction('r');
      turn_right();
      break;
    default:
      break;
  }
}
void port0_interrupt_handler()
{
//  if(digitalRead(2) == 0) {
//      ++step_count;
//  }
  if(current_system_mode == STEP_TURN_MODE) {
      ++step_count;
      if(step_count >= steps_needed) {
          stop_car();
          Serial1.print(step_turn_mode);
          Serial1.println("_ok");
          current_system_mode = NORMAL_MODE;
      }
  }else if(current_system_mode == DIST_GO_MODE && current_dist_go_states == DIST_MOVING) {
      ++step_count;
      if(count_std_length >= dist_in_std_lenght) {
          stop_car();
//          current_system_mode = NORMAL_MODE;
          current_dist_go_states = WAIT_D_ORDER;
          Serial1.println("D_ok");
      }
      if(step_count >= std_length) {
          ++ count_std_length;
          step_count -= std_length;
      }
  }
}
void change_pwm(char *rcv_ch, unsigned int *p_pwm,char channel)
{
  *p_pwm = rcv_ch[1] * 256;
  *p_pwm += (unsigned char)rcv_ch[2];
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
void change_state(char order)
{
  change_wheel_direction(order);
}
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
    state = 'f';
    Serial1.println("try to go forward");
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 1);
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
    Serial1.println("try to stop");
    stoped = 1;
}

void turn_left() {
    state = 'l';
    Serial1.println("try to turn left");
    digitalWrite(DIR_L, 0);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void turn_right() {
    state = 'r';
    Serial1.println("try to turn right");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 1);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}

void go_backward() {
    state = 'b';
    Serial1.println("try to run back");
    digitalWrite(DIR_L, 1);
    digitalWrite(DIR_R, 0);
    digitalWrite(ENA_L, 1);
    digitalWrite(ENA_R, 1);
    start_car();
}
void change_wheel_direction(char order)
{
  switch (order)
  {
  case 's':
    stop_car();
    Serial1.println("s_ack");
    break;
  case 'f':
    go_forward();
    Serial1.println("f_ack");
    break;
  case 'l':
    turn_left();
    Serial1.println("l_ack");
    break;
  case 'r':
    turn_right();
    Serial1.println("r_ack");
    break;
  case 'b':
    Serial1.println("b_ack");
    go_backward();
    break;
  default:
    break;
  }
}
void process_msg(char rcv_ch[3])
{
  int msg_processed = 0;
  while(0 == msg_processed) {
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

      default:
      break;
    }
  }
}


int process_normal(char rcv_ch[3]) {
  static unsigned int pwm = 0;

  switch (rcv_ch[0])
  {
  case 'p':
    change_pwm(rcv_ch, &pwm,  'l');
    change_pwm(rcv_ch, &pwm, 'r');
    Serial1.print("rcv:pwm=");
    Serial1.println(pwm);
    Serial1.println("p_ack");
    break;
  case 'y':// step left.
  case 'z':// step right.
    //start_step_turn(rcv_ch);
    stop_car();
    current_system_mode = STEP_TURN_MODE;
    return FALSE;
    break;
  case 'd':// set standard length(dist go mode)
    stop_car();
    current_system_mode = DIST_GO_MODE;
    return FALSE;
    break;
  default:
    change_state(rcv_ch[0]);
    break;
  }
  return TRUE;
}

int process_step_turn(char rcv_ch[3]) {
  if('y' == rcv_ch[0] || 'z' == rcv_ch[0]) {
    start_step_turn(rcv_ch);
    return TRUE;
  }else {
    current_system_mode = NORMAL_MODE;
    exit_step_turn();
    return FALSE;
  }
}

void exit_step_turn() {
  stop_car();
}

int process_dist_go(char rcv_ch[3]) {
  unsigned int _2bytes = rcv_ch[1] * 256;
  _2bytes += (unsigned char)rcv_ch[2];
  switch(current_dist_go_states) {
    case NOT_DIST_GO:
      if('d' == rcv_ch[0]) {
          if(0 != _2bytes) {
            std_length = _2bytes;
            Serial1.println("d_ack");
            current_dist_go_states = WAIT_D_ORDER;
            stop_car();
          }else {
            exit_dist_go("zero data");
            return FALSE;
          }
      } else {
          exit_dist_go("wrong order, NOT_DIST_GO");
          return FALSE;
      }
      break;
    case WAIT_D_ORDER:
      return process_dist_go_waiting(rcv_ch, _2bytes);
      break;
    case DIST_MOVING:
      if('w' == rcv_ch[0]) {
        stop_car();
        Serial1.println("w_ack");
        current_dist_go_states = DIST_GO_PAUSE;
      } else {
        exit_dist_go("wrong order, DIST_MOVING");
        Serial1.println(rcv_ch[0]);
        return FALSE;
      }
      break;
    case DIST_GO_PAUSE:
      if('g' == rcv_ch[0]) {
        go_forward();
        Serial1.println("g_ack");
        current_dist_go_states = DIST_MOVING;
      } else {
        exit_dist_go("wrong order, DIST_GO_PAUSE");
        return FALSE;
      }
      break;
  }
  return TRUE;
}

int process_dist_go_waiting(char rcv_ch[3], unsigned int _2bytes) {
      if('D' == rcv_ch[0]) {
          if(0 != _2bytes) {
            dist_in_std_lenght = _2bytes;
            Serial1.println("D_ack");
            step_count = 0;
            count_std_length = 0;
            current_dist_go_states = DIST_MOVING;
            go_forward();
          } else {
            exit_dist_go("zero data");
            return FALSE;
          }
      } else if('B' == rcv_ch[0]) {
          // go backwards.
          if(0 != _2bytes) {
            dist_in_std_lenght = _2bytes;
            Serial1.println("B_ack");
            step_count = 0;
            count_std_length = 0;
            current_dist_go_states = DIST_MOVING;
            go_backward();
          } else {
            exit_dist_go("zero data");
            return FALSE;
          }
      } else if('d' == rcv_ch[0]) {
          if(0 != _2bytes) {
            std_length = _2bytes;
            Serial1.println("d_ack");
            current_dist_go_states = WAIT_D_ORDER;
            stop_car();
          }else {
            exit_dist_go("zero data");
            return FALSE;
          }
      }
      else {
        exit_dist_go("wrong order, WAIT_D_ORDER");
        Serial1.println(rcv_ch[0]);
        return FALSE;
      }
      return TRUE;
}

void exit_dist_go(char * err_msg) {
    stop_car();
    current_system_mode = NORMAL_MODE;
    current_dist_go_states = NOT_DIST_GO;
    Serial1.println("exit_dist_go");
    Serial1.println(current_dist_go_states);
    Serial1.println("-->");
    Serial1.println(err_msg);
}

void loop()
{
  //Serial1.println("hello");
  //Serial.println("world");
  //Serial1.println("hello");
  char rcv_ch[3] = { 0 };
  if (Serial1.available() >= TOTAL_BYTES)
  {
    char tag = Serial1.read();
    if (tag == HEADER)
    {
      rcv_ch[0] = Serial1.read();
      rcv_ch[1] = Serial1.read();
      rcv_ch[2] = Serial1.read();
      process_msg(rcv_ch);
    }
  }
  // check_step_counter();
}
// void check_step_counter(){
//
//   if(step_turn_mode != 'N') {
//     // we are in step turning mode.
//     if(step_count >= steps_needed) {
//       change_wheel_direction('s');
//       step_turn_mode = 'N';
//     }
//   }
// }
