/*
 Name:		PWM_Static_Generator.ino
 Created:	2016/2/20 14:27:13
 Author:	Anna Sherlock
*/

// the setup function runs once when you press reset or power the board
#include <PWM.h>
#define ENA_1 4
#define ENA_2 5
#define DIR_1 2
#define DIR_2 3

//use pin 11 on the mega for this example to work
#define PWM_1 9
#define PWM_2 10
#define TOTAL_BYTES 4
#define BEST_PWM 4800
#define START_FIRST_PWM 3200
#define START_STEPS 20
#define START_PWM_STEP (( BEST_PWM - START_FIRST_PWM)/START_STEPS)
#define START_TIME 2000 //2000ms = 2s
#define STEP_TIME (START_TIME/START_STEPS)
#define PWM_FREQ  5000


void setup()
{
	InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
	Serial.begin(9600);
	Serial.println();

	pinMode(DIR_1, OUTPUT);
	pinMode(DIR_2, OUTPUT);
	pinMode(ENA_1, OUTPUT);
	pinMode(ENA_2, OUTPUT);
	pinMode(PWM_1, OUTPUT);
	pinMode(PWM_2, OUTPUT);
	digitalWrite(DIR_1, 1);
	delay(1);
	settingHighResolutionDuty(PWM_FREQ);

	//digitalWrite(ENA_1, 0);
}
void settingHighResolutionDuty(unsigned int pwm)
{
	SetPinFrequency(PWM_1, pwm); //setting the frequency to 10 Hz
	SetPinFrequency(PWM_2, pwm);
	//Serial.println("\r\npwmWrite() and pwmWriteHR() are identical except for the valid range of inputs.\r\nThe following loop calls both functions to produce the same result on the \r\nLED pin. The pin should to run 10Hz at 50% duty regardless of the function called.\r\n");

	//the PWM_1 should flicker (10Hz 50% duty) for 1 second before calling
	//the other function. This demonstrates the use of pwmWriteHR() and how its
	//use is nearly identical to pwmWrite()

	//setting the duty to 50% with 8 bit pwm. 128 is 1/2 of 256
	//pwmWrite(PWM_1, 128);
	//pwmWrite(PWM_2, 128);
	//Serial.println("8-Bit PWM");
	//delay(1000);

	//setting the duty to 50% with the highest possible resolution that 
	//can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
	pwmWriteHR(PWM_1, 32768);
	pwmWriteHR(PWM_2, 32768);
	//Serial.println("High Resolution PWM");
	//delay(1000);

}
void loop()
{
	;
}