#ifndef _MOTOR_H

#define _MOTOR_H

#include <stdio.h>
#include <semaphore.h>
#include <time.h>
#include <syslog.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include "object_detection.h"


// Defining pins used Wiring Pi for Motor Driver

#define AI1	(7) //GPIO 7 --> Rpi pin 7 --> wiring pin 7
#define AI2	(4) //GPIO 4 --> Rpi pin 16 --> wiring pin 4
#define PWM_A	(5)   //GPIO 5 --> Rpi pin 18 --> wiring pin 5

#define BI1	(0) //GPIO 0 --> Rpi pin 11 --> wiring pin 0
#define BI2	(2) //GPIO 2 --> Rpi pin 13 --> wiring pin 2
#define PWM_B	(3)   //GPIO 3 --> Rpi pin 15 --> wiring pin 3



// Declaration of Functions 
void *motor_thread(void *args);
double timestamp();
void setupmotor();
void forwardmotor();
void stopmotor();



#endif
