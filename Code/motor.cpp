#include "motor.h"


extern sem_t sem_motor;

extern uint8_t motor_STOP_flag;

void setupmotor()
{
	printf("SETTING UP THE PINS FOR MOTOR DRIVER \n\r");
	wiringPiSetup();
	// Defining 3 GPIO pins as output pins from Raspberry Pi for Motor 1
	pinMode(AI1, OUTPUT);
	pinMode(AI2, OUTPUT);
    pinMode(PWM_A, OUTPUT);

    // Using SoftPWM library to configure GPIO 5 pin as PWM pin
	softPwmCreate(PWM_A, 0, 100);
	softPwmWrite(PWM_A, 100);
	
	// Defining 3 GPIO pins as output pins from Raspberry Pi for Motor 2
	pinMode(BI1, OUTPUT);
	pinMode(BI2, OUTPUT);
	pinMode(PWM_B, OUTPUT);

	// Using SoftPWM library to configure GPIO 3 pin as PWM pin
	softPwmCreate(PWM_B, 0, 100);
	softPwmWrite(PWM_B, 100);
	
}

void forwardmotor()
{
	printf("Motor moving ahead\n\r");
// For Motor 1 to rotate in forward direction
	softPwmWrite(PWM_A, 60);	
	digitalWrite(AI1, HIGH);
	digitalWrite(AI2, LOW);
// For Motor 2 to rotate in forward direction	
	softPwmWrite(PWM_B, 60);	
	digitalWrite(BI1, HIGH);
	digitalWrite(BI2, LOW);

}



void stopmotor()
{
	printf("#### Stop Signed Detected by Camera ####\n\r Motor Halt \n\r");
// To halt Motor 1
	softPwmWrite(PWM_A, 0);
	digitalWrite(AI1, 0);
	digitalWrite(AI2, 0);
// To halt Motor 2
	softPwmWrite(PWM_B, 0);
	digitalWrite(BI1, 0);
	digitalWrite(BI2, 0);

}


double timestamp()      // For Computing the timestamp
{
   struct timespec time = {0, 0};
   clock_gettime( CLOCK_REALTIME, &time );
   double msecs = ( (double)time.tv_sec * 1000.0 ) +
                  ( (double)( (double)time.tv_nsec / 1000000.0 ) );
   return msecs;
}



void *motor_thread(void *args)
{
	static double start = 0.0;
   	static double end   = 0.0;
   	static double wcet  = 0.0;
	while(1)
	{
		sem_wait(&sem_motor);
		start = timestamp();          // Start Timestamp to compute capacity
		syslog(LOG_DEBUG,"In the motor task");
		
		if(motor_STOP_flag = 1)
		{
			stopmotor();		// Motor stops on short time so speed regulated on Stop sign detection
			delay(1000);

		}
		else
		
		{
		forwardmotor();         //By default, motor goes ahead
		end = timestamp();      // Stop Timestamp to compute capacity
		if ( ( end - start ) > wcet )
      		{
         		wcet = end - start;  // TO compute worst case execution time
		}
		syslog(LOG_DEBUG,"Motor service WCET: %.4f \n", wcet );
	}
	}


}


