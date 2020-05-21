/****************************************************************
* Author : Aaksha Jaywant & Atharv Desai                        *
* RTES Project : Autonomous Car Braking System                  *
* Reference: Sam Siewert Code seqgen.c, Atharv Desai seqgen.c   *
****************************************************************/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <syslog.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include "motor.h"
#include "object_detection.h"

#define NSEC_PER_SEC (1000000000)
#define USEC_PER_MSEC ( 1000 )
#define NANOSEC_PER_SEC ( 1000000000 )
#define NUM_CPU_CORES ( 1 )

#define NUM_THREADS ( 3 )			//*****
#define OK (1)



sem_t sem_motor, sem_obj;    // declare seamphores for 2 services
typedef struct
{
   int threadIdx;
   unsigned long long sequencePeriods;
} threadParams_t;

// Declaration of variables 

struct timeval current_time_val, start_time_val;
int i, rc, scope;
cpu_set_t threadcpu;
pthread_t threads[ NUM_THREADS ];
threadParams_t threadParams[ NUM_THREADS ];
pthread_attr_t rt_sched_attr[ NUM_THREADS ];
int rt_max_prio, rt_min_prio;
struct sched_param rt_param[ NUM_THREADS ];
struct sched_param main_param;
pthread_attr_t main_attr;
pid_t mainpid;
cpu_set_t allcpuset;			


void *Sequencer(void* args);      // sequencer declaration



int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }

  return(1);
}

void print_scheduler(void)    // to print the schedular policy under use
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_OTHER\n");
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}


int main(void)
{
	int a=0;
	gettimeofday( &start_time_val, (struct timezone *)0 );
   	gettimeofday( &current_time_val, (struct timezone *)0 );
   	syslog( LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)( current_time_val.tv_sec - start_time_val.tv_sec ), (int)current_time_val.tv_usec / USEC_PER_MSEC );
  setupmotor();


// Semaphore Initialization
	if ( sem_init( &sem_motor, 0, 0 ) )
   	{
     		 printf( "Failed to initialize motor service semaphore\n" );
     		 exit( -1 );
   	}
   	if ( sem_init( &sem_obj, 0, 0 ) )
   	{
      		printf( "Failed to initialize object detection task semaphore\n" );
      		exit( -1 );
   	}
    
	CPU_ZERO( &allcpuset );

   for ( a = 0; a < NUM_CPU_CORES; a++ )         //To allocate CPU cores
      CPU_SET( a, &allcpuset );

	mainpid = getpid();
	
	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
   	rt_min_prio = sched_get_priority_min(SCHED_FIFO);
	rc = sched_getparam(mainpid,&main_param);
   	main_param.sched_priority = rt_max_prio;
   	rc = sched_setscheduler(getpid(),SCHED_FIFO,&main_param);
   	if ( rc < 0 )
      		perror( "main_param" );
   	print_scheduler();
	pthread_attr_getscope( &main_attr, &scope );

   	if ( scope == PTHREAD_SCOPE_SYSTEM )
      		printf( "PTHREAD SCOPE SYSTEM\n" );
   	else if ( scope == PTHREAD_SCOPE_PROCESS )
      		printf( "PTHREAD SCOPE PROCESS\n" );
   	else
      		printf( "PTHREAD SCOPE UNKNOWN\n" );

   	printf( "rt_max_prio=%d\n", rt_max_prio );
   	printf( "rt_min_prio=%d\n", rt_min_prio );

// Setting attributes for the threads
	for (i=0;i<NUM_THREADS;i++)
   	{

      		rc = pthread_attr_init(&rt_sched_attr[i]);
      		rc = pthread_attr_setinheritsched( &rt_sched_attr[ i ], PTHREAD_EXPLICIT_SCHED );
      		rc = pthread_attr_setschedpolicy( &rt_sched_attr[ i ], SCHED_FIFO );
      		rt_param[i].sched_priority = rt_max_prio - i;
      		pthread_attr_setschedparam( &rt_sched_attr[i], &rt_param[i] );
      		threadParams[i].threadIdx = i;
   	}
	
// Motor thread creation @ 40 Hz
   	rt_param[ 1 ].sched_priority = rt_max_prio - 1;
   	pthread_attr_setschedparam( &rt_sched_attr[ 1 ], &rt_param[ 1 ] );
   	rc = pthread_create( &threads[ 1 ],        // pointer to thread descriptor
                        &rt_sched_attr[ 1 ],  // use specific attributes
                        //(void *)0,               // default attributes
                        motor_thread,                      // thread function entry point
                        (void *)&( threadParams[ 1 ] )  // parameters to pass in
   	);
   	if ( rc < 0 )
      		perror( "pthread_create for service 1" );
   	else
   	{
      		printf( "pthread_create successful for service 1\n" );
      		printf( "Checkpoint 4: Time: %.4f \n", timestamp() );
   	}

// Camera Object Detection Task creation @ 4 Hz

   	rt_param[ 2 ].sched_priority = rt_max_prio - 2;
   	pthread_attr_setschedparam( &rt_sched_attr[ 2 ], &rt_param[ 2 ] );
   	rc = pthread_create( &threads[ 2 ], &rt_sched_attr[ 2 ], objdetect_thread, (void *)&( threadParams[ 2 ] ) );
   	if ( rc < 0 )
      		perror( "pthread_create for service 2" );
   	else
   	{
      		printf( "pthread_create successful for service 2\n" );
      		printf( "Checkpoint 5: Time: %.4f \n", timestamp() );
   	}
    
// Sequencer Frequency @ 80  Hz
   	printf( "Start sequencer\n" );
   	threadParams[ 0 ].sequencePeriods = 900;

   	rt_param[ 0 ].sched_priority = rt_max_prio;
   	pthread_attr_setschedparam( &rt_sched_attr[ 0 ], &rt_param[ 0 ] );
   	rc = pthread_create( &threads[ 0 ], &rt_sched_attr[ 0 ], Sequencer, (void *)&( threadParams[ 0 ] ) );
   	if ( rc < 0 )
      		perror( "pthread_create for sequencer service 0" );
   	else
   	{
      		printf( "pthread_create successful for sequeencer service 0\n" );
      		printf( "Checkpoint 6: Time: %.4f \n", timestamp() );
   	}
	for(i=0;i<NUM_THREADS;i++)
    	{
		pthread_join(threads[i], NULL);
    	}



}

void *Sequencer(void* args)
{

   struct timeval current_time_val;
   struct timespec delay_time = {0, 12500000};  // delay for 12.5 msec, 80 Hz
   struct timespec remaining_time;
   double current_time;
   double residual;
   int rc, delay_cnt = 0;
   unsigned long long seqCnt    = 0;
   threadParams_t *threadParams = (threadParams_t *)args;

   gettimeofday( &current_time_val, (struct timezone *)0 );
   syslog( LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)( current_time_val.tv_sec - start_time_val.tv_sec ), (int)current_time_val.tv_usec / USEC_PER_MSEC );
   printf( "Sequencer thread @ sec=%d, msec=%d\n", (int)( current_time_val.tv_sec - start_time_val.tv_sec ), (int)current_time_val.tv_usec / USEC_PER_MSEC );
   
   do
   {
      delay_cnt = 0;
      residual  = 0.0;

      gettimeofday(&current_time_val, (struct timezone *)0);
      syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
      do
      {
         rc = nanosleep( &delay_time, &remaining_time );

         if ( rc == EINTR )
         {
            residual = remaining_time.tv_sec + ( (double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC );

            if ( residual > 0.0 )
               printf( "residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec );

            delay_cnt++;
         }
         else if ( rc < 0 )
         {
            perror( "Sequencer nanosleep" );
            exit( -1 );
         }

      } while ( ( residual > 0.0 ) && ( delay_cnt < 100 ) );

      seqCnt++;
      gettimeofday( &current_time_val, (struct timezone *)0 );
      syslog( LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)( current_time_val.tv_sec - start_time_val.tv_sec ), (int)current_time_val.tv_usec / USEC_PER_MSEC );


      // Release each service at a sub-rate of the generic sequencer rate
      // Posting of semaphores to both the tasks based on their frequency
      // Service_1 = RT_MAX-1 @ 40 Hz
      if ( ( seqCnt % 2 ) == 0 )
         sem_post( &sem_motor );

      // Service_2 = RT_MAX-2 @ 4 Hz
      if ( ( seqCnt % 20 ) == 0 )
         sem_post( &sem_obj );

      gettimeofday(&current_time_val, (struct timezone *)0);
      syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

   } while (1);

   pthread_exit( (void *)0 );

}



	
