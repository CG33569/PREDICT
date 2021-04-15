#include "PREDICTdummy.h"


void receiver_thread()
{
  printf("receiver check!\n");

  while(!program_off)
  {
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    transmitter_read();  
    printf("channel1: %d, channel2: %d, channel3: %d, channel4: %d,channel5: %d,channel6: %d\n",channel_1,channel_2,channel_3,channel_4,channel_5,channel_6 );

     //For starting the motors: throttle low and yaw left (step 1).
    if(channel_3 < 1050 && channel_4 < 1050)
    {
      start = 1;
      printf("motors arming\n");
    }
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && channel_3 < 1050 && channel_4 > 1450){
      start = 2;
      printf("motors start\n");
    }
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && channel_3 < 1050 && channel_4 > 1950)
    {
      start = 0;
      printf("motors stop\n");
    }


    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      program_off = 1;
      printf("code stop\n");
    }
    

  }
}


int main(int argc, char **argv)
{
  int cpucnt = 4;
  int th_id, retval;
  if(PRINT_COMMANDS)printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);

  /////////////////multi core thread initiate
  pthread_mutex_lock(&mutex);
  th_id=1;
  retval = pthread_create(threads+th_id, NULL, dummy_thread1, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
 
  th_id=2;
  retval = pthread_create(threads+th_id, NULL, receiver_thread, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }

  th_id=3;
  retval = pthread_create(threads+th_id, NULL, dummy_thread3, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
  pthread_mutex_unlock(&mutex);
////////////////////////////

  printf("thread initaited\n");

  millis(1000);

  printf("main loop start\n");

  dummy_thread2();
////////////////////////////

  printf("Waiting for threads to join\n");
  millis(1000);
  void * dummy;
  
  th_id=1;
  
  printf("Waiting for threard nr.%d... ", th_id);
  millis(1000);
  retval = pthread_join(*(threads+th_id), &dummy);
  if(retval != 0)
  {
    printf("Unable to join thread %d, error code %d\n", th_id, retval);
  }  

  th_id=2;
  printf("Waiting for threard nr.%d... ", th_id);
  millis(1000);
  retval = pthread_join(*(threads+th_id), &dummy);
  if(retval != 0)
  {
    printf("Unable to join thread %d, error code %d\n", th_id, retval);
  } 

  th_id=3;
  printf("Waiting for threard nr.%d... ", th_id);
  millis(1000);
  retval = pthread_join(*(threads+th_id), &dummy);
  if(retval != 0)
  {
    printf("Unable to join thread %d, error code %d\n", th_id, retval);
  } 

  free(threads);

  return 0;
}