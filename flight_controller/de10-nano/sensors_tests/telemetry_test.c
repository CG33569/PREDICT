#include "PREDICTdummy.h"

void telemetry_thread()
{
  const unsigned int msgL = 124;
  pthread_mutex_lock(&mutex);
  printf("-- START TELEMETRY TEST --\n");
  pthread_mutex_unlock(&mutex);
  char *xstr = "sample text 1";
  char recUART[512]="";
  for (int j=0;j<20;j++)
  {
    pthread_mutex_lock(&mutex);
    printf("\nWriting stuff nr.%d: %s\n",j,xstr);
    pthread_mutex_unlock(&mutex);
    
    send_telemtry(xstr,msgL);
    millis(100);
    
    pthread_mutex_lock(&mutex);
    receive_telemtry(recUART,msgL);
    get_telemetry_pid(recUART,msgL);
    pthread_mutex_unlock(&mutex);

    pthread_mutex_lock(&mutex);
    printf("received: %s\n",recUART);
    pthread_mutex_unlock(&mutex);
  }

  program_off = true;

  pthread_mutex_lock(&mutex);
  printf("-- END TELEMETRY TEST --\n");
  pthread_mutex_unlock(&mutex);
}


void telemetry_thread2()
{
  const unsigned int msgL = 124;
  pthread_mutex_lock(&mutex);
  printf("-- START TELEMETRY TEST --\n");
  pthread_mutex_unlock(&mutex);
  char *xstr = "sample text 1";
  char recUART[512]="";
  for (int j=0;j<20;j++)
  {
    pthread_mutex_lock(&mutex);
    printf("\nWriting stuff nr.%d: %s\n",j,xstr);
    pthread_mutex_unlock(&mutex);
    
    send_telemtry(xstr,msgL);
    millis(100);
    receive_telemtry(recUART,msgL);

    pthread_mutex_lock(&mutex);
    printf("received: %s\n",recUART);
    pthread_mutex_unlock(&mutex);
  }

  program_off = true;

  pthread_mutex_lock(&mutex);
  printf("-- END TELEMETRY TEST --\n");
  pthread_mutex_unlock(&mutex);
}


int main(int argc, char **argv)
{
  int cpucnt = 4;
  int th_id, retval;
  if(PRINT_COMMANDS)printf("Started using %d threads\n",cpucnt);
  
  printf("===========\nNote:\n===========\n");
  printf("- When the telemetry thread starts, run in a separe program the base script for the paired temeletry module\n",cpucnt);
  printf("- Run it using: python3 telemetry_base_test.py\n",cpucnt);
  printf("================================\n");

  millis(3000);
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
  retval = pthread_create(threads+th_id, NULL, dummy_thread2, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }

  th_id=3;
  retval = pthread_create(threads+th_id, NULL, telemetry_thread, NULL);
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

  dummy_thread3();
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