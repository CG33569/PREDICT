#include "PREDICTdummy.h"

void motors_thread()
{
  // Start output values
  pthread_mutex_lock(&mutex);
  printf("-- START MOTOR TEST -- \n");
  pthread_mutex_unlock(&mutex);
  actuator_write(0, 1000);
  actuator_write(1, 1000);
  actuator_write(2, 1000);
  actuator_write(3, 1000);

  for (int i=0; i<2; ++i) {

    // 2 Seconds wait for initialization
    millis(2000);

    pthread_mutex_lock(&mutex);
    printf("--Start %d\n\r",i);
    pthread_mutex_unlock(&mutex);
    //Test motor 1
    actuator_write(0, 1300);
    actuator_write(1, 1000);
    actuator_write(2, 1000);
    actuator_write(3, 1000);
    // 5 seconds spinning at that velocity
    millis(5000);

    //Test motor 2
    pthread_mutex_lock(&mutex);
    printf("Test motor 2\n\r");
    pthread_mutex_unlock(&mutex);
    actuator_write(0, 1000);
    actuator_write(1, 1300);
    actuator_write(2, 1000);
    actuator_write(3, 1000);
    // 5 seconds spinning at that velocity
    millis(5000);

    //Test motor 3
    pthread_mutex_lock(&mutex);
    printf("Test motor 3\n\r");
    pthread_mutex_unlock(&mutex);
    actuator_write(0, 1000);
    actuator_write(1, 1000);
    actuator_write(2, 1300);
    actuator_write(3, 1000);
    // 5 seconds spinning at that velocity
    millis(5000);

    //Test motor 4
    pthread_mutex_lock(&mutex);
    printf("Test motor 4\n\r");
    pthread_mutex_unlock(&mutex);
    actuator_write(0, 1000);
    actuator_write(1, 1000);
    actuator_write(2, 1000);
    actuator_write(3, 1300);
    // 5 seconds spinning at that velocity
    millis(5000);

    pthread_mutex_lock(&mutex);
    printf("--End round %d\n\r", i);
    pthread_mutex_unlock(&mutex);
  }

  // Stop the motors before exit the programme
  pthread_mutex_lock(&mutex);
  printf("END OF TEST\n\r");
  pthread_mutex_unlock(&mutex);
  actuator_write(0, 1000);
  actuator_write(1, 1000);
  actuator_write(2, 1000);
  actuator_write(3, 1000);

  program_off = true;
	 
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
  retval = pthread_create(threads+th_id, NULL, motors_thread, NULL);
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

  free(threads);

  return 0;
}