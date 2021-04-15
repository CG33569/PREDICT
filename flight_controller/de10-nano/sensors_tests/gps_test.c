#include "PREDICTdummy.h"

void gps_thread()
{
    struct gps_tpv GPS;
    bool gps_result = false, gps_output;
    char msg[256]="";

    if(PRINT_COMMANDS)
    {
        pthread_mutex_lock(&mutex);
        printf("GPS setup\n");
        pthread_mutex_unlock(&mutex);
    }
    gps_init_tpv(&GPS);

    int program_off_cnt = 0;

    while(!program_off){
        pthread_mutex_lock(&mutex);
        printf(" ---> CALL READ GPS <--- \n");
        //pthread_mutex_unlock(&mutex);
        gps_output = read_gps(&GPS);
        //pthread_mutex_unlock(&mutex);
        if(gps_output)
        {
            sprintf(msg,"RESULT long = %d, lat = %d, mode = %d\n", GPS.longitude, GPS.latitude, GPS.mode);
        }else
        {
            sprintf(msg,"GPS data not valid\n");
        }
        //pthread_mutex_lock(&mutex);
        printf("%s",msg);
        program_off = (program_off_cnt >=2);
        pthread_mutex_unlock(&mutex);
      program_off_cnt++;  
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
  retval = pthread_create(threads+th_id, NULL, dummy_thread2, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }

  th_id=3;
  retval = pthread_create(threads+th_id, NULL, gps_thread, NULL);
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