/*
    This program tests POSIX mutexes.

    Author: Torur Biskopsto Strom
    Copyright: DTU, BSD License
*/

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <machine/patmos.h>
#include <machine/rtc.h>

// Uncomment the line below to see the test fail
// when not using a mutex and cpucnt > 1
//#define WITHOUT_MUTEX 1

#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3
//Receiver controller
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

const unsigned int CPU_PERIOD = 20; //CPU period in ns.


void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}

//Reads from propulsion specified by propulsion ID (0 to 4)
int receiver_read(unsigned int receiver_id){
  return *(RECEIVER + receiver_id);
  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}


#ifdef WITHOUT_MUTEX
_UNCACHED int cnt = 0;
#else
int cnt = 0;
#endif

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void * work(void * arg) {
  int id = get_cpuid();
  pthread_mutex_lock(&mutex);
  pthread_mutex_unlock(&mutex);
  for(int i = 0; i < 1000; i++) {
#ifdef WITHOUT_MUTEX
    asm volatile ("" : : : "memory");
#else
    pthread_mutex_lock(&mutex);
#endif
    cnt++;
#ifdef WITHOUT_MUTEX
    asm volatile ("" : : : "memory");
#else
    pthread_mutex_unlock(&mutex);
#endif
  }
  return NULL;
}

int main() {

  actuator_write(m1, 1000);                                               //give motors 1000us pulse.
  actuator_write(m2, 1000);
  actuator_write(m3, 1000);
  actuator_write(m4, 1000);

  int cpucnt = get_cpucnt();
  
  printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);
  
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
  for(int i = 1; i < cpucnt; i++)
  {
    int retval = pthread_create(threads+i, NULL, work, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
  }
  pthread_mutex_unlock(&mutex);
  work(NULL);
  
  for(int i = 1; i < cpucnt; i++) {
    void * dummy;
    int retval = pthread_join(*(threads+i), &dummy);
    if(retval != 0)
    {
      printf("Unable to join thread %d, error code %d\n", i, retval);
      return retval;
    }
  }
  
  free(threads);
  
  // Locking to update the global state (get the newest value of cnt)
  pthread_mutex_lock(&mutex);
  printf("Expected count=%d, actual count=%d\n",cpucnt*1000,cnt);
  pthread_mutex_unlock(&mutex);
}
