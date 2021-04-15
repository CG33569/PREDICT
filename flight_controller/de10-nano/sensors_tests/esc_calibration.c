#include "../sensors/PREDICTSensors.h"
#include "../FC_functionalities/PREDICTFunctions.h"
#include "../PREDICTthread.h"
#include "../sensors_tests/PREDICTdummy.h"

// turn on the transmitter and put the throttle to full
// then switch on the fpga and upload this program
// waitfor the motors to beep 3 times in sets of 3
// lower the throttle of tranmitter and wait for the motors to beep again
// motors callibrated succesfully

int main() {

  int cpucnt = 3;
  
  printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);
  
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
  for(int i = 1; i < cpucnt;)
  {
    int retval = pthread_create(threads+i, NULL, dummy_thread1, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
    i++;
    retval = pthread_create(threads+i, NULL, receiver_thread, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
    i++;
  }
  pthread_mutex_unlock(&mutex);
  __uint32_t loop_timer = get_cpu_usecs();
  while(!program_off)
  { 
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
        //Stopping the code: throttle low and yaw right, roll left and pitch down
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      ///need to add safety land and then switch off
      program_off = true;
      break; 
    }

    esc_1 = channel_3;                                                   //Set the pulse for motor 1 equal to the throttle channel.
    esc_2 = channel_3;                                                   //Set the pulse for motor 2 equal to the throttle channel.
    esc_3 = channel_3;                                                   //Set the pulse for motor 3 equal to the throttle channel.
    esc_4 = channel_3;                                                   //Set the pulse for motor 4 equal to the throttle channel.
    motor_publish = true;
    while (get_cpu_usecs() - loop_timer < dt*1000000);                                            //We wait until 4000us are passed.
    loop_timer = get_cpu_usecs();
  }
  
  for(int i = 1; i < cpucnt; i++) {
    void * dummy;
    printf("i:%d\n",i );
    int retval = pthread_join(*(threads+i), &dummy);
    if(retval != 0)
    {
      printf("Unable to join thread %d, error code %d\n", i, retval);
      return retval;
    }
  }
  
  free(threads);
  return 0;
}

