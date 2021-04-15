#ifndef PREDICT_DUMMY_H
#define PREDICT_DUMMY_H

#include "../sensors/PREDICTSensors.h"

void dummy_thread1()
{  
  pthread_mutex_lock(&finishmutex);
  int finish = program_off;
  pthread_mutex_unlock(&finishmutex);

  int i = 0;
  while (!finish)
  {
    //pthread_mutex_lock(&mutex);
    //printf("Dummy thread 1, cycle %d\n", i);
    //pthread_mutex_unlock(&mutex);
    millis(10000);
    i++;

    pthread_mutex_lock(&finishmutex);
    int finish = program_off;
    pthread_mutex_unlock(&finishmutex);
  }

}

void dummy_thread2()
{  
  pthread_mutex_lock(&finishmutex);
  int finish = program_off;
  pthread_mutex_unlock(&finishmutex);
  int i = 0;
  while (!finish)
  {
    millis(2000);
    //pthread_mutex_lock(&mutex);
    //printf("Dummy thread 2, cycle %d\n", i);
    //pthread_mutex_unlock(&mutex);
    millis(7000);
    i++;
  }

}

void dummy_thread3()
{
  pthread_mutex_lock(&finishmutex);
  int finish = program_off;
  pthread_mutex_unlock(&finishmutex);

  int i = 0;
  while (!finish)
  {    
    millis(4000);
    //pthread_mutex_lock(&mutex);
    //printf("Dummy thread 3, cycle %d\n", i);
    //pthread_mutex_unlock(&mutex);
    millis(4000);
    pthread_mutex_lock(&finishmutex);
    finish = program_off;
    pthread_mutex_unlock(&finishmutex);
    i++;
  }
}


#endif //PREDICT_DUMMY_H