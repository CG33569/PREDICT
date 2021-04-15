#include "PREDICTdummy.h"

void imu_thread()
{
  pthread_mutex_lock(&printmutex);
  printf("Hello MCU6050!\n");
  pthread_mutex_unlock(&printmutex);
  unsigned int signature = 0;

  signature = i2c_reg8_read8(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I);
  pthread_mutex_lock(&printmutex);
  printf("Signature = 0x%.2X\n", signature);
  pthread_mutex_unlock(&printmutex);
  gyro_setup();
  int first_angle = false;
  pthread_mutex_lock(&printmutex);
  printf("gyro callibration start\n");
  pthread_mutex_unlock(&printmutex);
  callibrate_gyro();
  unsigned loop_timer = get_cpu_usecs();
    for (int i = 0; i < 1000; ++i)
    {

      gyro_signalen();

      //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
      gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
      gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
      gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += (gyro_pitch / 65.5)*dt;                                     //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += (gyro_roll / 65.5)*dt;                                       //Calculate the traveled roll angle and add this to the angle_roll variable.
      angle_yaw +=  (gyro_yaw / 65.5)*dt;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
      if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
      else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
      angle_pitch -= angle_roll * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the pitch angle to the roll angel.

      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));        //Calculate the total accelerometer vector.


      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
        angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
        angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.
   
        if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }
      // pthread_mutex_lock(&printmutex);
      // printf("pitch angle: %f  roll angle: %f yaw angle: %f \n",angle_pitch,angle_roll,angle_yaw);
      // pthread_mutex_unlock(&printmutex);
      char str2[15];

      strcpy(str2, "IMU running");
      puts(str2);
      while (get_cpu_usecs() - loop_timer < 40000);     
      loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
      
    }
  pthread_mutex_lock(&finishmutex);
  program_off = true;
  pthread_mutex_unlock(&finishmutex);
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
  retval = pthread_create(threads+th_id, NULL, imu_thread, NULL);
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

  dummy_thread1();
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