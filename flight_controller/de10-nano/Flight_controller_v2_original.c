//#include "Flight_controller_v2.h"
#include "basic_lib/PREDICTBasic.h"
#include "sensors/PREDICTSensors.h"
#include "FC_functionalities/PREDICTFunctions.h"
#include "thread/i2c_thread_core1.h"
#include "thread/gps_thread_core3.h"
///PREDICT PROJECT FLIGHT CONTROLLER
//this code is for autonomous flight controller
// This code callibrates the IMU before start and stabilizes the drone using the RPY values from IMU. 
// It then takes inputs from the transmitter to have a 6DOF control of the drone 

//////////////////////////////////////////////////////////
// Procedure to start the drone:

// - go to t-crest/patmos folder
// - upload the code using the command "make APP=de10-IMU comp download"
// - wait for the program to upload
// - wait for the IMU to callibrate which is indicated by bliking LED on FPGA
// - unplug the upload cable
// - throttle low and yaw left to arm the motors
// - throttle low and yaw right to disarm the motors
// - stop the code before uploading, which is done by throttle low, yaw right and roll left and pitch back

///controls
 //                    throttle up                                             pitch forward
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |                          
 //                          |                                                       |       
 // yaw left -------------------------------yaw right       roll left -------------------------------roll right 
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                    throttle down                                           pitch down


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int MAX_CNT = 30000;
int MAIN_CNT = 0;

// dummy threads
void dummy_thread0()
{
  int i = 0;
  while (!program_off)
  {
    printf("Dummy thread 0, cycle %d\n", i);
    millis(5000);
    if(i>=10)
    {
      pthread_mutex_lock(&mutex);
      program_off = true;
      pthread_mutex_unlock(&mutex);
    }
    i++;
  }

}

void dummy_thread1()
{  
  int i = 0;
  while (!program_off)
  {
    printf("Dummy thread 1, cycle %d\n", i);
    millis(3000);
    i++;
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
  }

}

void dummy_thread2()
{  
  int i = 0;
  while (!program_off)
  {
    millis(2000);
    printf("Dummy thread 2, cycle %d\n", i);
    millis(4000);
    i++;
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
  }

}

void dummy_thread3()
{
  int i = 0;
  while (!program_off)
  {
    printf("Dummy thread 3, cycle %d\n", i);
    millis(12000);
    i++;
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
  }
}

int main(int argc, char **argv)
{
  int cpucnt = 4;
  int th_id, retval;
  if(PRINT_COMMANDS)printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);


/////////////////multi core thread iniitate
// THE FUNCTION CALLS HAS BEEN MODIFIED
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
  th_id=1;
  
//  retval = pthread_create(threads+th_id, NULL, i2c_thread, NULL);
  retval = pthread_create(threads+th_id, NULL, dummy_thread2, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
  th_id++;
//  retval = pthread_create(threads+th_id, NULL, receiver_thread, NULL);
  retval = pthread_create(threads+th_id, NULL, dummy_thread1, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
//   th_id++;
  // retval = pthread_create(threads+th_id, NULL, dummy_thread3, NULL);
  // if(retval != 0)
  // {
  //   printf("Unable to start thread %d, error code %d\n", th_id, retval);
  //   return retval;
  // }
  
  pthread_mutex_unlock(&mutex);
////////////////////////////

printf("thread initaited\n");

  //Before starting the avarage accelerometer value is preloaded into the variables.
  millis(1000);
  pthread_mutex_lock(&mutex);
  pthread_mutex_unlock(&mutex);
  // for (start = 0; start <= 24; start++)acc_z_average_short[start] = acc_z;
  // for (start = 0; start <= 49; start++)acc_z_average_long[start] = acc_z;
  // acc_z_average_short_total = acc_z * 25;
  // acc_z_average_long_total = acc_z * 50;
  // start = 0;

  // if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  // if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.

  // loop_timer = get_cpu_usecs();                                        //Set the timer for the first loop.
  printf("main loop start\n");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  while(!program_off)
  { 
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);

        //Stopping the code: throttle low and yaw right, roll left and pitch down
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      ///need to add safety land and then switch off
      program_off = 1;
      // break; 
    }
    // In case the receiver is not calibrated
    if(MAIN_CNT<=MAX_CNT){
      MAIN_CNT++;
    } else {
      program_off = 1;
    }

    if(receiver_watchdog < 750)receiver_watchdog ++;
    if(receiver_watchdog == 750 && start == 2){
      channel_1 = 1500;
      channel_2 = 1500;
      channel_3 = 1500;
      channel_4 = 1500;
      error = 8;
      if (number_used_sats > 5){
        if(home_point_recorded == 1)channel_5 = 2000;
        else channel_5 = 1750;
      }
      else channel_5 = 1500;    
    }
    //Some functions are only accessible when the quadcopter is off.
    if (start == 0) {
      //Change settings
      if (channel_6 >= 1900 && previous_channel_6 == 0) {
        previous_channel_6 = 1;
        if (setting_adjust_timer > get_cpu_usecs()/1000)setting_click_counter ++;
        else setting_click_counter = 0;
        setting_adjust_timer = get_cpu_usecs()/1000 + 1000;
        if (setting_click_counter > 3) {
          setting_click_counter = 0;
          change_settings();
        }
      }
      if (channel_6 < 1900)previous_channel_6 = 0;
    }
    heading_lock = 0;
    if (channel_6 > 1200)heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

    flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
    if (channel_5 >= 1200 && channel_5 < 1600)flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
    if (channel_5 >= 1600 && channel_5 < 1950)flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3
    if (channel_5 >= 1950 && channel_5 < 2100) {
      if (waypoint_set == 1 && home_point_recorded == 1 && start == 2)flight_mode = 4;
      else flight_mode = 3;
    }

    if (flight_mode <= 3) {
      return_to_home_step = 0;
      return_to_home_lat_factor = 0;
      return_to_home_lon_factor = 0;
    }

    return_to_home();                                                                //Jump to the return to home step program.
    flight_mode_signal();                                                            //Show the flight_mode via the green LED.
    LED_out(1);                                                                  //Show the error via the red LED.
    if (gps_add_counter >= 0)gps_add_counter --;

    // read_gps();
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    channel_1_base = channel_1;                                                      //Normally channel_1 is the pid_roll_setpoint input.
    channel_2_base = channel_2;                                                      //Normally channel_2 is the pid_pitch_setpoint input.
    gps_man_adjust_heading = angle_yaw;                                              //
    //When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
    //At startup the heading is registerd in the variable course_lock_heading.
    //First the course deviation is calculated between the current heading and the course_lock_heading.
    //Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
    if (heading_lock == 1) {
       heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
      channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
      channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
      gps_man_adjust_heading = course_lock_heading;

    }
    if (flight_mode >= 3 && waypoint_set == 1) {
      pid_roll_setpoint_base = 1500 + gps_roll_adjust;
      pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
    }
    else {
      pid_roll_setpoint_base = channel_1_base;
      pid_pitch_setpoint_base = channel_2_base;
    }
    //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
    if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
    if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
    if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
    if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;

    calculate_pid();                                                                 //Calculate the pid outputs based on the receiver inputs.

    start_stop_takeoff();                                                            //Starting, stopping and take-off detection

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //1410.1 = 112.81 / 0.08.
    if(battery_voltage_available)
    {
      battery_voltage = battery_voltage * 0.92 + batteryRead() / 1410.1;

    //Turn on the led if battery voltage is to low. Default setting is 10.5V
      if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)error = 1;

    }


    
    //The variable base_throttle is calculated in the following part. It forms the base throttle for every motor.
    if (takeoff_detected == 1 && start == 2) {                                         //If the quadcopter is started and flying.
      throttle = channel_3 + takeoff_throttle;                                         //The base throttle is the receiver throttle channel + the detected take-off throttle.
      if (flight_mode >= 2) {                                                          //If altitude mode is active.
        throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
      }
    }

    if (start == 2) {                                                                //The motors are started.
      if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

      if(battery_voltage_available==1)
      { 
        if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //Is the battery connected?
          esc_1 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-1 pulse for voltage drop.
          esc_2 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-2 pulse for voltage drop.
          esc_3 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-3 pulse for voltage drop.
          esc_4 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-4 pulse for voltage drop.
        }
      }
      if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Keep the motors running.
      if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;                        //Keep the motors running.
      if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;                        //Keep the motors running.
      if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;                        //Keep the motors running.

      if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
      if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
      if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
      if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
    }

    else {
      esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
      esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
      esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
      esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
    }

    motor_publish = true;

    if (get_cpu_usecs() - loop_timer > 40050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
    while (get_cpu_usecs() - loop_timer < 40000);                                            //We wait until 4000us are passed.
    //if(PRINT_COMMANDS)printf("main loop_timer: %llu\n",get_cpu_usecs() - loop_timer );
  	//if(PRINT_COMMANDS)printf("flight_mode: %d  start: %d\n",flight_mode, start );
    //if(PRINT_COMMANDS)printf("esc1:%d esc2:%d esc3:%d esc4:%d\n",esc_1, esc_2, esc_3, esc_4 );
    loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.


  }


  //dummy_thread0(); // <- This needs to be removed
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

  th_id++;
  printf("Waiting for threard nr.%d... ", th_id);
  retval = pthread_join(*(threads+th_id), &dummy);
  if(retval != 0)
  {
    printf("Unable to join thread %d, error code %d\n", th_id, retval);
  }  
  // th_id++;
  // th_id = 3;
  // printf("Waiting for threard nr.%d... ", th_id);
  // retval = pthread_join(*(threads+th_id), &dummy);
  // if(retval != 0)
  // {
  //   printf("Unable to join thread %d, error code %d\n", th_id, retval);
  // }  
  
  free(threads);

  return 0;
}