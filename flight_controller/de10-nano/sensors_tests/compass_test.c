// Created by rahul on 11/22/20.
//

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../basic_lib/i2c_master.h"
#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"
#include "../gyro/gyro.h"
#include "../compass/compass.h"

void callibrate_compass(void) {
    compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
    LED_out(1);                                                             //The red led will indicate that the compass calibration is active.
    LED_out(0);                                                            //Turn off the green led as we don't need it.
                                             //Send telemetry data to the ground station.
    // micros(3700);                                                 //Simulate a 250Hz program loop.
    read_compass();                                                          //Read the raw compass values.
    //In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
    compass_calibration_on = 0;                                                //Reset the compass_calibration_on variable.


    setup_compass();                                                           //Initiallize the compass and set the correct registers.
    read_compass();                                                            //Read and calculate the compass data.
    angle_yaw = actual_compass_heading;                                        //Set the initial compass heading.

    LED_out(0);
    for (error = 0; error < 15; error ++) {
        LED_out(1);
        millis(50);
        LED_out(0);
        millis(50);
    }

    error = 0;

    loop_timer = get_cpu_usecs();                                                     //Set the timer for the next loop.
}


void callibrate_level(void) {
    level_calibration_on = 1;

    LED_out(1);
    LED_out(0);

    acc_pitch_cal_value = 0;
    acc_roll_cal_value = 0;

    int timer = get_cpu_usecs();
    for (error = 0; error < 64; error ++) {
        // send_telemetry_data();                                                   //Send telemetry data to the ground station.
        gyro_signalen();
        acc_pitch_cal_value += acc_y;
        acc_roll_cal_value += acc_x;
        if (acc_y > 500 || acc_y < -500)error = 80;
        if (acc_x > 500 || acc_x < -500)error = 80;
    }

    acc_pitch_cal_value /= 64;
    acc_roll_cal_value /= 64;

    LED_out(0);
    if (error < 80) {
        //EEPROM.write(0x10 + error, compass_cal_values[error]);
        for (error = 0; error < 15; error ++) {
            LED_out(1);
            millis(50);
            LED_out(0);
            millis(50);
        }
        error = 0;
    }
    else error = 3;
    level_calibration_on = 0;
    gyro_signalen();
    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

    if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
    }
    angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;
    loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
}

int main()
{
    callibrate_compass();
    callibrate_level();
    callibrate_gyro();
    gyro_setup();
    printf("imu setup done");
    setup_compass();                                              //Initiallize the compass and set the correct registers.
    printf("compass setup done");
    read_compass();                                               //Read and calculate the compass data.
    angle_yaw = actual_compass_heading;                           //Set the initial compass heading.//
    printf("hello compass and imu");
    loop_timer = get_cpu_usecs();
    for (int i = 0; i < 1000; ++i)
    {
        gyro_signalen();
        read_compass();



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

        angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
        if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
        else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));        //Calculate the total accelerometer vector.


      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
        angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
        angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.
   
        if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }
        pitch_level_adjust = angle_pitch;                                           //Calculate the pitch angle correction.
        roll_level_adjust = angle_roll;                                             //Calculate the roll angle correction.

        printf("pitch:: %f  roll: %f yaw : %f actual_compass_heading: %f \n",angle_pitch,angle_roll,angle_yaw,actual_compass_heading);
        while (get_cpu_usecs() - loop_timer < dt*1000000);                                            //We wait until 4000us are passed.
        loop_timer = get_cpu_usecs();



    }
    return 0;
}


