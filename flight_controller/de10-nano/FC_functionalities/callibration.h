#ifndef PATMOS_CALLIB_H
#define PATMOS_CALLIB_H

#include "../basic_lib/i2c_master.h"
#include "../basic_lib/PREDICTBasic.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the level and compass calibration procedres are handled.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void callibrate_compass(void) {
  if(PRINT_COMMANDS)printf("compass callibration\n");
   // LED_out(1);                                                             //The red led will indicate that the compass calibration is active.
   // LED_out(0);                                                            //Turn off the green led as we don't need it.
  compass_calibration_on = 1;
   for (int j = 0; j < 100; ++j)
   {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.
    // intr_handler();
    // send_telemetry_data();                                                   //Send telemetry data to the ground station.
    
    ////compass read
    compass_y = i2c_reg8_read16b(compass_address,0x03);                 //Add the low and high byte to the compass_y variable.
    compass_y *= -1;                                              //Invert the direction of the axis.
    compass_z = i2c_reg8_read16b(compass_address,0x05);                 //Add the low and high byte to the compass_z variable.;
    compass_x = i2c_reg8_read16b(compass_address,0x07);                 //Add the low and high byte to the compass_x variable.;
    compass_x *= -1;                                              //Invert the direction of the axis.


//    printf("compasss: %d, %d, %d ", compass_x,compass_y,compass_z);
    //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
        compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
        compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
        compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
        compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
        compass_x += compass_offset_x;                              //Add the x-offset to the raw value.
    }

    //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
    compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

    //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    
    if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);
    actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

     //In the following lines the maximum and minimum compass values are detected and stored.
     pthread_mutex_lock(&mutex);
     if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
     if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
     if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
     if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
     if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
     if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
     pthread_mutex_unlock(&mutex);
   }

   pthread_mutex_lock(&mutex);
   compass_calibration_on = 0;                                                //Reset the compass_calibration_on variable.
   pthread_mutex_unlock(&mutex);


   ////compass setup
   if(PRINT_COMMANDS)printf("compass setup\n");
    i2c_reg8_write8(compass_address,0x00,0x78);                                            //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
    i2c_reg8_write8(compass_address,0x01,0x20);                                            //Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
    i2c_reg8_write8(compass_address,0x02,0x00);                                            //Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.

    //Calculate the calibration offset and scale values
    pthread_mutex_lock(&mutex);
    compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
    compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

    compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
    compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
    compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
    pthread_mutex_unlock(&mutex);


   ////compass read
    compass_y = i2c_reg8_read16b(compass_address,0x03);                 //Add the low and high byte to the compass_y variable.
    compass_y *= -1;                                              //Invert the direction of the axis.
    compass_z = i2c_reg8_read16b(compass_address,0x05);                 //Add the low and high byte to the compass_z variable.;
    compass_x = i2c_reg8_read16b(compass_address,0x07);                 //Add the low and high byte to the compass_x variable.;
    compass_x *= -1;                                              //Invert the direction of the axis.


//    printf("compasss: %d, %d, %d ", compass_x,compass_y,compass_z);
    //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
        compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
        compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
        compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
        compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
        compass_x += compass_offset_x;                              //Add the x-offset to the raw value.
    }

    //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
    compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

    //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    
    if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);
    actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.



   pthread_mutex_lock(&mutex);
   angle_yaw = actual_compass_heading;                                        //Set the initial compass heading.
   pthread_mutex_unlock(&mutex);
  }


void callibrate_level(void) {
  if(PRINT_COMMANDS)printf("level callibration\n");
  level_calibration_on=1;
  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (int i = 0; i < 64; i ++) {
    //gyro read
    acc_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
    acc_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
    acc_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    gyro_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
    gyro_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
    gyro_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
    if (level_calibration_on == 0) {
        acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
        acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
    }


    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
  }
  pthread_mutex_lock(&mutex);
  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;
  pthread_mutex_unlock(&mutex);

  pthread_mutex_lock(&mutex);
  level_calibration_on = 0;
  pthread_mutex_unlock(&mutex);
    //gyro read
  acc_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
  acc_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
  acc_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
  gyro_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
  gyro_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
  gyro_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
  if (level_calibration_on == 0) {
      acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
      acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
  }

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }
  pthread_mutex_lock(&mutex);
  angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  pthread_mutex_unlock(&mutex);
  // loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
}


#endif //PATMOS_CALLIB_H