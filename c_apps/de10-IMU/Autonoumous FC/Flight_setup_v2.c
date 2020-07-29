//Edit the low, center and end values of the receiver inputs
//also edit the direction of the reciver channels
//MAY BE ADD TWBR which is ic speed to 400kHz

#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//battery voltage read
#define BATTERY ( ( volatile _IODEV unsigned * )  PATMOS_IO_AUDIO )

//Receiver controller
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

//LEDs
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )

//I2C controller
#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68

//MCU6050 registers
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_GYRO_CONFIG        0x1B   // R
#define MPU6050_ACCEL_CONFIG       0x1C   // R
#define MPU6050_CONFIG_REG         0x1A   // R

#define BARO_REG                   0xA0   // R

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//Let's declare some variables so we can use them in the complete program.
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
unsigned int disable_throttle, flip32;
unsigned int error;
unsigned int loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
int loop_counter;
unsigned int data, start, warning;
short int acc_axis[4], gyro_axis[4];
int temperature;
int gyro_axis_cal[4], acc_axis_cal[4];
int cal_int;
int channel_1_start, channel_1;
int channel_2_start, channel_2;
int channel_3_start, channel_3;
int channel_4_start, channel_4;
int channel_5_start, channel_5;
int channel_6_start, channel_6;
int measured_time, measured_time_start;
unsigned int channel_select_counter;

//Barometer variables.
unsigned int C[7];
unsigned int barometer_counter, temperature_counter;
int OFF, OFF_C2, SENS, SENS_C1, P;
unsigned int raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int dT, dT_C5;

//Compass_variables.
int compass_x, compass_y, compass_z;

unsigned int gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
unsigned int MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
unsigned int compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.


float batteryRead()
{
  return *(BATTERY);
}

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

void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}
void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}

//Writes to i2c, returns -1 if there was an error, 0 if succeded
int i2c_write(unsigned char chipaddress, unsigned char regaddress, unsigned char data)
{
  I2C = ((((unsigned int) data & 0x000000FF) << 16) | (((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) & 0xFFFFFFFE;
  if ((I2C & 0x00000100) != 0)
  {
    return -1;
  }else{
    return 0;
  }
}

//Reads to i2c, returns the read value (8 bits), if there was an error the returned value is -1 (0xFFFFFFFF)
int i2c_read(unsigned char chipaddress, unsigned char regaddress)
{
  I2C = ((((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) | 0x00000001;
  unsigned int I2C_tmp = I2C;
  if ((I2C_tmp & 0x00000100) != 0)
  {
    return -1;
  }else{
    return (int)((unsigned int)(I2C_tmp) & 0x000000FF);
  }
}

//Blinks the LEDs once
void blink_once()
{
  int i, j;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0001;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0000;
  return;
}

void LED_out(int i){
  if(i==1) LED = 0x0001;
  else LED = 0x0000;
  return;
}

// interrupt handler
void timer_setup(void) {
  exc_prologue();

  // read the receiver pwm duty cycle
  receiver_input[1] = receiver_read(0);
  receiver_input[2] = receiver_read(1);
  receiver_input[3] = receiver_read(2);
  receiver_input[4] = receiver_read(3);

  exc_epilogue();
}

void set_gyro_registers()
{
  //Setup the MPU-6050
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
}

void gyro_signalen()
{
  //Read the MPU-6050
  ACCEL_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
  ACCEL_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_L);
  ACCEL_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
  ACCEL_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_L);
  ACCEL_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
  ACCEL_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_L);
  TEMP_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_L);
  TEMP_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
  GYRO_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
  GYRO_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_L);
  GYRO_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
  GYRO_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_L);
  GYRO_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
  GYRO_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_L);

  acc_axis[1] = (ACCEL_X_H<<8|ACCEL_X_L);                    //Add the low and high byte to the acc_x variable.
  acc_axis[2] = (ACCEL_Y_H<<8|ACCEL_Y_L);                  //Add the low and high byte to the acc_y variable.
  acc_axis[3] = (ACCEL_Z_H<<8|ACCEL_Z_L);                    //Add the low and high byte to the acc_z variable.
  temperature = (TEMP_H<<8|TEMP_L);                    //Add the low and high byte to the temperature variable.
  gyro_axis[1] = (GYRO_X_H<<8|GYRO_X_L);                   //Read high and low part of the angular data.
  gyro_axis[2] = (GYRO_Y_H<<8|GYRO_Y_L);                   //Read high and low part of the angular data.
  gyro_axis[3] = (GYRO_Z_H<<8|GYRO_Z_L);                   //Read high and low part of the angular data.

  if(cal_int == 2000)
  {
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
}

void print_intro(void) {
  printf("\n");
  printf("\n");
  printf("===================================================\n");
  printf("          YMFC-32 quadcopter setup tool\n");
  printf("===================================================\n");
  printf("a = Read the receiver input pulses\n");
  printf("c = Read the raw gyro values\n");
  printf("d = Read the raw accelerometer values\n");
  printf("e = Check the IMU angles\n");
  printf("f = Test the LEDs\n");
  printf("g = Read the battery voltage input\n");
  printf("h = Check barometer\n");
  printf("i = Check GPS\n");
  printf("j = Check HMC5883L compass\n");
  printf("===================================================\n");
  printf("1 = Check motor 1 (front right, counter clockwise direction)\n");
  printf("2 = Check motor 2 (rear right, clockwise direction)\n");
  printf("3 = Check motor 3 (rear left, counter clockwise direction)\n");
  printf("4 = Check motor 4 (front left, clockwise direction)\n");
  printf("5 = Check all motors");
  printf("===================================================\n");
  printf("For support and questions: www.brokking.net\n");
  printf("\n");
  if (!disable_throttle) {                                      //If the throttle is not disabled.
    printf("===================================================\n");
    printf("     WARNING >>>THROTTLE IS ENABLED<<< WARNING\n");
    printf("===================================================\n");
  }
}

void reading_receiver_signals(void) {
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    millis(250);
    //For starting the motors: throttle low and yaw left (step 1).
    if (channel_3 < 1100 && channel_4 < 1100)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && channel_3 < 1100 && channel_4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel_3 < 1100 && channel_4 > 1900)start = 0;

    printf("Start: %d", start);

    printf("  Roll:");
    if (channel_1 - 1480 < 0)printf("<<<");
    else if (channel_1 - 1520 > 0)printf(">>>");
    else printf("-+- %d",channel_1);

    printf("  Pitch:");
    if (channel_2 - 1480 < 0)printf("^^^");
    else if (channel_2 - 1520 > 0)printf("vvv");
    else printf("-+- %d",channel_2);

    printf("  Throttle:");
    if (channel_3 - 1480 < 0)printf("vvv");
    else if (channel_3 - 1520 > 0)printf("^^^");
    else printf("-+- %d",channel_3);

    printf("  Yaw:");
    if (channel_4 - 1480 < 0)printf("<<<");
    else if (channel_4 - 1520 > 0)printf(">>>");
    else printf("-+- %d",channel_4);

    printf("  CH5: %d",channel_5);

    printf("  CH6: %d \n",channel_6);

  }
  print_intro();
}

void read_gyro_values(void) {
  cal_int = 0;                                                                        //If manual calibration is not used.

  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    millis(250);                                                                           //Print the receiver values on the screen every 250ms.
    if (data == 'c') {                                                                    //If the user requested a 'c'.
      if (cal_int != 2000) {
        gyro_axis_cal[1] = 0;                                                             //Reset calibration variables for next calibration.
        gyro_axis_cal[2] = 0;                                                             //Reset calibration variables for next calibration.
        gyro_axis_cal[3] = 0;                                                             //Reset calibration variables for next calibration.
        printf("Calibrating the gyro");
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
          if (cal_int % 125 == 0) {                                                       //After every 125 readings.
            blink_once();
            printf(".");                                                            //Print a dot to show something is still working.
          }
          gyro_signalen();                                                                //Read the gyro output.
          gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
          gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
          gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
          millis(4);                                                                       //Small delay to simulate a 250Hz loop during calibration
        }
        printf(".\n");
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
        gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
        gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.
        printf("X calibration value: %d \n",gyro_axis_cal[1]);
        printf("Y calibration value: %d \n",gyro_axis_cal[2]);
        printf("Z calibration value: %d \n",gyro_axis_cal[3]);
      }
      gyro_signalen();                                                                    //Read the gyro output.
      printf("Gyro_x = %d ",gyro_axis[1]);
      printf(" Gyro_y = %d ",gyro_axis[2]);
      printf(" Gyro_z = %d \n",gyro_axis[3]);
    }
    else {                                                                                //If the user requested a 'd'.
      gyro_signalen();                                                                    //Read the accelerometer output.
      printf("ACC_x = %d ",acc_axis[1]);
      printf(" ACC_y = %d ",acc_axis[2]);
      printf(" ACC_z = %d \n",acc_axis[3]);
    }
  }
  print_intro();                                                                          //Print the intro to the serial monitor.

}

void check_battery_voltage(void) {
  loop_counter = 0;                                                                       //Reset the loop counter.
  battery_voltage = analogRead(4);                                                        //Set battery voltage.
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    micros(4000);                                                              //Wait for 4000us to simulate a 250Hz loop.

    loop_counter++;
    if (loop_counter == 250) {                                                            //Print the battery voltage every second.
      printf("Voltage = %fV \n", battery_voltage / 112.81, 1);                                          //Print the avarage battery voltage to the serial monitor.
      loop_counter = 0;                                                                   //Reset the loop counter.
    }
    //A complimentary filter is used to filter out the voltage spikes caused by the ESC's.
    battery_voltage = (battery_voltage * 0.99) + ((float)batteryRead() * 0.01);
  }
  loop_counter = 0;                                                                       //Reset the loop counter.
  print_intro();                                                                          //Print the intro to the serial monitor.
}



void check_imu_angles(void) {
  unsigned int first_angle = 0;
  loop_counter = 0;
  first_angle = false;
  cal_int = 0;                                                                        //If manual calibration is not used.

  while (data != 'q') {                                                                 //Stay in this loop until the data variable data holds a q.
    loop_timer = get_cpu_usecs() + 4000;                                                       //Set the loop_timer variable to the current micros() value + 4000.

    if (cal_int == 0) {                                                                 //If manual calibration is not used.
      gyro_axis_cal[1] = 0;                                                             //Reset calibration variables for next calibration.
      gyro_axis_cal[2] = 0;                                                             //Reset calibration variables for next calibration.
      gyro_axis_cal[3] = 0;                                                             //Reset calibration variables for next calibration.
      printf("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
        if (cal_int % 125 == 0) {
          blink_once();                                         //Change the led status to indicate calibration.
          printf(".");
        }
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
        millis(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
      }
      printf(". \n");
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
      gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
      gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.
    }

    gyro_signalen();                                                                    //Let's get the current gyro data.

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_axis[2] * 0.0000611;                                            //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += gyro_axis[1] * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch -= angle_roll * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    if (acc_axis[1] > 4096)acc_axis[1] = 4096;                                          //Limit the maximum accelerometer value.
    if (acc_axis[1] < -4096)acc_axis[1] = -4096;                                        //Limit the maximum accelerometer value.
    if (acc_axis[2] > 4096)acc_axis[2] = 4096;                                          //Limit the maximum accelerometer value.
    if (acc_axis[2] < -4096)acc_axis[2] = -4096;                                        //Limit the maximum accelerometer value.


    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_axis[1] / 4096) * 57.296;                         //Calculate the pitch angle.
    angle_roll_acc = asin((float)acc_axis[2] / 4096) * 57.296;                          //Calculate the roll angle.


    if (!first_angle) {                                                                 //When this is the first time.
      angle_pitch = angle_pitch_acc;                                                    //Set the pitch angle to the accelerometer angle.
      angle_roll = angle_roll_acc;                                                      //Set the roll angle to the accelerometer angle.
      first_angle = true;
    }
    else {                                                                              //When this is not the first time.
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                       //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    }

    //We can't print all the data at once. This takes to long and the angular readings will be off.
    if (loop_counter == 0)Serial.print("Pitch: ");
    if (loop_counter == 1)Serial.print(angle_pitch , 1);
    if (loop_counter == 2)Serial.print(" Roll: ");
    if (loop_counter == 3)Serial.print(angle_roll , 1);
    if (loop_counter == 4)Serial.print(" Yaw: ");
    if (loop_counter == 5)Serial.print(gyro_axis[3] / 65.5 , 0);
    if (loop_counter == 6)Serial.print(" Temp: ");
    if (loop_counter == 7)Serial.println(temperature / 340.0 + 35.0 , 1);
    loop_counter ++;
    if (loop_counter == 60)loop_counter = 0;

    while (loop_timer > get_cpu_usecs());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  print_intro();                                                                        //Print the intro to the serial monitor.
}

void check_barometer(void) {
  loop_counter = 0;

    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++) {
      C[start] = i2c_read(MS5611_address, 0xA0 + start*2) << 8 | i2c_read(MS5611_address, 0xA0 + start*2 +1);                    //Start communication with the MPU-6050.
    }
    //Print the 6 calibration values on the screen.
    Serial.print("C1 = ");
    Serial.println(C[1]);
    Serial.print("C2 = ");
    Serial.println(C[2]);
    Serial.print("C3 = ");
    Serial.println(C[3]);
    Serial.print("C4 = ");
    Serial.println(C[4]);
    Serial.print("C5 = ");
    Serial.println(C[5]);
    Serial.print("C6 = ");
    Serial.println(C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

    start = 0;

  while (data != 'q') {                                           //Stay in this loop until the data variable data holds a q.
    loop_timer = get_cpu_usecs() + 4000;                                 //Set the loop_timer variable to the current micros() value + 4000.

    barometer_counter ++;                                         //Increment the barometer_counter variable for the next step.

    if (barometer_counter == 1) {
      if (temperature_counter == 0) {
        //Get temperature data from MS-5611
        raw_temperature = i2c_read(MS5611_address, 0x58) << 16 | i2c_read(MS5611_address, 0x58 + 1) << 8 | i2c_read(MS5611_address, 0x58 + 2);
      }
      else {
        //Get pressure data from MS-5611
        raw_pressure = i2c_read(MS5611_address, 0x48) << 16 | i2c_read(MS5611_address, 0x48 + 1) << 8 | i2c_read(MS5611_address, 0x48 + 2);
      }

      temperature_counter ++;
      if (temperature_counter > 9) {
        temperature_counter = 0;
        //Request temperature data
        i2c_write(MS5611_address, 0x58, 0x00);
      }
      else {
        //Request pressure data
        i2c_write(MS5611_address, 0x48, 0x00);
      }
    }
    if (barometer_counter == 2) {
      //Calculate pressure as explained in the datasheet of the MS-5611.
      dT = C[5];
      dT <<= 8;
      dT *= -1;
      dT += raw_temperature;

      OFF = OFF_C2 + ((int)dT * (int)C[4]) / pow(2, 7);

      SENS = SENS_C1 + ((int)dT * (int)C[3]) / pow(2, 8);

      P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

      if (actual_pressure == 0) {
        actual_pressure = P;
        actual_pressure_fast = P;
        actual_pressure_slow = P;
      }

      actual_pressure_fast = actual_pressure_fast * (float)0.92 + P * (float)0.08;
      actual_pressure_slow = actual_pressure_slow * (float)0.99 + P * (float)0.01;
      actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
      if (actual_pressure_diff > 8)actual_pressure_diff = 8;
      if (actual_pressure_diff < -8)actual_pressure_diff = -8;
      if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
      actual_pressure = actual_pressure_slow;
      if (start < 200){
        start++;
        actual_pressure = 0;
      }
      else printf("%f \n", actual_pressure);
    }
    if (barometer_counter == 3) {
      barometer_counter = 0;
    }
    while (loop_timer > get_cpu_usecs());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  start = 0;
  print_intro();                                                                        //Print the intro to the serial monitor.
}

int main(int argc, char **argv)
{
  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.

  set_gyro_registers();

  print_intro();                                                //Print the intro on the serial monitor.

  for (int j=0;j<10;j++)
  {
    // millis(10);

    // if (Serial.available() > 0) {
      // data = Serial.read();                                       //Read the incomming byte.
      // millis(100);                                                 //Wait for any other bytes to come in.
      // while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer.
      disable_throttle = 1;                                       //Set the throttle to 1000us to disable the motors.
    // }

    if (!disable_throttle) {                                      //If the throttle is not disabled.
      actuator_write(m1, channel_3);
      actuator_write(m2, channel_3);
      actuator_write(m3, channel_3);
      actuator_write(m4, channel_3);
    }
    else {                                                        //If the throttle is disabled
      actuator_write(m1, 1000);
      actuator_write(m2, 1000);
      actuator_write(m3, 1000);
      actuator_write(m4, 1000);
    }

    if (data == 'a') {
      printf("Reading receiver input pulses.\n");
      printf("You can exit by sending a q (quit).\n");
      millis(2500);
      reading_receiver_signals();
    }
    if (data == 'c') {
      printf("Reading raw gyro data.\n");
      printf("You can exit by sending a q (quit).\n");
      read_gyro_values();
    }

    if (data == 'd') {
      printf("Reading the raw accelerometer data.\n");
      printf("You can exit by sending a q (quit).\n");
      millis(2500);
      read_gyro_values();
    }

    if (data == 'e') {
      printf("Reading the IMU angles.\n");
      printf("You can exit by sending a q (quit).\n");
      check_imu_angles();
    }

    if (data == 'g') {
      printf("Reading the battery voltage.\n");
      printf("You can exit by sending a q (quit).\n");
      check_battery_voltage();
    }

    if (data == 'h') {
      printf("Checking MS-5611 barometer.\n");
      printf("You can exit by sending a q (quit).\n");
      millis(2500);
      check_barometer();
    }

    // if (data == 'i') {
    //   printf("Checking raw GPS data.\n");
    //   check_gps();
    // }

    // if (data == 'j') {
    //   printf("Checking HMC5883L compass.\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_compass();
    // }

    // if (data == '1') {
    //   printf("Check motor 1 (front right, counter clockwise direction).\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_motor_vibrations();
    // }

    // if (data == '2') {
    //   printf("Check motor 2 (rear right, clockwise direction).\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_motor_vibrations();
    // }

    // if (data == '3') {
    //   printf("Check motor 3 (rear left, counter clockwise direction).\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_motor_vibrations();
    // }

    // if (data == '4') {
    //   printf("Check motor 4 (front lefft, clockwise direction).\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_motor_vibrations();
    // }

    // if (data == '5') {
    //   printf("Check motor all motors.\n");
    //   printf("You can exit by sending a q (quit).\n");
    //   millis(2500);
    //   check_motor_vibrations();
    // }
  }
  return 0;
}
