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


///////////initialization

int temperature=0;
float acc_x=0.0, acc_y=0.0, acc_z=0.0, acc_total_vector=0.0;
int acc_axis[4]={0,0,0,0}, gyro_axis[4]={0,0,0,0};
double gyro_pitch=0.0, gyro_roll=0.0, gyro_yaw=0.0;
int cal_int=0, loop_counter=0;
double gyro_axis_cal[4]={0.0,0.0,0.0,0.0};
int gyro_address = MPU6050_I2C_ADDRESS;
bool first_angle=false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 1.3;               //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;              //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 18.0;              //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

bool auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int  last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned int  eeprom_data[36];
unsigned int  highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int start;
int receiver_input[5];
float roll_level_adjust, pitch_level_adjust;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input=0.0, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input=0.0, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input=0.0, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
bool gyro_angles_set;

unsigned int signature = 0;
unsigned int ACCEL_X_H = 0;
unsigned int ACCEL_X_L = 0;
unsigned int ACCEL_Y_H = 0;
unsigned int ACCEL_Y_L = 0;
unsigned int ACCEL_Z_H = 0;
unsigned int ACCEL_Z_L = 0;
unsigned int TEMP_L = 0;
unsigned int TEMP_H = 0;
unsigned int GYRO_X_H = 0;
unsigned int GYRO_X_L = 0;
unsigned int GYRO_Y_H = 0;
unsigned int GYRO_Y_L = 0;
unsigned int GYRO_Z_H = 0;
unsigned int GYRO_Z_L = 0;

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
}

void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
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
void intr_handler(void) {
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
  i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);                    //Set the register bits as 00000000 to activate the gyro
  i2c_write(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08);                   //Set the register bits as 00001000 (500dps full scale)
  i2c_write(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  i2c_write(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03);                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid()
{
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(unsigned int  function)
{
  unsigned int  channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  if(function==1)reverse = 1;                      //Reverse channel when most significant bit is set
  else if(function==2)reverse = 0;
  else if(function==3)reverse = 0;
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = 1000;  //Store the low value for the specific receiver input channel
  center = 1500; //Store the center value for the specific receiver input channel
  high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void gyro_signalen()
{

  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  //Read the MPU-6050
  ACCEL_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
  ACCEL_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
  ACCEL_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_L);
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

  acc_axis[1] = (float)(ACCEL_X_H<<8|ACCEL_X_L);                    //Add the low and high byte to the acc_x variable.
  acc_axis[2] = (float)(ACCEL_Y_H<<8|ACCEL_Y_L);                  //Add the low and high byte to the acc_y variable.
  acc_axis[3] = (float)(ACCEL_Z_H<<8|ACCEL_Z_L);                    //Add the low and high byte to the acc_z variable.
  temperature = (float)(TEMP_H<<8|TEMP_L);                    //Add the low and high byte to the temperature variable.
  gyro_axis[1] = (float)(GYRO_X_H<<8|GYRO_X_L);                   //Read high and low part of the angular data.
  gyro_axis[2] = (float)(GYRO_Y_H<<8|GYRO_Y_L);                   //Read high and low part of the angular data.
  gyro_axis[3] = (float)(GYRO_Z_H<<8|GYRO_Z_L);                   //Read high and low part of the angular data.

  if(cal_int == 2000)
  {
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[1];           //Set gyro_roll to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;               //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[2];          //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[3];            //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[1];                //Set acc_x to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[2];                //Set acc_y to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                   //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[3];                //Set acc_z to the correct axis that was stored in the EEPROM.
  //  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                   //Invert acc_z if the MSB of EEPROM bit 30 is set.


   //   printf("-----------------------\n");
   // printf("ACCEL_X = 0x%.2X%.2X (%d)\n", ACCEL_X_H, ACCEL_X_L, (short int)((ACCEL_X_H << 8) | ACCEL_X_L));
   // printf("ACCEL_Y = 0x%.2X%.2X (%d)\n", ACCEL_Y_H, ACCEL_Y_L, (short int)((ACCEL_Y_H << 8) | ACCEL_Y_L));
   // printf("ACCEL_Z = 0x%.2X%.2X (%d)\n", ACCEL_Z_H, ACCEL_Z_L, (short int)((ACCEL_Z_H << 8) | ACCEL_Z_L));
   // printf("TEMP    = 0x%.2X%.2X (%.1f C)\n", TEMP_H, TEMP_L, ((double)((short int)((TEMP_H << 8) | TEMP_L)) + 12412.0) / 340.0 ); //using datasheet formula for T in degrees Celsius
   // printf("GYRO_X  = 0x%.2X%.2X (%d)\n", GYRO_X_H, GYRO_X_L, (short int)((GYRO_X_H << 8) | GYRO_X_L));
   // printf("GYRO_Y  = 0x%.2X%.2X (%d)\n", GYRO_Y_H, GYRO_Y_L, (short int)((GYRO_Y_H << 8) | GYRO_Y_L));
   // printf("GYRO_Z  = 0x%.2X%.2X (%d)\n", GYRO_Z_H, GYRO_Z_L, (short int)((GYRO_Z_H << 8) | GYRO_Z_L));
}

int main(int argc, char **argv)
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  start =0;
  LED_out(1);                                                               //Turn on the warning led.

  set_gyro_registers();

  for (cal_int = 0; cal_int < 1250 ; cal_int ++)
  {                                                                         //Wait 5 seconds before continuing.
    actuator_write(m1, 1000);                                               //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);
    micros(3000);  
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    if(cal_int % 15 == 0)LED_out(1);                //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    actuator_write(m1, 1000);                                               //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);
    micros(3000);                                                                 //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  //interrpt declaration
  // register exception handler
  exc_register(14, &intr_handler);
  exc_register(16, &intr_handler);
  exc_register(18, &intr_handler);
  exc_register(20, &intr_handler);

  // unmask interrupts
  intr_unmask_all();
  // clear pending flags
  intr_clear_all_pending();
  // enable interrupts
  intr_enable();


  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)
  {
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.

    actuator_write(m1, 1000);                                               //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);
    micros(3000);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      LED_out(1);                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (batteryRead() + 65) * 1.2317;

  loop_timer = get_cpu_usecs();                                                    //Set the timer for the next loop.

  //When everything is done, turn off the led.
  LED_out(0);                                                     //Turn off the warning led.

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Main program loop
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  for (int j=0;j<10;j++)
  {

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //This is the added IMU code from the videos:
    //https://youtu.be/4BoIE8YQwM8
    //https://youtu.be/j-kE0AMEWy4
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch -= angle_roll * (float)sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * (float)sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
    
    if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
    }
    if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
    }
    
    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
    angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
    
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
    roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

    if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
      pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
      roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
    }


    //For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
      start = 2;

      angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
      angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
      gyro_angles_set = true;                                                 //Set the IMU started flag.

      //Reset the PID controllers for a bumpless start.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    }
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
    else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

    pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
      if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
      else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
    }
    
    calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (batteryRead() + 65) * 0.09853;

    //Turn on the led if battery voltage is to low.
    if(battery_voltage < 1000 && battery_voltage > 600)LED_out(1);


    throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

    if (start == 2){                                                          //The motors are started.
      if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

      if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
        esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
      } 

      if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
      if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
      if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
      if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

      if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
      if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
      if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
      if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
    }

    else{
      esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
      esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
      esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
      esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating the pulses for the ESC's is explained in this video:
    //https://youtu.be/fqEkVcqxtU8
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    //Because of the angle calculation the loop time is getting very important. If the loop time is 
    //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
    //that the loop time is still 4000us and no longer! More information can be found on 
    //the Q&A page: 
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
      
    if(get_cpu_usecs() - loop_timer > 4050)LED_out(1);                   //Turn on the LED if the loop time exceeds 4050us.
    
    //All the information for controlling the motor's is available.
    //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    while(get_cpu_usecs() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = get_cpu_usecs();                                                    //Set the timer for the next loop.

    //esc pwm write
    actuator_write(m1, esc_1);
    actuator_write(m2, esc_2);
    actuator_write(m3, esc_3);
    actuator_write(m4, esc_4);
    
    //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
    //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
    gyro_signalen();

  }
  return 0;
}

