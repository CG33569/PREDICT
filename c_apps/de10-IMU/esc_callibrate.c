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

#define battery_voltage_available 0
 //channel 1- roll
// channel 2 - pitch
// channel 3- throttle
// channel 4- yaw
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

const unsigned int CPU_PERIOD = 20; //CPU period in ns.


///////////initialization

int temperature=0;
double acc_x=0.0, acc_y=0.0, acc_z=0.0, acc_total_vector=0.0;
short int acc_axis[4]={0,0,0,0}, gyro_axis[4]={0,0,0,0};
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
int low[5]={0,1068,1100,1108,1068}, center[5]={0,1488,1504,1504,1468}, high[5]={0,1892,1908,1904,1864};
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
bool new_function_request=true;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long zero_timer;

void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}

//Reads from propulsion specified by propulsion ID (0 to 4)
int receiver_read(unsigned int receiver_id){

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
// interrupt handler
void intr_handler(void) {
  // exc_prologue();

  // read the receiver pwm duty cycle
  receiver_input[1] = receiver_read(0);
  receiver_input[2] = receiver_read(1);
  receiver_input[3] = receiver_read(2);
  receiver_input[4] = receiver_read(3);

  // exc_epilogue();
}

void wait_for_receiver(){
  int zero = 1;                                                                //Set all bits in the variable zero to 0
  while(zero){
    intr_handler();                                                             //Stay in this loop until the 4 lowest bits are set
    if(receiver_input[1] < 2100 && receiver_input[1] > 900 && 
    receiver_input[2] < 2100 && receiver_input[2] > 900 && 
    receiver_input[3] < 2100 && receiver_input[3] > 900 && 
    receiver_input[4] < 2100 && receiver_input[4] > 900)zero=0;  //Set bit 3 if the receiver pulse 4 is within the 900 - 2100 range
    micros(500000);                                                                 //Wait 500 milliseconds
  }
}

void LED_out(int i){
  if(i==1) LED = 0x0001;
  else LED = 0x0000;
  return;
}


//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel, reverse;                                                       //First we declare some local variables
  int actual; ///(0,th,roll,pitch,yaw)
  int difference;

  if(function==1)
  {
    reverse = 0;                      //Reverse channel when most significant bit is set
    channel =2;
  }
  else if(function==2)
  {
    reverse = 0;
    channel=3;
  }
  else if(function==3)
  {
    reverse = 0;
    channel=1;
  }
  else
  {
    reverse = 0;                                                            //If the most significant is not set there is no reverse
    channel =4;
  }

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  // low = 1000;  //Store the low value for the specific receiver input channel
  // center = 1500; //Store the center value for the specific receiver input channel
  // high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center[channel]){                                                         //The actual receiver value is lower than the center value
    if(actual < low[channel])actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center[channel]){                                                                        //The actual receiver value is higher than the center value
    if(actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}


int main(int argc, char **argv)
{
  wait_for_receiver();                                                                  //Wait until the receiver is active.
  zero_timer = get_cpu_usecs(); 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
while(1)
{
  while(zero_timer + 4000 > get_cpu_usecs());                                                  //Start the pulse after 4000 micro seconds.
  zero_timer = get_cpu_usecs();
  printf("zero_timer:%ld\n",zero_timer );

  intr_handler();

    receiver_input_channel_3 = convert_receiver_channel(3);                               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  if(receiver_input_channel_3 < 1025)new_function_request = false;                      //If the throttle is in the lowest position set the request flag to false.


  if(new_function_request==false)
  {
    receiver_input_channel_3 = convert_receiver_channel(3);                             //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    esc_1 = receiver_input_channel_3;                                                   //Set the pulse for motor 1 equal to the throttle channel.
    esc_2 = receiver_input_channel_3;                                                   //Set the pulse for motor 2 equal to the throttle channel.
    esc_3 = receiver_input_channel_3;                                                   //Set the pulse for motor 3 equal to the throttle channel.
    esc_4 = receiver_input_channel_3;                                                   //Set the pulse for motor 4 equal to the throttle channel.
  }

    //esc pwm write
  actuator_write(m1, esc_1);
  actuator_write(m2, esc_2);
  actuator_write(m3, esc_3);
  actuator_write(m4, esc_4);
    

  }
  return 0;
}

