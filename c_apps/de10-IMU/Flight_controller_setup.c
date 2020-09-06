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

// int temperature=0;
// double acc_x=0.0, acc_y=0.0, acc_z=0.0, acc_total_vector=0.0;
// short int acc_axis[4]={0,0,0,0}, gyro_axis[4]={0,0,0,0};
// double gyro_pitch=0.0, gyro_roll=0.0, gyro_yaw=0.0;
// int cal_int=0, loop_counter=0;
// double gyro_axis_cal[4]={0.0,0.0,0.0,0.0};
// int gyro_address = MPU6050_I2C_ADDRESS;

unsigned int  last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned int  highByte, lowByte, type, error, clockspeed_ok;
unsigned int channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
int roll_axis, pitch_axis, yaw_axis;
unsigned int receiver_check_byte, gyro_check_byte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
int address, cal_int;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

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
  // exc_prologue();

  // read the receiver pwm duty cycle
  receiver_input[1] = receiver_read(0);
  receiver_input[2] = receiver_read(1);
  receiver_input[3] = receiver_read(2);
  receiver_input[4] = receiver_read(3);

  // exc_epilogue();
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

  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  //Read the MPU-6050
  ACCEL_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
  ACCEL_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_L);
  ACCEL_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
  ACCEL_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_L);
  ACCEL_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
  ACCEL_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_L);
  TEMP_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
  TEMP_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_L);
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
  gyro_roll = gyro_axis[1];           //Set gyro_roll to the correct axis that was stored in the EEPROM.
  gyro_pitch = gyro_axis[2];          //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  gyro_pitch *= -1;              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[3];            //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  gyro_yaw *= -1;                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.


  acc_x = acc_axis[2];                //Set acc_x to the correct axis that was stored in the EEPROM.
  acc_x *= -1;                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[1];                //Set acc_y to the correct axis that was stored in the EEPROM.
  acc_z = acc_axis[3];                //Set acc_z to the correct axis that was stored in the EEPROM.
  acc_z *= -1;                   //Invert acc_z if the MSB of EEPROM bit 30 is set.

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
  LED_out(1);                                                               //Turn on the warning led.

  set_gyro_registers();

  // for (cal_int = 0; cal_int < 1250 ; cal_int ++)
  // {                                                                         //Wait 5 seconds before continuing.
    actuator_write(m1, 1000);                                               //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);
    // micros(3000);  
  // }

  // //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  // for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
  //   if(cal_int % 15 == 0)LED_out(1);                //Change the led status to indicate calibration.
  //   gyro_signalen();                                                        //Read the gyro output.
  //   gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
  //   gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
  //   gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
  //   //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
  //   actuator_write(m1, 1000);                                               //give motors 1000us pulse.
  //   actuator_write(m2, 1000);
  //   actuator_write(m3, 1000);
  //   actuator_write(m4, 1000);
  //   // micros(3000);                                                                 //Wait 3 milliseconds before the next loop.
  // }
  // //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  // gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  // gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  // gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  // loop_timer = get_cpu_usecs();                                                    //Set the timer for the next loop.

  // //When everything is done, turn off the led.
  // LED_out(0);                                                     //Turn off the warning led.

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Main program loop
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for (int j=0;j<10000;j++)
  // while(1)
  // {
    intr_handler();
    micros(3000000);
    printf("\n");
    printf("===================================================\n");
    printf("Gyro calibration\n");
    printf("===================================================\n");
    printf("Don't move the quadcopter!! Calibration starts in 3 seconds\n");
    micros(3000000);
    printf("Calibrating the gyro, this will take +/- 8 seconds\n");
    printf("Please wait");
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++)
    {              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
      gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
      gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
      
      //Show the calibration results
      printf("Axis 1 offset=%f\n",gyro_roll_cal);
      printf("Axis 2 offset=%f\n",gyro_pitch_cal);
      printf("Axis 3 offset=%f\n",gyro_yaw_cal);
      
      printf("===================================================\n");
      printf("Gyro axes configuration\n");
      printf("===================================================\n");
      
    // if(get_cpu_usecs() - loop_timer > 4050)LED_out(1);                   //Turn on the LED if the loop time exceeds 4050us.
    
    //All the information for controlling the motor's is available.
    //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    // while(get_cpu_usecs() - loop_timer < 4000);                                      //We wait until 4000us are passed.\
    
    // loop_timer = get_cpu_usecs();                                                    //Set the timer for the next loop.

    // printf("diff loop_timer:  %ld\n", loop_timer);

    //esc pwm write
    // actuator_write(m1, esc_1);
    // actuator_write(m2, esc_2);
    // actuator_write(m3, esc_3);
    // actuator_write(m4, esc_4);
    
    //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
    //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
    gyro_signalen();

  // }
  return 0;
}

