
// #include <Wire.h>               //Include the Wire.h library so we can communicate with the gyro
#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
#include <machine/patmos.h>
//#include "libcorethread/corethread.h"
//#include "libmp/mp.h"

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

//Declaring Global Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte lowByte, highByte, type, gyro_address, error, clockspeed_ok;
byte channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
byte roll_axis, pitch_axis, yaw_axis;
byte receiver_check_byte, gyro_check_byte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
int address, cal_int;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

//Writes to i2c, returns -1 if there was an error, 0 if succeded
int i2c_write(unsigned char chipaddress, unsigned char regaddress, unsigned char data){
  I2C = ((((unsigned int) data & 0x000000FF) << 16) | (((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) & 0xFFFFFFFE;
  if ((I2C & 0x00000100) != 0)
  {
    return -1;
  }else{
    return 0;
  }
}

//Reads to i2c, returns the read value (8 bits), if there was an error the returned value is -1 (0xFFFFFFFF)
int i2c_read(unsigned char chipaddress, unsigned char regaddress){
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
void blink_once(){
  int i, j;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0001;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0000;
  return;
}

//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  // Wire.beginTransmission(gyro_address);
  // Wire.write(who_am_i);
  // Wire.endTransmission();
  // Wire.requestFrom(gyro_address, 1);
  // timer = millis() + 100;
  // while(Wire.available() < 1 && timer > millis());
  // lowByte = Wire.read();
  // address = gyro_address;
  // return lowByte;
  signature = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I);
  printf("Signature = 0x%.2X\n", signature);

  if(signature) lowByte=1;
  else lowByte=0;

  address = gyro_address;
  return lowByte;
}

void start_gyro(){

  //Setup the MPU-6050

    // Wire.beginTransmission(address);                             //Start communication with the gyro
    // Wire.write(0x6B);                                            //PWR_MGMT_1 register
    // Wire.write(0x00);                                            //Set to zero to turn on the gyro
    // Wire.endTransmission();                                      //End the transmission
    
    // Wire.beginTransmission(address);                             //Start communication with the gyro
    // Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    // Wire.endTransmission();                                      //End the transmission
    // Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    // while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    // printf("Register 0x6B is set to:");
    // printf(Wire.read(),BIN);
    // printf("\n");
    
    // Wire.beginTransmission(address);                             //Start communication with the gyro
    // Wire.write(0x1B);                                            //GYRO_CONFIG register
    // Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    // Wire.endTransmission();                                      //End the transmission
    
    // Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    // Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    // Wire.endTransmission();                                      //End the transmission
    // Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    // while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    // printf("Register 0x1B is set to:");
    // printf(Wire.read(),BIN);
    // printf("\n");

  printf("PWR_MGMT_1 = 0x%.2X\n", i2c_read(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1));
  printf("Getting MPU-6050 out of sleep mode.\n");
  i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
  printf("PWR_MGMT_1 = 0x%.2X\n", i2c_read(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1));

}

void gyro_signalen(){

    // Wire.beginTransmission(address);                             //Start communication with the gyro
    // Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    // Wire.endTransmission();                                      //End the transmission
    // Wire.requestFrom(address,6);                                 //Request 6 bytes from the gyro
    // while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    // gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    // if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    // gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    // if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    // gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    // if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration

  GYRO_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
  GYRO_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_L);
  GYRO_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
  GYRO_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_L);
  GYRO_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
  GYRO_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_L);
  printf("-----------------------\n");
  printf("GYRO_X  = 0x%.2X%.2X (%d)\n", GYRO_X_H, GYRO_X_L, (short int)((GYRO_X_H << 8) | GYRO_X_L));
  printf("GYRO_Y  = 0x%.2X%.2X (%d)\n", GYRO_Y_H, GYRO_Y_L, (short int)((GYRO_Y_H << 8) | GYRO_Y_L));
  printf("GYRO_Z  = 0x%.2X%.2X (%d)\n", GYRO_Z_H, GYRO_Z_L, (short int)((GYRO_Z_H << 8) | GYRO_Z_L));

  gyro_roll=GYRO_X_H<<8|GYRO_X_L;                        //Read high and low part of the angular data
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
  gyro_pitch=GYRO_Y_H<<8|GYRO_Y_L;                       //Read high and low part of the angular data
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
  gyro_yaw=GYRO_Z_H<<8|GYRO_Z_L;                         //Read high and low part of the angular data
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration


}

//Check if a receiver input value is changing within 30 seconds
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    delay(250);
    if(receiver_input_channel_1 > 1750 || receiver_input_channel_1 < 1250){
      trigger = 1;
      receiver_check_byte |= 0b00000001;
      pulse_length = receiver_input_channel_1;
    }
    if(receiver_input_channel_2 > 1750 || receiver_input_channel_2 < 1250){
      trigger = 2;
      receiver_check_byte |= 0b00000010;
      pulse_length = receiver_input_channel_2;
    }
    if(receiver_input_channel_3 > 1750 || receiver_input_channel_3 < 1250){
      trigger = 3;
      receiver_check_byte |= 0b00000100;
      pulse_length = receiver_input_channel_3;
    }
    if(receiver_input_channel_4 > 1750 || receiver_input_channel_4 < 1250){
      trigger = 4;
      receiver_check_byte |= 0b00001000;
      pulse_length = receiver_input_channel_4;
    } 
  }
  if(trigger == 0){
    error = 1;
    printf("No stick movement detected in the last 30 seconds!!! (ERROR 2)");
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      channel_3_assign = trigger;
      if(pulse_length < 1250)channel_3_assign += 0b10000000;
    }
    if(movement == 2){
      channel_1_assign = trigger;
      if(pulse_length < 1250)channel_1_assign += 0b10000000;
    }
    if(movement == 3){
      channel_2_assign = trigger;
      if(pulse_length < 1250)channel_2_assign += 0b10000000;
    }
    if(movement == 4){
      channel_4_assign = trigger;
      if(pulse_length < 1250)channel_4_assign += 0b10000000;
    }
  }
}

void check_to_continue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(channel_2_assign == 0b00000001 && receiver_input_channel_1 > center_channel_1 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000001 && receiver_input_channel_1 < center_channel_1 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000010 && receiver_input_channel_2 > center_channel_2 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000010 && receiver_input_channel_2 < center_channel_2 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000011 && receiver_input_channel_3 > center_channel_3 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000011 && receiver_input_channel_3 < center_channel_3 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000100 && receiver_input_channel_4 > center_channel_4 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000100 && receiver_input_channel_4 < center_channel_4 - 150)continue_byte = 1;
    delay(100);
  }
  wait_sticks_zero();
}

//Check if the transmitter sticks are in the neutral position
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    delay(100);
  }
}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15){
    if(receiver_input_channel_1 < 2100 && receiver_input_channel_1 > 900)zero |= 0b00000001;
    if(receiver_input_channel_2 < 2100 && receiver_input_channel_2 > 900)zero |= 0b00000010;
    if(receiver_input_channel_3 < 2100 && receiver_input_channel_3 > 900)zero |= 0b00000100;
    if(receiver_input_channel_4 < 2100 && receiver_input_channel_4 > 900)zero |= 0b00001000;
    delay(500);
    printf(".");
  }
  if(zero == 0){
    error = 1;
    printf(".");
    printf("No valid receiver signals found!!! (ERROR 1)");
  }
  else printf(" OK");
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void register_min_max(){
  byte zero = 0;
  low_channel_1 = receiver_input_channel_1;
  low_channel_2 = receiver_input_channel_2;
  low_channel_3 = receiver_input_channel_3;
  low_channel_4 = receiver_input_channel_4;
  while(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)delay(250);
  printf("Measuring endpoints....");
  while(zero < 15){
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    if(receiver_input_channel_1 < low_channel_1)low_channel_1 = receiver_input_channel_1;
    if(receiver_input_channel_2 < low_channel_2)low_channel_2 = receiver_input_channel_2;
    if(receiver_input_channel_3 < low_channel_3)low_channel_3 = receiver_input_channel_3;
    if(receiver_input_channel_4 < low_channel_4)low_channel_4 = receiver_input_channel_4;
    if(receiver_input_channel_1 > high_channel_1)high_channel_1 = receiver_input_channel_1;
    if(receiver_input_channel_2 > high_channel_2)high_channel_2 = receiver_input_channel_2;
    if(receiver_input_channel_3 > high_channel_3)high_channel_3 = receiver_input_channel_3;
    if(receiver_input_channel_4 > high_channel_4)high_channel_4 = receiver_input_channel_4;
    delay(100);
  }
}

//Check if the angular position of a gyro axis is changing within 10 seconds
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyro_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.00007;
      gyro_angle_yaw += gyro_yaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyro_roll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.0000611;
      gyro_angle_yaw += gyro_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    printf("No angular motion is detected in the last 10 seconds!!! (ERROR 4)");
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

//Intro subroutine
void intro(){
  printf("===================================================");
  delay(1500);
  printf("");
  printf("Your");
  delay(500);
  printf("  Multicopter");
  delay(500);
  printf("    Flight");
  delay(500);
  printf("      Controller");
  delay(1000);
  printf("===================================================");
}

int main(int argc, char **argv)
{
    // pinMode(12, OUTPUT);
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan (OGT_INT0)/////change later to alterra interrupt pins
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change
  // Wire.begin();             //Start the I2C as master
  delay(250);               //Give the gyro time to start 

  // printf("Hello MCU6050!\n");
  blink_once();

  unsigned int signature = 0;
  // unsigned int ACCEL_X_H = 0;
  // unsigned int ACCEL_X_L = 0;
  // unsigned int ACCEL_Y_H = 0;
  // unsigned int ACCEL_Y_L = 0;
  // unsigned int ACCEL_Z_H = 0;
  // unsigned int ACCEL_Z_L = 0;
  // unsigned int TEMP_L = 0;
  // unsigned int TEMP_H = 0;
  unsigned int GYRO_X_H = 0;
  unsigned int GYRO_X_L = 0;
  unsigned int GYRO_Y_H = 0;
  unsigned int GYRO_Y_L = 0;
  unsigned int GYRO_Z_H = 0;
  unsigned int GYRO_Z_L = 0;

//Main program
for(;;){
  //Show the YMFC-3D V2 intro
  intro();
  
  printf("");
  printf("===================================================");
  printf("System check");
  printf("===================================================");
  delay(1000);
  printf("Checking I2C clock speed.");
  delay(1000);
  
  TWBR = 12;                      //Set the I2C clock speed to 400kHz. /////change later for atlterra clock speed
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(TWBR == 12 && clockspeed_ok){
    printf("I2C clock speed is correctly set to 400kHz.");
  }
  else{
    printf("I2C clock speed is not set to 400kHz. (ERROR 8)");
    error = 1;
  }
  
  if(error == 0){
    printf("");
    printf("===================================================");
    printf("Transmitter setup");
    printf("===================================================");
    delay(1000);
    printf("Checking for valid receiver signals.");
    //Wait 10 seconds until all receiver inputs are valid
    wait_for_receiver();
    printf("");
  }
  //Quit the program in case of an error
  if(error == 0){
    delay(2000);
    printf("Place all sticks and subtrims in the center position within 10 seconds.");
    for(int i = 9;i > 0;i--){
      delay(1000);
      printf(i);
      printf(" ");
    }
    printf(" \n");
    //Store the central stick positions
    center_channel_1 = receiver_input_channel_1;
    center_channel_2 = receiver_input_channel_2;
    center_channel_3 = receiver_input_channel_3;
    center_channel_4 = receiver_input_channel_4;
    printf("");
    printf("Center positions stored.");
    printf("Digital input 08 = ");  ////change pins to alterra's
    printf(receiver_input_channel_1);
    printf("\n");
    printf("Digital input 09 = ");  ////change pins to alterra's
    printf(receiver_input_channel_2);
    printf("Digital input 10 = ");  ////change pins to alterra's
    printf(receiver_input_channel_3);
    printf("Digital input 11 = ");  ////change pins to alterra's
    printf(receiver_input_channel_4);
    printf("\n");
    printf("");
    printf("");
  }
  if(error == 0){  
    printf("Move the throttle stick to full throttle and back to center");
    //Check for throttle movement
    check_receiver_inputs(1);
    printf("Throttle is connected to digital input ");
    printf((channel_3_assign & 0b00000111) + 7);
    printf("\n");
    if(channel_3_assign & 0b10000000)printf("Channel inverted = yes");
    else printf("Channel inverted = no");
    wait_sticks_zero();
    
    printf("");
    printf("");
    printf("Move the roll stick to simulate left wing up and back to center");
    //Check for throttle movement
    check_receiver_inputs(2);
    printf("Roll is connected to digital input ");
    printf((channel_1_assign & 0b00000111) + 7);
    printf("\n");
    if(channel_1_assign & 0b10000000)printf("Channel inverted = yes");
    else printf("Channel inverted = no");
    wait_sticks_zero();
  }
  if(error == 0){
    printf("");
    printf("");
    printf("Move the pitch stick to simulate nose up and back to center");
    //Check for throttle movement
    check_receiver_inputs(3);
    printf("Pitch is connected to digital input ");
    printf((channel_2_assign & 0b00000111) + 7);
    printf("\n");
    if(channel_2_assign & 0b10000000)printf("Channel inverted = yes");
    else printf("Channel inverted = no");
    wait_sticks_zero();
  }
  if(error == 0){
    printf("");
    printf("");
    printf("Move the yaw stick to simulate nose right and back to center");
    //Check for throttle movement
    check_receiver_inputs(4);
    printf("Yaw is connected to digital input ");
    printf((channel_4_assign & 0b00000111) + 7);
    printf("\n");
    if(channel_4_assign & 0b10000000)printf("Channel inverted = yes");
    else printf("Channel inverted = no");
    wait_sticks_zero();
  }
  if(error == 0){
    printf("");
    printf("");
    printf("Gently move all the sticks simultaneously to their extends");
    printf("When ready put the sticks back in their center positions");
    //Register the min and max values of the receiver channels
    register_min_max();
    printf("");
    printf("");
    printf("High, low and center values found during setup");
    printf("Digital input 08 values:");
    printf(low_channel_1);
    printf(" - ");
    printf(center_channel_1);
    printf(" - ");
    printf(high_channel_1);
    printf("\n");
    printf("Digital input 09 values:");
    printf(low_channel_2);
    printf(" - ");
    printf(center_channel_2);
    printf(" - ");
    printf(high_channel_2);
    print("\n");
    printf("Digital input 10 values:");
    printf(low_channel_3);
    printf(" - ");
    printf(center_channel_3);
    printf(" - ");
    printf(high_channel_3);
    printf("\n");
    printf("Digital input 11 values:");
    printf(low_channel_4);
    printf(" - ");
    printf(center_channel_4);
    printf(" - ");
    printf(high_channel_4);
    printf("\n");
    printf("Move stick 'nose up' and back to center to continue");
    check_to_continue();
  }
    
  if(error == 0){
    //What gyro is connected
    printf("");
    printf("===================================================");
    printf("Gyro search");
    printf("===================================================");
    delay(2000);
    
    printf("Searching for MPU-6050 on address 0x68/104");
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      printf("MPU-6050 found on address 0x68");
      type = 1;
      gyro_address = 0x68;
    }
    
    if(type == 0){
      printf("Searching for MPU-6050 on address 0x69/105");
      delay(1000);
      if(search_gyro(0x69, 0x75) == 0x68){
        printf("MPU-6050 found on address 0x69");
        type = 1;
        gyro_address = 0x69;
      }
    }
    
//    
//    if(type == 0){
//      printf("Searching for L3G4200D on address 0x68/104"));
//      delay(1000);
//      if(search_gyro(0x68, 0x0F) == 0xD3){
//        printf("L3G4200D found on address 0x68"));
//        type = 2;
//        gyro_address = 0x68;
//      }
//    }
//    
//    if(type == 0){
//      printf("Searching for L3G4200D on address 0x69/105"));
//      delay(1000);
//      if(search_gyro(0x69, 0x0F) == 0xD3){
//        printf("L3G4200D found on address 0x69"));
//        type = 2;
//        gyro_address = 0x69;
//      }
//    }
//    
//    if(type == 0){
//      printf("Searching for L3GD20H on address 0x6A/106"));
//      delay(1000);
//      if(search_gyro(0x6A, 0x0F) == 0xD7){
//        printf("L3GD20H found on address 0x6A"));
//        type = 3;
//        gyro_address = 0x6A;
//      }
//    }
//    
//    if(type == 0){
//     printf("Searching for L3GD20H on address 0x6B/107"));
//      delay(1000);
//      if(search_gyro(0x6B, 0x0F) == 0xD7){
//        printf("L3GD20H found on address 0x6B"));
//        type = 3;
//        gyro_address = 0x6B;
//      }
//    }
//    
    if(type == 0){
      printf("No gyro device found!!! (ERROR 3)");
      error = 1;
    }
    
    else{
      delay(3000);
      printf("");
      printf("===================================================");
      printf("Gyro register settings");
      printf("===================================================");
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    printf("");
    printf("===================================================");
    printf("Gyro calibration");
    printf("===================================================");
    printf("Don't move the quadcopter!! Calibration starts in 3 seconds");
    delay(3000);
    printf("Calibrating the gyro, this will take +/- 8 seconds");
    printf("Please wait");
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
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
    printf("");
    printf("Axis 1 offset=");
    printf(gyro_roll_cal);
    printf("\n");
    printf("Axis 2 offset=");
    printf(gyro_pitch_cal);
    printf("\n");
    printf("Axis 3 offset=");
    printf(gyro_yaw_cal);
    printf("\n");
    printf("");
    
    printf("===================================================");
    printf("Gyro axes configuration");
    printf("===================================================");
    
    //Detect the left wing up movement
    printf("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds");
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      printf("OK!");
      printf("Angle detection = ");
      printf(roll_axis & 0b00000011);
      printf("\n");
      if(roll_axis & 0b10000000)printf("Axis inverted = yes");
      else printf("Axis inverted = no");
      printf("Put the quadcopter back in its original position");
      printf("Move stick 'nose up' and back to center to continue");
      check_to_continue();

      //Detect the nose up movement
      printf("");
      printf("");
      printf("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds");
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      printf("OK!");
      printf("Angle detection = ");
      printf(pitch_axis & 0b00000011);
      printf("\n");
      if(pitch_axis & 0b10000000)printf("Axis inverted = yes");
      else printf("Axis inverted = no");
      printf("Put the quadcopter back in its original position");
      printf("Move stick 'nose up' and back to center to continue");
      check_to_continue();
      
      //Detect the nose right movement
      printf("");
      printf("");
      printf("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds");
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      printf("OK!");
      printf("Angle detection = ");
      printf(yaw_axis & 0b00000011);
      printf("\n");
      if(yaw_axis & 0b10000000)printf("Axis inverted = yes");
      else printf("Axis inverted = no");
      printf("Put the quadcopter back in its original position");
      printf("Move stick 'nose up' and back to center to continue");
      check_to_continue();
    }
  }
  if(error == 0){
    printf("");
    printf("===================================================");
    printf("LED test");
    printf("===================================================");
    digitalWrite(12, HIGH); ///change the digital writes  ///PORTB |= B10000
    printf("The LED should now be lit");
    printf("Move stick 'nose up' and back to center to continue");
    check_to_continue();
    digitalWrite(12, LOW); ///PORTB |= B00000
  }
  
  printf("");
  
  if(error == 0){
    printf("===================================================");
    printf("Final setup check");
    printf("===================================================");
    delay(1000);
    if(receiver_check_byte == 0b00001111){
      printf("Receiver channels ok");
    }
    else{
      printf("Receiver channel verification failed!!! (ERROR 6)");
      error = 1;
    }
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      printf("Gyro axes ok");
    }
    else{
      printf("Gyro exes verification failed!!! (ERROR 7)");
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    printf("");
    printf("===================================================");
    printf("Storing EEPROM information");
    printf("===================================================");
    printf("Writing EEPROM");
    delay(1000);
    printf("Done!");
    EEPROM.write(0, center_channel_1 & 0b11111111);
    EEPROM.write(1, center_channel_1 >> 8);
    EEPROM.write(2, center_channel_2 & 0b11111111);
    EEPROM.write(3, center_channel_2 >> 8);
    EEPROM.write(4, center_channel_3 & 0b11111111);
    EEPROM.write(5, center_channel_3 >> 8);
    EEPROM.write(6, center_channel_4 & 0b11111111);
    EEPROM.write(7, center_channel_4 >> 8);
    EEPROM.write(8, high_channel_1 & 0b11111111);
    EEPROM.write(9, high_channel_1 >> 8);
    EEPROM.write(10, high_channel_2 & 0b11111111);
    EEPROM.write(11, high_channel_2 >> 8);
    EEPROM.write(12, high_channel_3 & 0b11111111);
    EEPROM.write(13, high_channel_3 >> 8);
    EEPROM.write(14, high_channel_4 & 0b11111111);
    EEPROM.write(15, high_channel_4 >> 8);
    EEPROM.write(16, low_channel_1 & 0b11111111);
    EEPROM.write(17, low_channel_1 >> 8);
    EEPROM.write(18, low_channel_2 & 0b11111111);
    EEPROM.write(19, low_channel_2 >> 8);
    EEPROM.write(20, low_channel_3 & 0b11111111);
    EEPROM.write(21, low_channel_3 >> 8);
    EEPROM.write(22, low_channel_4 & 0b11111111);
    EEPROM.write(23, low_channel_4 >> 8);
    EEPROM.write(24, channel_1_assign);
    EEPROM.write(25, channel_2_assign);
    EEPROM.write(26, channel_3_assign);
    EEPROM.write(27, channel_4_assign);
    EEPROM.write(28, roll_axis);
    EEPROM.write(29, pitch_axis);
    EEPROM.write(30, yaw_axis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyro_address);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    printf("Verify EEPROM data");
    delay(1000);
    if(center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_1_assign != EEPROM.read(24))error = 1;
    if(channel_2_assign != EEPROM.read(25))error = 1;
    if(channel_3_assign != EEPROM.read(26))error = 1;
    if(channel_4_assign != EEPROM.read(27))error = 1;
    
    if(roll_axis != EEPROM.read(28))error = 1;
    if(pitch_axis != EEPROM.read(29))error = 1;
    if(yaw_axis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyro_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)printf("EEPROM verification failed!!! (ERROR 5)");
    else printf("Verification done");
  }
  
  
  if(error == 0){
    printf("Setup is finished.");
    printf("You can now calibrate the esc's and upload the YMFC-AL code.");
  }
  else{
   printf("The setup is aborted due to an error.");
   printf("Check the Q and A page of the YMFC-AL project on:");
   printf("www.brokking.net for more information about this error.");
  }
  while(1);
}
return 0;
}


