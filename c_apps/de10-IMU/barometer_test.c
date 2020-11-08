#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>

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

//I2C controller
#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )
#define BARO_REG                   0xA0   // R
#define MS5611_ADDRESS            0x77

unsigned long loop_timer;
int loop_counter;

//Barometer variables.
unsigned int C[7];
unsigned int data[2];
unsigned int barometer_counter, temperature_counter;
int OFF, OFF_C2, SENS, SENS_C1, P;
unsigned int raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int dT, dT_C5;
unsigned int start;

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

void check_barometer(void) {
  loop_counter = 0;

    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++)
    {
      data[0], data[1] = i2c_read(MS5611_ADDRESS, 0xA0 + start*2);
      //data[1] = i2c_read(MS5611_ADDRESS, 0xA0 + start*2+1);
      C[start] = data[0]  << 8 | data[1] ;                    //Start communication with the MPU-6050.
      
      printf("C%d, data[%d] = %X\n",start, 0, data[0]);
      printf("C%d, data[%d] = %X\n",start, 1, data[1]);
      printf("-----------\n");
    }
    //Print the 6 calibration values on the screen.
    printf("C1 = %d\n",C[1]);
    printf("C2 = %d\n",C[2]);
    printf("C3 = %d\n",C[3]);
    printf("C4 = %d\n",C[4]);
    printf("C5 = %d\n",C[5]);
    printf("C6 = %d\n",C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

    start = 0;

  for(int i=0;i<30;i++) {                                           //Stay in this loop until the data variable data holds a q.
    loop_timer = get_cpu_usecs() + 4000;                                 //Set the loop_timer variable to the current micros() value + 4000.

    barometer_counter ++;                                         //Increment the barometer_counter variable for the next step.

    if (barometer_counter == 1) {
      if (temperature_counter == 0) {
        //Get temperature data from MS-5611
        raw_temperature = i2c_read(MS5611_ADDRESS, 0x58) << 16 | i2c_read(MS5611_ADDRESS, 0x58 + 1) << 8 | i2c_read(MS5611_ADDRESS, 0x58 + 2);
      }
      else {
        //Get pressure data from MS-5611
        raw_pressure = i2c_read(MS5611_ADDRESS, 0x48) << 16 | i2c_read(MS5611_ADDRESS, 0x48 + 1) << 8 | i2c_read(MS5611_ADDRESS, 0x48 + 2);
      }

      temperature_counter ++;
      if (temperature_counter > 9) {
        temperature_counter = 0;
        //Request temperature data
        i2c_write(MS5611_ADDRESS, 0x58, 0x00);
      }
      else {
        //Request pressure data
        i2c_write(MS5611_ADDRESS, 0x48, 0x00);
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
      // else printf("%f \n", actual_pressure);
    }
    if (barometer_counter == 3) {
      barometer_counter = 0;
    }
    while (loop_timer > get_cpu_usecs());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  start = 0;
}

int main(int argc, char **argv)
{
  printf("Hello Baro!\n");
  
  check_barometer();
  return 0;
}
  
