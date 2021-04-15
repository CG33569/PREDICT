#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../basic_lib/i2c_master.h"


//Barometer variables.
uint16_t C[7];
int loop_counter, loop_timer,start=0;
uint8_t barometer_counter, temperature_counter;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;


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
//LEDs
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )


#define BARO_REG                   0xA0   // R

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//Barometer v2 variables
#define MS5611_ADDR 0x77

unsigned long Coff[6], Ti = 0, offi = 0, sensi = 0;
unsigned int data[3];
//////////////

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

void LED_out(int i){
  if(i==1) LED = 0x0001;
  else LED = 0x0000;
  return;
}

void check_barometer(void) {
    loop_counter = 0;

        //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
        //These 2 byte values are stored in the memory location 0xA2 and up.
        for (start = 1; start <= 6; start++) {
            C[start] = i2c_reg8_read16b(MS5611_ADDR,0xA0 + start * 2);                //Add the low and high byte to the C[x] calibration variable.
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

    while(1){                                           //Stay in this loop until the data variable data holds a q.
        loop_timer = get_cpu_usecs() + 20000;                                 //Set the loop_timer variable to the current micros() value + 4000.

        barometer_counter ++;                                         //Increment the barometer_counter variable for the next step.

        if (barometer_counter == 1) {
            if (temperature_counter == 0) {
                //Get temperature data from MS-5611
                raw_temperature = i2c_reg8_read24b(MS5611_ADDR, 0x00);
            }
            else {
                //Get pressure data from MS-5611
                raw_pressure = i2c_reg8_read24b(MS5611_ADDR, 0x00);
            }

            temperature_counter ++;
            if (temperature_counter > 9) {
                temperature_counter = 0;
                //Request temperature data
                i2c_reg8_write8_empty(MS5611_ADDR, 0x58);
            }
            else {
                //Request pressure data
                i2c_reg8_write8_empty(MS5611_ADDR, 0x48);
            }
        }
        if (barometer_counter == 2) {
            //Calculate pressure as explained in the datasheet of the MS-5611.
            dT = C[5];
            dT <<= 8;
            dT *= -1;
            dT += raw_temperature;

            OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);

            SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);

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
            else printf("Pressure : %f \n", actual_pressure);
        }
        if (barometer_counter == 3) {
            barometer_counter = 0;
        }
        loop_counter++;
        if(loop_counter>10000)
            break;
        while (loop_timer > get_cpu_usecs());
    }
    loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
    start = 0;
}

int main(int argc, char **argv)
{
  printf("Hello Baro!\n");
  
  ///////baromter v2
    check_barometer();
  return 0;
}
  
