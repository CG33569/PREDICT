//
// Created by mark on 02/22/21.
//

#ifndef PATMOS_BARO_H
#define PATMOS_BARO_H

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../basic_lib/PREDICTBasic.h"

/*
//Barometer v2 variables
#define BARO_REG        0xA0     // Register address for coeffifient
#define MS5611_ADDR     0x77
#define CMD_ADC_D1      0x48     // Pressure - 4096 resolution
#define CMD_ADC_D2      0x58     // Temperature - 4096 resolution
*/

#define TRUE            1        
#define FALSE           0
#define BARO_REG        0xA0     // Register address for coeffifient
#define MS5611_ADDR     0x77
#define CMD_ADC_D1      0x48     // Pressure - 4096 resolution
#define CMD_ADC_D2      0x58     // Temperature - 4096 resolution

unsigned long offi;
unsigned long sensi;
unsigned long Ti;
unsigned long BaroMeterCoff[6];
unsigned int data[3];


char altitude_band = 8; // The dead band between the slow and fast reacting altitude measurements
//float rot_val = 5;     // Length of the moving average / rotating value

unsigned long pressure_rot_mem[5];
unsigned char pressure_rot_mem_loc;

__int32_t pressure_total_avarage;
float actual_pressure_fast;
float actual_pressure_slow;
float actual_pressure_diff;


/*
 * Function: millis
 * ----------------------------
 *  Waits for the given amount of ms
 *
 *  ms : amount of milliseconds to wait for
 *  
 *  Returns: None
 */
// void millis(int ms)
// {
//   unsigned int timer_ms = (get_cpu_usecs()/1000);
//   unsigned int loop_timer = timer_ms;
//   while(timer_ms - loop_timer < ms)timer_ms = (get_cpu_usecs()/1000);
// }

/*
 * Function: barometer_reset
 * ----------------------------
 *   Sends a reset command
 *    "Shall be sent once after power-on"
 *
 *  Returns: None
 */
// void barometer_reset(void)
// {
//   // Reset the barometer
//   i2c_reg8_write8(MS5611_ADDR, 0x1E, (int)NULL);
//   millis(5);
// }


/*
 * Function: barometer_adc
 * ----------------------------
 *   Starts the ADC conversion and reads the value
 *    the waiting time depends on the conversion resolution
 *   
 *   ADDR: the memory address (command) for the specific conversion
 *         Possible commands are descibed in the spec sheet of the MS5611
 *
 *  Returns: The converted (digital) pressure or temperature value
 */
unsigned long barometer_adc(char ADDR)
{
  i2c_reg8_write8(MS5611_ADDR, ADDR, (int)NULL);
  millis(10);
  unsigned long reading = i2c_reg8_read24b(MS5611_ADDR, 0x00);
  return reading;
}

/*
 * Function: barometer_main
 * ----------------------------
 *  Combines the above functions
 *    the waiting time depends on the conversion resolution
 *   
 *   ADDR: the memory address (command) for the specific conversion
 *         Possible commands are descibed in the spec sheet of the MS5611
 *
 *  Returns: None
 */
int barometer_pressure(void)
{
  // printf("\t\t TEMP TIMER : %llu\n",get_cpu_usecs() - temp_timer );
  // tempp = i2c_reg8_read24b(MS5611_ADDR, 0x00);
  if(baro_setup){   
    ptemp = barometer_adc(CMD_ADC_D1);
    tempp = barometer_adc(CMD_ADC_D2);
    }
  else{
        tempp = i2c_reg8_read24b(MS5611_ADDR, 0x00);
  }
  //ptemp = barometer_adc(CMD_ADC_D1);
  //tempp = barometer_adc(CMD_ADC_D2);
  // }
  // else{
  //   ptemp = 0;
  //   temp = 0;
  // }
  //
  // ----------- Calculate temperature -----------
  //
  // Calculate difference between actual and reference temperature
  unsigned long dT = tempp - ((BaroMeterCoff[4] * 256));
  // Actual temperature 
  // 20 degrees + (temp_difference * temperature sensitivity)
  tempp = 2000 + (dT * (BaroMeterCoff[5] / pow(2, 23)));
  //
  // ----------- Calculate temperature compensated pressure -----------
  //
  // Offset at actual temperature
  unsigned long long off = BaroMeterCoff[1] * 65536 + (BaroMeterCoff[3] * dT) / 128;
  // Sensitivity at actual temperature
  unsigned long long sens = BaroMeterCoff[0] * 32768 + (BaroMeterCoff[2] * dT) / 256;
  //
  // ----------- Second order temperature compensation -----------
  //
  if(tempp < 2000)
  {
    Ti = (dT * dT) / (pow(2,31));
    offi = 5 * ((pow((tempp - 2000), 2))) / 2;
    sensi =  5 * ((pow((tempp - 2000), 2))) / 4; 
    
    if(tempp < -1500)
    {
       offi = offi + 7 * ((pow((tempp + 1500), 2)));      
       sensi = sensi + 11 * ((pow((tempp + 1500), 2))) / 2;
    }
  }
   
  tempp -= Ti;
  off -= offi;
  sens -= sensi;
  //
  // ----------- Pressure calculation -----------
  //
  ptemp = (ptemp * sens / 2097152 - off);
  ptemp /= 32768;


  //
  // ----------- Add values to rotating memory -----------
  //
  if(baro_setup == false){
    pressure_total_avarage -= pressure_rot_mem[pressure_rot_mem_loc];
    pressure_rot_mem[pressure_rot_mem_loc] = ptemp;
    pressure_total_avarage += pressure_rot_mem[pressure_rot_mem_loc];
    pressure_rot_mem_loc++;
    if (pressure_rot_mem_loc == (int)rot_val)pressure_rot_mem_loc = 0;
    actual_pressure_fast = (float)pressure_total_avarage / rot_val;


    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;

    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > altitude_band) actual_pressure_diff = altitude_band;
    if (actual_pressure_diff < -altitude_band) actual_pressure_diff = -altitude_band;
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
    //
    // ----------- PID Gains for the altitude hold mode -----------
    //                                                                                   //Set the barometer counter to 0 for the next measurements.
        //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
        //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
        if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
        parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
        parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
        parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
        pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
        parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
        if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

        if (flight_mode >= 2) {                                                          //If the quadcopter is in altitude mode and flying.
            if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
            //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
            //will indicate if the altitude of the quadcopter is changed by the pilot.
            manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
            manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
            
            if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
                manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
            }
            if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
                manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
            }
            //Calculate the PID output of the altitude hold.
            pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
            float pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

            //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
            //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
            float pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
            if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
                pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
                if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
            }

            //In the following section the I-output is calculated. It's an accumulation of errors over time.
            //The time factor is removed as the program loop runs at 250Hz.
            pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
            if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
            else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
            //In the following line the PID-output is calculated.
            //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
            //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
            //D = pid_d_gain_altitude * parachute_throttle.
            
            pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
            //To prevent extreme PID-output the output must be limited.
            
            if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
            else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
        }

            //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
        else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
            pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
            pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
            pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
            manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
            manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
        }
    pthread_mutex_lock(&mutex);
    actual_pressure +=0;
    pid_altitude_setpoint += 0;                                                     //Reset the PID altitude setpoint.
    pid_output_altitude += 0;                                                       //Reset the output of the PID controller.
    pid_i_mem_altitude += 0;                                                        //Reset the I-controller.
    manual_throttle += 0;                                                           //Set the manual_throttle variable to 0 .
    manual_altitude_change += 0;                                                    //Set the manual_altitude_change to 1.
    pthread_mutex_unlock(&mutex);
    return actual_pressure;
  }
  return ptemp;


// ----------- Printing functions for comments -----------
//

  // printf("Temperature in Celsius : %f            ",ctemp);
  // printf(" Pressure : %f ",pressure);
  // printf(" mbar \n"); 
}

/*
 * Function: barometer_setup
 * ----------------------------
 *   Reads the calibration data from PROM
 *      C1 : Pressure sensitivity
 *      C2 : Pressure offset
 *      C3 : Temp coeff. for pressure sensitivity
 *      C4 : Temp. coeff. for pressure offset
 *      C5 : Reference temperature
 *      C6 : Temperature coeffifient for temperature
 *
 *  Returns: None
 */
void barometer_setup(void)
{
  pthread_mutex_lock(&mutex);
  printf("Barometer callibration and setup...\n");
  pthread_mutex_unlock(&mutex);
  // Read the coefficients (only done once)
  for (int i = 0; i < 6; i++) {
      BaroMeterCoff[i] = i2c_reg8_read16b(MS5611_ADDR, 0xA2 + i*2);
    }

  // Set up the rotating memory and the basis for the complementary filter
  for (int i=0; i < rot_val; i++){
    pressure_rot_mem[i] = barometer_pressure();
    pressure_total_avarage += pressure_rot_mem[i];
  }
  actual_pressure_fast = (float)pressure_total_avarage / rot_val;
  actual_pressure_slow = actual_pressure_fast;
    pthread_mutex_lock(&mutex);
    printf("... barometer callibration done: %f\n", actual_pressure_slow);
    pthread_mutex_unlock(&mutex);
  baro_setup = false;
}

#endif //PATMOS_BARO_H
