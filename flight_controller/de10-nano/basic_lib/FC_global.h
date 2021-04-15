//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_FLIGHT_CONTEROLLER_V2_H
#define PATMOS_FLIGHT_CONTEROLLER_V2_H

//standard header files
#include <stdio.h>
#include <stdlib.h>

float dt =0.02; /// loop timer in secs
bool motor_publish = false;///publish motor commands
#define battery_voltage_available false// battery_voltage input from the fpga to compensate the esc input for change in battery volatge(will be later provided by DTU)
#define GYRO_CALLIB 1 //set to 1 to swtich on gyro callibration before flight
#define COMP_CALLIB 0 //set to 1 to swtich on gyro callibration before flight
#define LEVEL_CALLIB 0 //set to 1 to swtich on gyro callibration before flight

//channel 1- roll
// channel 2 - pitch
// channel 3- throttle
// channel 4- yaw

///motor configuration
// m4    ^     m1
//  \    |    /
//   \   |   /
//    \  |  /
//     \   /
//      \ /
//       /\
//      /  \
//     /    \
//    /      \
//   /        \
//m3/          \m2

#define PRINT_COMMANDS false  // set to true to print sensor data from other threads
float pitch_offset, roll_offset =0;
int acc_count =0;
bool first_time=true;  //these variables are use to fix the initial offset error in the pitch and yaw angles

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.35;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0002;             //Gain setting for the roll I-controller
float pid_d_gain_roll = 8;                  //Gain setting for the roll D-controller
int pid_max_roll = 400;                     //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 0.35;              //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.0002;            //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 8;                 //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1;                   //Gain setting for the pitch P-controller. 
float pid_i_gain_yaw = 0.002;               //Gain setting for the pitch I-controller. 
float pid_d_gain_yaw = 0.0;                 //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                      //Maximum output of the PID-controller (+/-)


//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;

float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

float declination = 0.0;                   //Set the declination between the magnetic and geographic north.

__int16_t manual_takeoff_throttle = 1500;    //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).

__uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
__uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
__uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
bool program_off=false;
__uint8_t start;
__uint8_t flight_mode;
__uint8_t takeoff_detected, manual_altitude_change;
__uint8_t level_calibration_on;

__int16_t esc_1, esc_2, esc_3, esc_4;
__int16_t manual_throttle;
__int16_t throttle, takeoff_throttle, cal_int;
__int16_t acc_x, acc_y, acc_z;
__int16_t gyro_pitch, gyro_roll, gyro_yaw;

int reverse_channel[4]={0,1,0,0};///(th,roll,pitch,yaw)  for transmitter 1
/* ===== Rahul's transmitter ===== */
int low[4]={1000,1000,1000,1090}, center[4]={1495,1500,1500,1530}, high[4]={2000,2000,2000,2000};///(th,roll,pitch,yaw)  for transmitter 1
/* ===== Mark's transmitter ===== */
// int low[4]={1003,1000,1000,1002}, center[4]={1507,1500,1498,1500}, high[4]={1985,2000,1990,1998};///(th,roll,pitch,yaw)  for transmitter 2

__int32_t channel_1;
__int32_t channel_2;
__int32_t channel_3;
__int32_t channel_4;
__int32_t channel_5;
__int32_t channel_6;// 6 channels of transmitter


///gyro variables
__int32_t acc_total_vector, acc_total_vector_at_start;
__int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
__int16_t acc_pitch_cal_value;
__int16_t acc_roll_cal_value;

__int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
__int16_t acc_z_average_short[26], acc_z_average_long[51];
__uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
__int32_t acc_alt_integrated;

// gyro PID variables
float roll_level_adjust, pitch_level_adjust;
float pid_i_mem_roll, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;

//Compass variables
__uint8_t compass_calibration_on, heading_lock;
__int16_t compass_x, compass_y, compass_z;
__int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
__int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;


//Pressure variables.
__int64_t temp_timer = 0;
__int64_t ptemp;
__int64_t tempp;
bool baro_setup = true;
__uint16_t C[7];
__uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
__int64_t OFF_C2, SENS, SENS_C1, P;
__uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
char rot_val = 5;   // Length of the moving average or rotating memory
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
__int32_t dT, dT_C5;

//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
__uint8_t parachute_rotating_mem_location;
__int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
__int32_t pressure_rotating_mem[50], pressure_total_avarage;
__uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

//GPS variables
__uint8_t number_used_sats;
__uint8_t waypoint_set ;
__int32_t l_lat_gps, l_lon_gps;
__int32_t l_lat_waypoint, l_lon_waypoint;
float  gps_pitch_adjust, gps_roll_adjust;
float l_lon_gps_float_adjust, l_lat_gps_float_adjust,gps_man_adjust_heading;
float return_to_home_move_factor;
__uint8_t home_point_recorded;
__int32_t lat_gps_home, lon_gps_home;

short int acc_axis[4], gyro_axis[4];



#endif //PATMOS_FLIGHT_CONTEROLLER_V2_H
