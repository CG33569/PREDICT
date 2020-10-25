#include "gps.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)
#define LED (*((volatile _IODEV unsigned *)PATMOS_IO_LED))
#define MOTOR ( ( volatile _IODEV unsigned * )	PATMOS_IO_ACT+0x10 )
#define m1 0  // top-right
#define m2 1  // bottom-right
#define m3 2  // bottom-left
#define m4 3  // top-left
#define BATTERY ( ( volatile _IODEV unsigned * )  PATMOS_IO_AUDIO )
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )
// Change this in case of barometer
#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )
#define MPU6050_I2C_ADDRESS 0x68
//IMU sensor MCU6050 registers
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
// ============================= Functions hearders ============================
//-----> Basic usage:
void micros(int microseconds);
void millis(int milliseconds);
int uart2_write(unsigned char data);
int uart2_read(unsigned char *data);
void actuator_write(unsigned int actuator_id, unsigned int data);
int receiver_read(unsigned int receiver_id);
int i2c_write(unsigned char chipaddress, unsigned char regaddress, unsigned char data);
int i2c_read(unsigned char chipaddress, unsigned char regaddress);
//-----> Components handling:
void send_telemtry(char *xstr);
void receive_telemtry(char *Outstr);
/*
void print_tpv_value(const char *name, const char *format, const int32_t value, const int32_t scale_factor);
void print_gps(char*str,struct gps_tpv* tpv);
void check_gps(struct gps_tpv* tpv);
*/
