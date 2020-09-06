
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>
// #include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
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



/* Kalman filter variables */
double X_Q_angle = 0.001, Y_Q_angle = 0.001; // Process noise variance for the accelerometer
double X_Q_bias = 0.003, Y_Q_bias = 0.003; // Process noise variance for the gyro bias
double X_R_measure = 0.03, Y_R_measure = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise

double X_angle = 0, Y_angle = 0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double X_bias = 0, Y_bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double X_rate, Y_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double X_P[2][2]={{0,0},{0,0}} , Y_P[2][2]={{0,0},{0,0}}; // Error covariance matrix - This is a 2x2 matrix
double X_K[2], Y_K[2]; // Kalman gain - This is a 2x1 vector
double X_y, Y_y; // Angle difference
double X_S , Y_S; // Estimate error

/* IMU Data */
double accX=0, accY=0, accZ=0;
double gyroX=0, gyroY=0, gyroZ=0;
int tempRaw=0;

double gyroXangle=0, gyroYangle=0; // Angle calculate using the gyro only
double compAngleX=0, compAngleY=0; // Calculated angle using a complementary filter
double kalAngleX=0, kalAngleY=0; // Calculated angle using a Kalman filter

double timer;

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


double getAngle(double newAngle, double newRate, double dt, char axis) 
{
  // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
  // Modified by Kristian Lauszus
  // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  if(axis=='x')
  {
    X_rate = newRate - X_bias;
    X_angle += dt * X_rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    X_P[0][0] += dt * (dt*X_P[1][1] - X_P[0][1] - X_P[1][0] + X_Q_angle);
    X_P[0][1] -= dt * X_P[1][1];
    X_P[1][0] -= dt * X_P[1][1];
    X_P[1][1] += X_Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    X_S = X_P[0][0] + X_R_measure;
    /* Step 5 */
    X_K[0] = X_P[0][0] / X_S;
    X_K[1] = X_P[1][0] / X_S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    X_y = newAngle - X_angle;
    /* Step 6 */
    X_angle += X_K[0] * X_y;
    X_bias += X_K[1] * X_y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    X_P[0][0] -= X_K[0] * X_P[0][0];
    X_P[0][1] -= X_K[0] * X_P[0][1];
    X_P[1][0] -= X_K[1] * X_P[0][0];
    X_P[1][1] -= X_K[1] * X_P[0][1];

    return X_angle;
  }
  else
  {
    Y_rate = newRate - Y_bias;
    Y_angle += dt * Y_rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Y_P[0][0] += dt * (dt*Y_P[1][1] - Y_P[0][1] - Y_P[1][0] + Y_Q_angle);
    Y_P[0][1] -= dt * Y_P[1][1];
    Y_P[1][0] -= dt * Y_P[1][1];
    Y_P[1][1] += Y_Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    Y_S = Y_P[0][0] + Y_R_measure;
    /* Step 5 */
    Y_K[0] = Y_P[0][0] / Y_S;
    Y_K[1] = Y_P[1][0] / Y_S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    Y_y = newAngle - Y_angle;
    /* Step 6 */
    Y_angle += Y_K[0] * Y_y;
    Y_bias += Y_K[1] * Y_y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    Y_P[0][0] -= Y_K[0] * Y_P[0][0];
    Y_P[0][1] -= Y_K[0] * Y_P[0][1];
    Y_P[1][0] -= Y_K[1] * Y_P[0][0];
    Y_P[1][1] -= Y_K[1] * Y_P[0][1];

    return Y_angle;
  }
}
void setAngle(double newAngle, char axis)
{ 
  if(axis=='x')X_angle = newAngle;
  else Y_angle = newAngle;
} // Used to set angle, this should be set as the starting angle
double getRate(char axis)
{ 
  if(axis=='x')return X_rate; 
  else return Y_rate;
  
} // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(double newQ_angle,char axis) 
{ 
  if(axis=='x') X_Q_angle = newQ_angle; 
  else Y_Q_angle = newQ_angle; 
  
}
void setQbias(double newQ_bias, char axis) 
{ 
  if(axis=='x') X_Q_bias = newQ_bias;
  else Y_Q_bias = newQ_bias;
   
}
void setRmeasure(double newR_measure,char axis) 
{
  if(axis=='x') X_R_measure = newR_measure;
  else Y_R_measure = newR_measure;
}

double getQangle(char axis)
{ 
  if(axis=='x')return X_Q_angle;  
  else return Y_Q_angle;
}
double getQbias(char axis)
{ 
  if(axis=='x')return X_Q_bias; 
  else return Y_Q_bias;
}
double getRmeasure(char axis)
{ 
  if(axis=='x')return X_R_measure; 
  else return Y_R_measure;
}

void set_gyro_registers()
{
  //Setup the MPU-6050
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x01));                    //Set the register bits as 00000000 to activate the gyro
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x00));                   //Set the register bits as 00001000 (500dps full scale)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00));                  //Set the register bits as 00010000 (+/- 8g full scale range)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x00));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
}

void gyro_signalen()
{

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

  accX = (float)(ACCEL_X_H<<8|ACCEL_X_L);                    //Add the low and high byte to the acc_x variable.
  accY = (float)(ACCEL_Y_H<<8|ACCEL_Y_L);                  //Add the low and high byte to the acc_y variable.
  accZ = (float)(ACCEL_Z_H<<8|ACCEL_Z_L);                    //Add the low and high byte to the acc_z variable.
  tempRaw = (float)(TEMP_H<<8|TEMP_L);                    //Add the low and high byte to the temperature variable.
  gyroX = (float)(GYRO_X_H<<8|GYRO_X_L);                   //Read high and low part of the angular data.
  gyroY = (float)(GYRO_Y_H<<8|GYRO_Y_L);                   //Read high and low part of the angular data.
  gyroZ = (float)(GYRO_Z_H<<8|GYRO_Z_L);                   //Read high and low part of the angular data.
}

int main(int argc, char **argv)
{

  set_gyro_registers();
  micros(100*1000);


  gyro_signalen(); 

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  setAngle(roll, 'x'); // Set starting angle
  setAngle(pitch,'y');
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = get_cpu_usecs();                                                    //Set the timer for the next loop.

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Main program loop
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for (int j=0;j<1000;j++)
  while(1)
  {

  /* Update all the values */
  gyro_signalen(); 

  double dt = (double)(get_cpu_usecs() - timer) / 1000000; // Calculate delta time
  timer = get_cpu_usecs();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      setAngle(roll, 'x');
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = getAngle(roll, gyroXrate, dt,'x'); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = getAngle(pitch, gyroYrate, dt,'y');
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      setAngle(pitch,'y');
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = getAngle(pitch, gyroYrate, dt,'y'); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = getAngle(roll, gyroXrate, dt,'x'); // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    /* Print Data */
  #if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
  #endif


  printf("roll: %f       gyroXangle: %f      compAngleX: %f     kalAngleX: %f     \n", roll, gyroXangle, compAngleX,kalAngleX);
  // printf("pitch: %f       gyroYangle: %f      compAngleY: %f     kalAngleY: %f     \n", pitch, gyroYangle, compAngleY,kalAngleY);
  // printf("roll: %f   kalAngleX: %f     \n", roll,kalAngleX);
  // printf("pitch: %f  kalAngleY: %f     \n", pitch,kalAngleY);


  #if 0 // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
  #endif
    micros(1000000);
}
return 0;
}