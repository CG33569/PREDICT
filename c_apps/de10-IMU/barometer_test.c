#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>

#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )		//I2C controller
#define MS5611_I2C_ADDRESS 0x77         								//The I2C address of the MS5611 barometer

int loop_counter;
unsigned long loop_timer;

//Barometer variables.
unsigned long C[7], T2 = 0, OFF2 = 0, SENS2 = 0;
unsigned int D[3];
unsigned int barometer_counter, temperature_counter;
unsigned int T_REF;
unsigned long long OFF, OFF_T1, SENS, SENS_T1;
unsigned long D1, D2;
float temp, pressure;
unsigned long dT, dT_C5;
unsigned int start;


void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}

//Writes to i2c, returns -1 if there was an error, 0 if succeded
int i2c_write(unsigned char chipaddress, unsigned char regaddress, unsigned char data){

  I2C = ((((unsigned int) data & 0x000000FF) << 16) | (((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) & 0xFFFFFFFE;
  if ((I2C & 0x00000100) != 0) return -1;
  else return 0;
}


//Reads to i2c, returns the read value (8 bits), if there was an error the returned value is -1 (0xFFFFFFFF)
int i2c_read(unsigned char chipaddress, unsigned char regaddress){
  I2C = ((((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) | 0x00000001; 			// 0x00000001 in the end tells to read
  unsigned int I2C_tmp = I2C;
  if ((I2C_tmp & 0x00000100) != 0) return -1;
  else return (int)((unsigned int)(I2C_tmp) & 0x000000FF);
}


void check_barometer(void) {

	// Polling calibration values
    for (start = 1; start <= 6; start++)
    { // 
      C[start] = i2c_read(MS5611_I2C_ADDRESS, 0xA0 + start*2) << 8 | i2c_read(MS5611_I2C_ADDRESS, 0xA0 + start*2);                    //Start communication with the MPU-6050.
    }
    //Print the 6 calibration values on the screen.
    printf("C1 = %lu\n",C[1]);			// Pressure sensitivity	(SENS_T1)
    printf("C2 = %lu\n",C[2]);			// Pressure offset		(OFF_T1)
    printf("C3 = %lu\n",C[3]);			// Temp. coeff of pressure sensitivity (TCS)
    printf("C4 = %lu\n",C[4]);			// Temp. coeff of presure offset	(TCO)
    printf("C5 = %lu\n",C[5]);			// Reference temp.		(T_REF)
    printf("C6 = %lu\n",C[6]);			// Temp. coeff of the temp. (TEMPSENS)

    // These conversions are described in the data sheet
    OFF_T1 = C[2] * pow(2,16);                                   
    SENS_T1 = C[1] * pow(2,15);                                  
    T_REF = C[5]*pow(2,8);

  for (int i=0; i<500; i++){

  	// Convert D1
  	i2c_write(MS5611_I2C_ADDRESS, 0x40, 0x00);		// Start a conversion command
  	millis(10);										// Wait 10 ms (9.04 max conversion time for 4096 OSR)
  	D1 = i2c_read(MS5611_I2C_ADDRESS, 0x00) << 16 | i2c_read(MS5611_I2C_ADDRESS, 0x00) << 8 | i2c_read(MS5611_I2C_ADDRESS, 0x00);
	
  	// Convert D2
  	i2c_write(MS5611_I2C_ADDRESS, 0x50, 0x00);
  	millis(10);										// Wait 10 ms (9.04 max conversion time for 4096 OSR)
  	D2 = i2c_read(MS5611_I2C_ADDRESS, 0x00) << 16 | i2c_read(MS5611_I2C_ADDRESS, 0x00) << 8 | i2c_read(MS5611_I2C_ADDRESS, 0x00);
  	
  	// Calculate temperature
  	dT = D2 - T_REF;
  	temp = (2000 + (dT * C[6])/pow(2,23));

  	// Calculate temperature compensated pressure
 	OFF = OFF_T1 + (dT*C[4])/pow(2,7);
 	SENS = SENS_T1 + (dT*C[3])/pow(2,7);


 	// Second-order compensation
 	if (temp < 2000){
 		T2 = (dT * dT) / pow(2,31);
 		OFF2 = 5 * pow((temp-2000),2) / 2;
 		SENS2 = 5 * pow((temp-2000),2) / 4;
 		if (temp < -1500){
 			OFF2 = OFF2 + 7 * pow((temp+1500),2);
 			SENS2 = SENS2 + 11 * pow((temp+1500),2) / 2;
 		}
 	}

 	temp -= T2;
 	OFF -= OFF2;
 	SENS -= SENS2;
 	
 	pressure = (((D1*SENS)/pow(2,21)-OFF)/pow(2,15));

 	float P = pressure / 100.0;
 	float T = temp / 100.0;
 	// Print result
	printf("Temp = %.2f°C,\tPressure = %.2f mbar\n",T, P);

  }
}


int main(int argc, char **argv)
{
  printf("Hello Baro!\n");
  check_barometer();
  return 0;
}
  
