#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)
#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )
//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3
//Receiver controller
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

const unsigned int CPU_PERIOD = 20; //CPU period in ns.
#define compass_address 0x1E

unsigned char gps_data=0;
float compass_x, compass_y, compass_z;
int loop_counter;

void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}

//Reads from propulsion specified by propulsion ID (0 to 4)
int receiver_read(unsigned int receiver_id){
  return *(RECEIVER + receiver_id);
  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}

void delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}

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

//Writes a byte to the uart2 (to be sent)
//Returns 0 is a character was sent, -1 otherwise.
int uart2_write(unsigned char data)
{
  if ((*UART2 & 0x00000001) != 0)
  {
    *UART2 = (unsigned int)data;
    return 1;
  }
  else
  {
    data = 0;
    return 0;
  }
}

//Reads a byte from uart2 (from received data) and places it int the variable
//specified by the pointer * data.
//Returns 0 is a character was read, -1 otherwise.
int uart2_read(unsigned char *data)
{
  if ((*UART2 & 0x00000002) != 0)
  {
    *data = (unsigned char)(*(UART2 + 1) & 0x000000FF);
    return 1;
  }
  else
  {
    *data = 0;
    return 0;
  }
}

// void read_data() {
//   compass_x = (i2c_read(compass_address, 0x03) << 8) | i2c_read(compass_address, 0x04);
//   compass_z = (i2c_read(compass_address, 0x05) << 8) | i2c_read(compass_address, 0x06);
//   compass_y = (i2c_read(compass_address, 0x07) << 8) | i2c_read(compass_address, 0x08);
// }


// void check_compass(void) 
// {
//   loop_counter = 0;                                                                       //Reset the loop counter.

//   //Check if the compass is responding.
//   // HWire.beginTransmission(compass_address);                     //Start communication with the compass.
//   // error = HWire.endTransmission();                              //End the transmission and register the exit status.
//   // if (error != 0) {                                             //Print a message when the compass did not respond.
//   //   Serial.println("Compass not responding");
//   // }
//   // else {

//     i2c_write(compass_address,0x00,0x10); //open communication with HMC5883
//     i2c_write(compass_address,0x00,0x60);
//     i2c_write(compass_address,0x00,0x00);
//     delay(250);
//     i2c_write(compass_address,0x00,0x11); //open communication with HMC5883
//     i2c_write(compass_address,0x00,0x60);
//     i2c_write(compass_address,0x00,0x01);
//     // HWire.write(0x00);
//     // HWire.write(0x11);
//     // HWire.write(0x60);
//     // HWire.write(0x01);
//     // HWire.endTransmission();
//     printf("Positive bias test: ");
//     delay(10);
//     read_data();
//     printf("X-axis: %d",compass_x);
//     printf(" Z-axis: %d",compass_z);
//     printf(" Y-axis: %d \n",compass_y);

//     i2c_write(compass_address,0x00,0x12); //open communication with HMC5883
//     i2c_write(compass_address,0x00,0x60);
//     i2c_write(compass_address,0x00,0x01);
//     printf("Negative bias test: ");
//     delay(10);
//     read_data();
//     printf("X-axis: %d",compass_x);
//     printf(" Z-axis: %d",compass_z);
//     printf(" Y-axis: %d \n",compass_y);

//     i2c_write(compass_address,0x00,0x10); //open communication with HMC5883
//     i2c_write(compass_address,0x00,0x60);
//     i2c_write(compass_address,0x00,0x00);

//     ///in fc code
//     // i2c_write(compass_address,0x00,0x78); //open communication with HMC5883
//     // i2c_write(compass_address,0x00,0x20);
//     // i2c_write(compass_address,0x00,0x00);

//     delay(2000);


//   // while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
//   //   delayMicroseconds(3700);                                                              //Wait for 4000us to simulate a 250Hz loop.
//   //   if (Serial.available() > 0) {                                                         //If serial data is available.
//   //     data = Serial.read();                                                               //Read the incomming byte.
//   //     delay(100);                                                                         //Wait for any other bytes to come in.
//   //     while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer.
//   //   }

//     //Tell the HMC5883 where to begin reading data
//     // HWire.beginTransmission(compass_address);
//     // HWire.write(0x03); //select register 3, X MSB register
//     // HWire.endTransmission();

//     read_data();
//     loop_counter++;
//     // if (loop_counter == 125) {
//       printf("X-axis: %d",compass_x);
//       printf(" Z-axis: %d",compass_z);
//       printf(" Y-axis: %d \n",compass_y);
//       loop_counter = 0;
//     // }
//   loop_counter = 0;                                                                       //Reset the loop counter.
// }

void compass_signalen(){
  //Read the MPU-6050
  compass_x = (i2c_read(compass_address, 0x03) << 8) | i2c_read(compass_address, 0x04);
  compass_z = (i2c_read(compass_address, 0x05) << 8) | i2c_read(compass_address, 0x06);
  compass_y = (i2c_read(compass_address, 0x07) << 8) | i2c_read(compass_address, 0x08);
 
}

void set_compass_registers()
{
  //Setup the MPU-6050
  while(i2c_write(compass_address,0x00,0x78)); //open communication with HMC5883
  while(i2c_write(compass_address,0x00,0x20));
  while(i2c_write(compass_address,0x00,0x00));

  
}

void check_gps(void) {
  loop_counter = 0;
  delay(250);

  while (loop_counter < 1000) {                                                           //Stay in this loop until the data variable data holds a q.
    if (loop_counter < 1000)loop_counter ++;
    delay(4);                                                              //Wait for 4000us to simulate a 250Hz loop.
    if (loop_counter == 1) {
      printf("\n");
      printf("====================================================================\n");
      printf("Checking gps data @ 9600bps\n");
      printf("====================================================================\n");
    }
    if (loop_counter > 1 && loop_counter < 500)while (uart2_read(&gps_data))printf("%c",gps_data);
    if (loop_counter == 500) {
      printf("\n");
      printf("====================================================================\n");
      printf("Checking gps data @ 57600bps\n");
      printf("====================================================================\n");
      delay(200);
      
      //Disable GPGSV messages
      int Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
      for(int i=0;i<11;i++)
      {
        uart2_write(Disable_GPGSV[i]);
      }
      
      delay(350);
      //Set the refresh rate to 5Hz
      int Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
      for(int i=0;i<14;i++)
      {
        uart2_write(Set_to_5Hz[i]);
      }
      delay(350);
      //Set the baud rate to 57.6kbps
      int Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                   0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                                  };
      for(int i=0;i<28;i++)
      {
        uart2_write(Set_to_57kbps[i]);
      }
      delay(200);


      uart2_read(&gps_data);
    }
    if (loop_counter > 500 && loop_counter < 1000)while (uart2_read(&gps_data))printf("%c",gps_data);

  }
  loop_counter = 0;                                                                       //Reset the loop counter.
}

int main(int argc, char **argv)
{
  printf("Hello GPS!\n");
  // set_compass_registers();

  actuator_write(m1, 1000);                                               //give motors 1000us pulse.
  actuator_write(m2, 1000);
  actuator_write(m3, 1000);
  actuator_write(m4, 1000);
  //for (int i = 0; i < 5; i++) {
  for (int j=0;j<10;j++)
  // while(1)
  {
    // while(loop_timer + 4000 > get_cpu_usecs());                                                  //Start the pulse after 4000 micro seconds.
    // loop_timer = get_cpu_usecs();                                                                //Reset the zero timer.
    check_gps();
  
    // check_compass();
    // compass_signalen();
    // printf("X-axis: %f",compass_x);
    // printf(" Z-axis: %f",compass_z);
    // printf(" Y-axis: %f \n",compass_y);
  }
  return 0;
}



