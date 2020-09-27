#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
//new
//#include "TinyGPSpp.h"

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

unsigned char gps_data=0;
float compass_x, compass_y, compass_z;
int loop_counter;


void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = get_cpu_usecs()/1000;
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



void check_gps(void) {
  loop_counter = 0;
  millis(250);

  while (loop_counter < 1000) {                                                           //Stay in this loop until the data variable data holds a q.
    if (loop_counter < 1000)loop_counter ++;
    millis(4);                                                              //Wait for 4000us to simulate a 250Hz loop.
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
      millis(200);

      //Disable GPGSV messages
      int Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
      for(int i=0;i<11;i++)
      {
        uart2_write(Disable_GPGSV[i]);
      }

      millis(350);
      //Set the refresh rate to 5Hz
      int Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
      for(int i=0;i<14;i++)
      {
        uart2_write(Set_to_5Hz[i]);
      }
      millis(350);
      //Set the baud rate to 57.6kbps
      int Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                   0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                                  };
      for(int i=0;i<28;i++)
      {
        uart2_write(Set_to_57kbps[i]);
      }
      millis(200);


      uart2_read(&gps_data);
    }
    if (loop_counter > 500 && loop_counter < 1000)while (uart2_read(&gps_data))printf("%c",gps_data);

  }
  loop_counter = 0;                                                                       //Reset the loop counter.
}

int main(int argc, char **argv)
{
  printf("Hello GPS!\n");

  for (int j=0;j<30;j++)
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
