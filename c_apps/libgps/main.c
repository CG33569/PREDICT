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
//new

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

int loop_counter;
unsigned char gps_data=0;

struct gps_tpv tpv;
int result;
//---- to filter Data
char str[500];
int str_i[500];
char str_c[500];
char cRMC[6] = "$GNRMC";
char cVTG[6] = "$GNVTG";
char cGGA[6] = "$GNGGA";
char cGGL[6] = "$GNGGL";
char cGSA[6] = "$GNGSA";
// Drone GPS = GNRMC, NEO6M GPS = GPRMC


// THis shitty fcn is from Arduino
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

void print_tpv_value(const char *name, const char *format, const int32_t value, const int32_t scale_factor)
{
    printf("%s: ", name);
    if (GPS_INVALID_VALUE != value)
    {
        printf(format, (double)value / scale_factor);
    }
    else
    {
        puts("INVALID");
    }
}

void print_gps(void){
  /* Attempt to decode the user supplied string */
  for (int i=0;i<300;i++){
    str[i] = str_c[i];
  }

  result = gps_decode(&tpv, str);
  if (result != GPS_OK)
  {
      fprintf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
    //  return EXIT_FAILURE;
  }

  /* Go through each TPV value and show what information was decoded */
  printf("Talker ID: %s\n", tpv.talker_id);
  printf("Time Stamp: %s\n", tpv.time);
  print_tpv_value("Latitude", "%.6f\n", tpv.latitude, GPS_LAT_LON_FACTOR);
  print_tpv_value("Longitude", "%.6f\n", tpv.longitude, GPS_LAT_LON_FACTOR);
  print_tpv_value("Altitude", "%.3f\n", tpv.altitude, GPS_VALUE_FACTOR);
  print_tpv_value("Track", "%.3f\n", tpv.track, GPS_VALUE_FACTOR);
  print_tpv_value("Speed", "%.3f\n", tpv.speed, GPS_VALUE_FACTOR);

  printf("Mode: ");
  switch (tpv.mode)
  {
  case GPS_MODE_UNKNOWN:
      puts("Unknown");
      break;
  case GPS_MODE_NO_FIX:
      puts("No fix");
      break;
  case GPS_MODE_2D_FIX:
      puts("2D");
      break;
  case GPS_MODE_3D_FIX:
      puts("3D");
      break;
  default:
      break;
  }

  printf("\n");
}

void check_gps(void) {
  int loop_counter = 0;
  //------ to filter data ------------------
  int str_temp[6];
  char str_tempc[6];
  int start_temp = 0;
  int end_temp = 6;
  bool b_temp = false;
  bool printed = false;
  bool equal_RMC = false;
  int start_c = 6;
  int end_c = 300;
  bool equal_VTG = false;


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
    //if (loop_counter > 1 && loop_counter < 500){
    if (loop_counter >=1 && loop_counter < 500){
      while (uart2_read(&gps_data)){
        //printf("%c",gps_data);
        //The delimiter "$" is 36 in ASCII
        if(gps_data == 36){
          b_temp = true;
        }
        if(b_temp && (start_temp<end_temp)){
          str_temp[start_temp]=gps_data;
          str_tempc[start_temp]=(char)gps_data;
          start_temp++;
        }

        if(equal_RMC&&(start_c<end_c)){
          str_c[start_c] = (char) gps_data;
          start_c++;
        }
        //find the RMC string
        if((start_temp==end_temp)&&!equal_RMC){//&&!printed){
          //printed = true;
          b_temp = false;
          int comp = 0;
          for(int j=0;j<6;j++){
             comp = comp + str_tempc[j] - cRMC[j];
             str_c[j] = str_tempc[j];
          }
          if(comp == 0){
            equal_RMC = true;
          }
          start_temp = 0; // Try again?
        }

        //find the VTG string
        if((start_temp==end_temp)&&equal_RMC&&!equal_VTG){
          b_temp = false;
          int comp = 0;
          for(int j=0;j<6;j++){
             comp = comp + str_tempc[j] - cVTG[j];
             //str_c[j] = str_tempc[j];
          }
          if(comp == 0){
            equal_VTG = true;
          //  printf("\n\n");
          }else{
            equal_RMC = false; //If the next string is not VTG, it must go back false
            start_c = 6;
          }
          start_temp = 0; // Try again?
        }
      }
    }

    if (loop_counter == 500) {
      printf("\n");


      print_gps();
      bool start_b = false;
      int start = 0;
      int end = 256;

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

    if (loop_counter >= 500 && loop_counter < 1000){
      while (uart2_read(&gps_data)){

      }
    }
  }
  loop_counter = 0;                                                                       //Reset the loop counter.
}

int main(int argc, char **argv)
{
  gps_init_tpv(&tpv);
  printf("Hello GPS!\n");

  for (int j=0;j<4;j++)
  // while(1)
  {
    check_gps();
  }

  return 0;
}
