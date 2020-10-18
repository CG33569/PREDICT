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
#include "PREDICT.h"

struct gps_tpv tpv;
int result;
//---- to filter Data
int str_i[500];
char str_c[500];


void print_gps(char*str){
  /* Attempt to decode the user supplied string */
  result = gps_decode(&tpv, str);
  if (result != GPS_OK)
  {
      fprintf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
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

void check_gps(gps_tpv& tpv) {
  char cRMC[6] = "$GNRMC";
  char cVTG[6] = "$GNVTG";
  char cGGA[6] = "$GNGGA";
  char cGGL[6] = "$GNGGL";
  char cGSA[6] = "$GNGSA";
  // Drone GPS = GNRMC, NEO6M GPS = GPRMC
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

  micros(250);

  while (loop_counter <500) {                                                           //Stay in this loop until the data variable data holds a q.
    if (loop_counter < 1000)loop_counter ++;
    micros(4);                                                              //Wait for 4000us to simulate a 250Hz loop.

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

  }
  printf("\n");
  print_gps(str_c,tpv);
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
