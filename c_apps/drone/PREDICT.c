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

#include "PREDICT.h"
// ================================ Variables ===============================
const unsigned int CPU_PERIOD = 20;
// ================================ Functions  ===============================
//-----> Basic usage:
void millis (int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = get_cpu_usecs()/1000;
}
void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
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
//Writes to actuator specified by actuator ID (0 to 4)
// The data is the PWM signal width in us (from 1000 to 2000 = 1 to 2 ms)
void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}
//get pulse width data from receiver
int receiver_read(unsigned int receiver_id){

  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
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
//-----> Components handling:
// telemetry modules
void send_telemtry(char *xstr)
{
  char *START_STR = ".FPGA1";
  char *END_STR = "!";
  char full_message[100];
  memset(full_message, '\0', 100);
  snprintf(full_message, 30, "%s%s%s",START_STR,xstr,END_STR);
  for(int i=0;i<30;i++)
  {
    uart2_write(full_message[i]);
    micros(1);
  }
}

void receive_telemtry(char *Outstr)
{
  unsigned char uart_data=0;
  char full_message[124]="";
  memset(full_message, '\0', 512);
  bool busy = true, start_temp = false, start = false;
  int k =0, comp = 0, cnt = 0, pos = 0 ;

    while((k<124) && busy){ //(busy){
      uart2_read(&uart_data);
      // find end
      if((uart_data=='-')&&start){
        start = false;
        //printf("Message: %s\n",full_message);
        cnt=0;
        pos=0;
        busy = false;
      }
      // Appending message
      if(start){
        full_message[cnt]=uart_data;
        cnt++;
      }
      // find beginning
      if((uart_data=='$') && !start){
        pos = 0;
        start = true;
        cnt = 0;
      }
      micros(3);
      k++;
    }
    strcpy(Outstr, full_message);
}
// GPS
/*
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

void print_gps(char*str,struct gps_tpv* tpv){
  // Attempt to decode given string
  int result;

  result = gps_decode(tpv, str);
  if (result != GPS_OK)
  {
      fprintf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
    //  return EXIT_FAILURE;
  }

  // Go through each TPV value and show what information was decoded 
  printf("Talker ID: %s\n", tpv->talker_id);
  printf("Time Stamp: %s\n", tpv->time);
  print_tpv_value("Latitude", "%.6f\n", tpv->latitude, GPS_LAT_LON_FACTOR);
  print_tpv_value("Longitude", "%.6f\n", tpv->longitude, GPS_LAT_LON_FACTOR);
  print_tpv_value("Altitude", "%.3f\n", tpv->altitude, GPS_VALUE_FACTOR);
  print_tpv_value("Track", "%.3f\n", tpv->track, GPS_VALUE_FACTOR);
  print_tpv_value("Speed", "%.3f\n", tpv->speed, GPS_VALUE_FACTOR);

  printf("Mode: ");
  switch (tpv->mode)
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

void check_gps(struct gps_tpv* tpv){
  int loop_counter;
  unsigned char gps_data=0;
  //struct gps_tpv tpv;
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
  //---- to filter Data
  int str_i[500];
  char str_c[500];
  char cRMC[6] = "$GNRMC";
  char cVTG[6] = "$GNVTG";
  char cGGA[6] = "$GNGGA";
  char cGGL[6] = "$GNGGL";
  char cGSA[6] = "$GNGSA";
  // Drone GPS = GNRMC, NEO6M GPS = GPRMC

  millis(250);

  while (loop_counter < 500) {                                                           //Stay in this loop until the data variable data holds a q.
    if (loop_counter < 500)loop_counter ++;
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
      print_gps(str_c,tpv);

  }
  loop_counter = 0;                                                                       //Reset the loop counter.
}
*/