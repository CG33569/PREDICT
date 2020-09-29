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
unsigned char uart_data=0;

int result;
//---- to filter Data
char str[500];
int str_i[500];
char str_c[500];
char cPRE[6] = "$PREDI";


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


void print_message(void){
  /* Attempt to decode the user supplied string */
  for (int i=0;i<300;i++){
    str[i] = str_c[i];
    printf("%c",str[i]);
  }

  printf("\n");
}

void check_telemetry(void) {
  int loop_counter = 0;
  //------ to filter data ------------------
  int str_temp[6];
  char str_tempc[6];
  int start_temp = 0;
  int end_temp = 6;
  bool b_temp = false;
  bool printed = false;
  bool equal_PY = false;
  int start_c = 0;
  int end_c = 300;

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
      while (uart2_read(&uart_data)){
        //printf("%c",uart_data);
        //The delimiter "$" is 36 in ASCII
        if(uart_data == 36){
          b_temp = true;
          printf("Test: found beginning\n");
        }
        if(b_temp && (start_temp<end_temp)){
          str_temp[start_temp]=uart_data;
          str_tempc[start_temp]=(char)uart_data;
          start_temp++;
        }

        if(equal_PY&&(start_c<end_c)){
          str_c[start_c] = (char) uart_data;
          start_c++;
        }
        //find the delimiter string
        if((start_temp==end_temp)&&!equal_PY){//&&!printed){
          //printed = true;
          printf("Checking delimiter\n");
          b_temp = false;
          int comp = 0;
          for(int j=0;j<6;j++){
             comp = comp + str_tempc[j] - cPRE[j];
            // str_c[j] = str_tempc[j];
          }
          if(comp == 0){
            equal_PY = true;
            printf("Found message\n");
          }
          start_temp = 0; // Try again?
        }

      }
    }
    print_message();
    if (loop_counter == 500) {
      printf("\n");

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


      uart2_read(&uart_data);
    }

  }
  loop_counter = 0;                                                                       //Reset the loop counter.
}

int main(int argc, char **argv)
{
  printf("Hello Telemetry!\n

  char *START_STR = "$PREDI";
  char *END_STR = "!";
  char START_IN[6] = ".FPGA1";
  char full_message[24];
  char *xstr = " sample text 1.";

  snprintf(full_message, 24, "%s%s%s",START_STR,xstr,END_STR);

  int Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
// Write
/*
  for (int j=0;j<20;j++)
  // while(1)
  {
  //  check_telemetry();
    for(int i=0;i<24;i++)
    {
      //int temp = fill_message[i]-48;
      uart2_write(Set_to_57kbps[i]);
      millis(100);
    }
    millis(100);
    printf("Writing stuff nr.%d: %s\n",j,full_message);
  }
*/
for (int j=0;j<100000;j++){
  millis(4);                                                              //Wait for 4000us to simulate a 250Hz loop.
  printf("message nr.%d\n",j);
    while (uart2_read(&uart_data)){
      printf("%c",uart_data);
    }
    millis(10);
}
  return 0;
}
