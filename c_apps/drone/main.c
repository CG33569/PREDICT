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

int main(int argc, char **argv)
{
  printf("Hello Telemetry!\n");
  char *xstr = "sample text 1";
  char recUART[512]="";
  for (int j=0;j<20;j++)
  {
    printf("\nWriting stuff nr.%d: %s\n",j,xstr);
    send_telemtry(xstr);
    micros(100);
    receive_telemtry(recUART);
    printf("received: %s\n",recUART);
  }

  return 0;
}
