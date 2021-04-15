#include "../sensors/PREDICTSensors.h"
#include "../FC_functionalities/PREDICTFunctions.h"

const int UART_LENGTH = 128;
const int UART_SEND_LENGTH = 128;

int main(int argc, char **argv)
{
  printf("Hello Coppelia!\n");
  char *xstr = "";
  char sendUART[UART_SEND_LENGTH]="";
  char recUART[UART_LENGTH]="";
  for (int j=0;j<30;j++)
  {
    // Read receiver channels
    transmitter_read();
    memset(sendUART, '\0', UART_SEND_LENGTH);
    sprintf(sendUART, "%d;%d;%d;%d;%d;%d;%d;", j, channel_1, channel_2, channel_3, channel_4, channel_5, channel_6);
    strcpy(xstr, sendUART);
    // send through uart
    printf("Sending nr.%d: %s. \n",j,xstr);
    send_telemtry(xstr,UART_SEND_LENGTH);
    millis(200);
    receive_telemtry(recUART,UART_SEND_LENGTH);
    printf("received: %s\n",recUART);
  }
  // Finish simulator
  sprintf(sendUART, "END_SIM");
  strcpy(xstr, sendUART);
  printf("Finish: %s. \n",xstr);
  send_telemtry(xstr,UART_SEND_LENGTH);
  millis(100);
  return 0;
}
