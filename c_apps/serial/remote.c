#define TERMINAL    "/dev/ttyUSB0"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

//==========================================================================
// ALL THIS DOES NOT LOOK NECESSARY
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    // tty.c_cc[VMIN] = 1;
    // tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] > 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}
//
// void set_mincount(int fd, int mcount)
// {
//     struct termios tty;
//
//     if (tcgetattr(fd, &tty) < 0) {
//         printf("Error tcgetattr: %s\n", strerror(errno));
//         return;
//     }
//
//     tty.c_cc[VMIN] = mcount ? 1 : 0;
//     tty.c_cc[VTIME] = 5;        /* half second timer */
//
//     if (tcsetattr(fd, TCSANOW, &tty) < 0)
//         printf("Error tcsetattr: %s\n", strerror(errno));
// }
//==========================================================================

int main()
{
    char *portname = TERMINAL;
    int fd;
    int wlen;
    char *xstr = "Hello!1234567891011\n\r";
    int xlen = strlen(xstr);

    printf("%s",xstr);
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    printf("Port opened sucessfully\n");
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    //set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */
    printf("Set interface\n");
    /* simple output */
    wlen = write(fd, xstr, xlen);
    if (wlen != xlen) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */


// Reading variables
unsigned char buf[80];
int n = 0;
int num_bytes;

char *START_STR = "$PREDI";
char *END_STR = "!";
char START_IN[6] = ".FPGA1";
char full_message[512];

/* Main questions to adapt this to Patmos:
 * How to modify read and write, so it does it from the main uart
 * What the hell, honestly
*/
  while(1){
      //printf("Writing stuff nr.%d \n",n);
      char *xstr = " sample text 1.";
      snprintf(full_message, 512, "%s%s%s",START_STR,xstr,END_STR);
      write(fd, full_message, strlen(full_message));
      tcdrain(fd);
      sleep(0.1);

      memset(buf, 0, sizeof buf);
      num_bytes = read(fd, buf, sizeof(buf));
      if(num_bytes>0){
        //For some reason, to be sync, it requires to be at least 5 messages read-written
        n = n+1;
        if(n>5){
          printf("Received: %s\n",buf);
          //Keep analysing and decoding message from here
        }
      }
      // The rest of the if is not necessary
      /*else if (num_bytes <= 0) {
          //printf("Error reading: %s", strerror(errno));
          n=0;
      }else{
        n=0;
        printf("Timeout\n");
      }
      */
      sleep(1);
  }

  close(fd);

}
