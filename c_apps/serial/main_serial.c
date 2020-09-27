#include <stdio.h>
#include <machine/patmos.h>
#include <machine/rtc.h>
#include "demo_tasks.h"

#define LED (*((volatile _IODEV unsigned *)PATMOS_IO_LED))


int main()
{
    int a, b, c, i;
    uint64_t loop_timer, timer_ms;

     for (i=0; i<10; ++i) {
      printf("This is the channel you are looking for!\n\r");
     // 2 Seconds wait for initialization
     timer_ms = (get_cpu_usecs()/MS_TO_US);
     loop_timer = timer_ms;
     while(timer_ms - loop_timer < 5000){timer_ms = (get_cpu_usecs()/MS_TO_US);}
   }

    for(i=0; i<5; ++i) {
      printf("Enter the first value:");
      scanf("%d", &a);
      printf("Enter the second value:");
      scanf("%d", &b);
      c = a + b;
      printf("%d + %d = %d\n\r", a, b, c);
  }
    return 0;
}