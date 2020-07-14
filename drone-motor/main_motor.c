#include <stdio.h>
#include <machine/patmos.h>
#include <machine/spm.h>
#include <stdint.h>
#include <inttypes.h>
#include <machine/rtc.h>
#include "tt_minimal_scheduler.h"
#include "demo_tasks.h"
#include "schedule.h"

#define PORTD (*((volatile _IODEV unsigned *)PATMOS_IO_LED))
//Actuators and Propulsion controller
#define ACTUATORS (*(( volatile _IODEV unsigned * )	PATMOS_IO_ACT))
//#define PROPULSION ( ( volatile _IODEV unsigned * )	PATMOS_IO_ACT+0x10 )
int main() {

  int i, j;
  uint64_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_1, esc_2, esc_3, esc_4, led;
  uint64_t loop_timer, timer_ms, esc_loop_timer;

// Start output values
 ACTUATORS = 0;
 PORTD = 0;
 printf("--Started.\n");
  for (i=0; i<10; ++i) {

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating the pulses for the ESC's is explained in this video:
    //https://youtu.be/fqEkVcqxtU8
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // Example constant values
    esc_1 = 1800;
    esc_2 = 1500;
    esc_3 = 1400;
    esc_4 = 1200;

  // Calculating times
  timer_ms = (get_cpu_usecs()/MS_TO_US);
  loop_timer = timer_ms;
  //printf("--Waiting...");
  while(timer_ms - loop_timer < 4000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

  led = 15;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + timer_ms;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + timer_ms;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + timer_ms;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + timer_ms;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  //printf("Time = %llu ms, Goal = %llu\n", timer_ms, timer_channel_1);

  led = 16; // Only a start value
  while (led!=0) {
    esc_loop_timer = (get_cpu_usecs()/MS_TO_US);
    led = 0;
    if(timer_channel_1 >= esc_loop_timer){led=1;}
    if(timer_channel_2 >= esc_loop_timer){led=led+2;}
    if(timer_channel_3 >= esc_loop_timer){led=led+4;}
    if(timer_channel_4 >= esc_loop_timer){led=led+8;}

    //printf("%d-",led);

    PORTD = led;
    ACTUATORS = PORTD;
  }

// Printing and testing stuff
  //printf("\nEnd n.%d\n",i);
}

return 0;
}
