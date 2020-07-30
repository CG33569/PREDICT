#include <stdio.h>
#include <machine/patmos.h>
#include <machine/rtc.h>
#include "demo_tasks.h"

#define LED (*((volatile _IODEV unsigned *)PATMOS_IO_LED))
//#define ACTUATORS ( ( volatile _IODEV unsigned * )	PATMOS_IO_ACT) // the real pins did not work out
#define ACTUATORS ( ( volatile _IODEV unsigned * )	PATMOS_IO_ACT+0x10 )

//Writes to actuator specified by actuator ID (0 to 4)
// The data is the PWM signal width in us (from 1000 to 2000 = 1 to 2 ms)
void actuator_write(unsigned int actuator_id, unsigned int data){
  *(ACTUATORS + actuator_id) = data;
}

int main() {
  int i;
  uint64_t led, loop_timer, timer_ms;

// Start output values

 actuator_write(0, 1000);
 actuator_write(1, 1000);
 actuator_write(2, 1000);
 actuator_write(3, 1000);

 for (i=0; i<3; ++i) {

 // 2 Seconds wait for initialization
 timer_ms = (get_cpu_usecs()/MS_TO_US);
 loop_timer = timer_ms;
 while(timer_ms - loop_timer < 2000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

 printf("--Start\n");
 	 //Test motor 1
	 actuator_write(0, 1300);
	 actuator_write(1, 1000);
	 actuator_write(2, 1000);
	 actuator_write(3, 1000);
	 // 5 seconds spinning at that velocity
	 timer_ms = (get_cpu_usecs()/MS_TO_US);
	 loop_timer = timer_ms;
	 while(timer_ms - loop_timer < 5000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

 	 //Test motor 2
	 actuator_write(0, 1000);
	 actuator_write(1, 1300);
	 actuator_write(2, 1000);
	 actuator_write(3, 1000);
	 // 5 seconds spinning at that velocity
	 timer_ms = (get_cpu_usecs()/MS_TO_US);
	 loop_timer = timer_ms;
	 while(timer_ms - loop_timer < 5000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

 	 //Test motor 3
	 actuator_write(0, 1000);
	 actuator_write(1, 1000);
	 actuator_write(2, 1300);
	 actuator_write(3, 1000);
	 // 5 seconds spinning at that velocity
	 timer_ms = (get_cpu_usecs()/MS_TO_US);
	 loop_timer = timer_ms;
	 while(timer_ms - loop_timer < 5000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

 	 //Test motor 4
	 actuator_write(0, 1000);
	 actuator_write(1, 1000);
	 actuator_write(2, 1000);
	 actuator_write(3, 1300);
	 // 5 seconds spinning at that velocity
	 timer_ms = (get_cpu_usecs()/MS_TO_US);
	 loop_timer = timer_ms;
	 while(timer_ms - loop_timer < 5000){timer_ms = (get_cpu_usecs()/MS_TO_US);}
	 printf("--End.\n");

	}

	 // Stop the motors before exit the programme
	 actuator_write(0, 1000);
	 actuator_write(1, 1000);
	 actuator_write(2, 1000);
	 actuator_write(3, 1000);
	 
return 0;
}
