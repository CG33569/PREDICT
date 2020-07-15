#include <stdio.h>
#include <machine/patmos.h>
#include <machine/rtc.h>
#include "demo_tasks.h"

#define LED (*((volatile _IODEV unsigned *)PATMOS_IO_LED))
#define MOTOR ( ( volatile _IODEV unsigned * )	PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//Writes to actuator specified by actuator ID (0 to 4)
// The data is the PWM signal width in us (from 1000 to 2000 = 1 to 2 ms)
void actuator_write(unsigned int actuator_id, unsigned int data){
  *(MOTOR + actuator_id) = data;
}

int main() {

  int i, j;
  unsigned int esc_1, esc_2, esc_3, esc_4, led;
  uint64_t loop_timer, timer_ms, esc_loop_timer;

// Start output values
 LED = 0;
 actuator_write(m1, 1000);
 actuator_write(m2, 1000);
 actuator_write(m3, 1000);
 actuator_write(m4, 1000);
 // 2 Seconds wait for initialization
 timer_ms = (get_cpu_usecs()/MS_TO_US);
 loop_timer = timer_ms;
 while(timer_ms - loop_timer < 2000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

 printf("--Started.\n");
  for (i=0; i<10; ++i) {

    // Example constant values from the controller
    esc_1 = 1800;
    esc_2 = 1500;
    esc_3 = 1400;
    esc_4 = 1200;

    actuator_write(m1, esc_1);
    actuator_write(m2, esc_2);
    actuator_write(m3, esc_3);
    actuator_write(m4, esc_4);
    // 2 Seconds just spinning stuff
    timer_ms = (get_cpu_usecs()/MS_TO_US);
    loop_timer = timer_ms;
    while(timer_ms - loop_timer < 2000){timer_ms = (get_cpu_usecs()/MS_TO_US);}

}

actuator_write(m1, 1000);
actuator_write(m2, 1000);
actuator_write(m3, 1000);
actuator_write(m4, 1000);

return 0;
}
