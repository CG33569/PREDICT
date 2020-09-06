/*
    The blinking example implemented with interrupts.

    Author: Wolfgang Puffitsch
    Copyright: DTU, BSD License
*/

#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <machine/rtc.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>
#include <stdio.h>
#include <stdlib.h>

unsigned int pwm0, pwm1, pwm2, pwm3;

// definitions for I/O devices
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )
#define LEDS (*((volatile _IODEV unsigned *)  PATMOS_IO_LED))
#define SLEEP (*((volatile _IODEV unsigned *) PATMOS_IO_EXCUNIT+0x10))

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3
//Receiver controller
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

// the blinking frequency in microseconds
#define PERIOD 1000000

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}

void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}

//Reads from propulsion specified by propulsion ID (0 to 4)
int receiver_read(unsigned int receiver_id){
  // return *(RECEIVER + receiver_id);
  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}

//Blinks the LEDs once
void blink_once()
{
  int i, j;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0001;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0000;
  return;
}

void intr_handler(void) __attribute__((naked));

// variable keep track of time and generate interrupts without drift
static unsigned long long start_time;

// int main(void) {

  
//   // register exception handler
//   exc_register(18, &intr_handler);

//   // unmask interrupts
//   intr_unmask_all();
//   // clear pending flags
//   intr_clear_all_pending();
//   // enable interrupts
//   intr_enable();

//   actuator_write(m1, 1000);                                               //give motors 1000us pulse.
//   actuator_write(m2, 1000);
//   actuator_write(m3, 1000);
//   actuator_write(m4, 1000);

  

//   // // arm timer
//   // start_time = get_cpu_usecs() + PERIOD/2;
//   // arm_usec_timer(start_time);

//   // // generate output
//   // LEDS = 0;
//   // putc('0', stderr);

//   // loop forever
//   for(int i=0;i<1000;i++) {
//     // blink_once();
//     pwm0 = receiver_read(0);
//     pwm1 = receiver_read(1);
//     pwm2 = receiver_read(2);
//     pwm3 = receiver_read(3);
//     // micros(100000);
//     printf("%d  %d  %d  %d\n", pwm0 ,pwm1,pwm2,pwm3);

//     // if (pwm0>1078 && pwm0<2000)
//     // {
//     //   arm_usec_timer(1);
//     // }

//     actuator_write(0,pwm0);


//   }
// }

// // interrupt handler
// void intr_handler(void) {
//   exc_prologue();

//   printf("in interrupt\n");
  

//   actuator_write(0,pwm0);

//   // arm timer for next interrupt
//   // start_time += PERIOD/2;
//   // arm_usec_timer(start_time);

//   // // generate output
//   // LEDS ^= 1;
//   // putc('0' + (LEDS & 1), stderr);

//   exc_epilogue();
// }



/////////////////////////////original

int main(void) {
  actuator_write(m1, 1000);                                               //give motors 1000us pulse.
  actuator_write(m2, 1000);
  actuator_write(m3, 1000);
  actuator_write(m4, 1000);
  // register exception handler
  exc_register(17, &intr_handler);

  // unmask interrupts
  intr_unmask_all();
  // clear pending flags
  intr_clear_all_pending();
  // enable interrupts
  intr_enable();

  // arm timer
  start_time = get_cpu_usecs() + PERIOD/2;
  arm_usec_timer(start_time);

  // generate output
  LEDS = 0;
  putc('0', stderr);

  // loop forever
  for(int i=0;i<1000;i++) {
    printf("%d  %d  %d  %d\n", pwm0 ,pwm1,pwm2,pwm3);
  }
}

// interrupt handler
void intr_handler(void) {
  exc_prologue();

  // arm timer for next interrupt
  start_time = 100000;
  arm_usec_timer(start_time);

  // generate output
  pwm0 = receiver_read(0);
  pwm1 = receiver_read(1);
  pwm2 = receiver_read(2);
  pwm3 = receiver_read(3);
  // putc('0' + (LEDS & 1), stderr);

  exc_epilogue();
}
