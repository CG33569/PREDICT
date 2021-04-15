//
// Created by rahul on 11/8/20.
//

#ifndef PREDICT_BASIC_H
#define PREDICT_BASIC_H
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

#include "fixedptc.h"
#include "FC_global.h"
#include "gps.h"
#include "i2c_master.h"
// Multicore
pthread_mutex_t printmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t putsmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t timermutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t finishmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// ======================================== TIMING ===================================
//delay function ins microseconds
void micros(int microseconds)
{
    unsigned int timer_ms = (get_cpu_usecs());
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}

void millis(int milliseconds)
{
    unsigned int timer_ms = (get_cpu_usecs()/1000);
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}

// ======================================== LEDS ===================================
//LEDs register
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )

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

///1 to switch on LED and 0 to off
void LED_out(int i){
    if(i==1) LED = 0x0001;
    else LED = 0x0000;
    return;
}

// ======================================== ANALOG ===================================
//SPI
#define ADC ((volatile _IODEV unsigned *)0xf00e0000)
// This is the value of the config value to set CH0-CH1 as differential, so the module:
// - VCC goes to 5V
// - CH1 and GND go to 0V
// - CH0 goes to the analog value
//              S/D O/S S1 S0 UNI SLP
// bit position: 5   4   3  2  1   0
// unipolar CH0: 1   0   0  0  0   0   = 16
// polar CH0-1:  0   0   0  0  0   0   = 0

unsigned int reverseBits(unsigned int n)
{
  unsigned int rev = 0;
  while(n > 0)
  {
    rev <<= 1;
    if((n&1) == 1)
    {
      rev ^= 1;
    }
    n >>= 1;
  }

  return rev;
}

void write_adc(unsigned int config_word)
{
  *(ADC) = config_word;
}

int read_adc()
{
  return reverseBits(*(ADC));
}

//
unsigned int read_battery()
{
  unsigned const int ADC_CH0 = 0;   // 0x1000
  unsigned int adc_val = 0;

  // read 3 times to make sure to get a stable value
  for(int i=0; i<3; i++)
  {
      write_adc(ADC_CH0);
      adc_val = read_adc();
  }

  return adc_val;
}

// ======================================== RECEIVER / ACTUATOR ===================================
const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//Receiver controller register
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )


//writes pwm signlas of width=data to the esc
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

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel, reverse;       //First we declare some local variables
  int actual;                           ///(0,th,roll,pitch,yaw)
  int difference;

  if(function==0)
  {
    reverse = reverse_channel[2];                        //Reverse =1 when the transmitter channels are reversed, else 0
    channel =1;//roll
  }
  else if(function==1)
  {
    reverse = reverse_channel[1];
    channel=2;//pitch
  }
  else if(function==2)
  {
    reverse = reverse_channel[0];
    channel=0;//throttle
  }
  else
  {
    reverse = reverse_channel[3];                                            
    channel =3;//yaw
  }

  actual = receiver_read(channel);      //Read the actual receiver value for the corresponding function
  // low = 1000;  //Store the low value for the specific receiver input channel
  // center = 1500; //Store the center value for the specific receiver input channel
  // high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center[channel]){                                                         //The actual receiver value is lower than the center value
    if(actual < low[channel])actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center[channel]){                                                                        //The actual receiver value is higher than the center value
    if(actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

// stores receiver values in an global array
void transmitter_read() {
    // read the receiver pwm duty cycle

    int channel_1_tmp = convert_receiver_channel(0);  //1(0)               //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    int channel_2_tmp = convert_receiver_channel(1);  //2(1)               //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    int channel_3_tmp = convert_receiver_channel(2);  //0(0)               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    int channel_4_tmp = convert_receiver_channel(3);  //3(0)               //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    int channel_5_tmp = receiver_read(4);
    int channel_6_tmp = receiver_read(5);
    pthread_mutex_lock(&mutex);
    channel_1 = channel_1_tmp;
    channel_2 = channel_2_tmp;
    channel_3 = channel_3_tmp;
    channel_4 = channel_4_tmp;
    channel_5 = channel_5_tmp;
    channel_6 = channel_6_tmp;
    pthread_mutex_unlock(&mutex);

    if(motor_publish)
    {
      actuator_write(m1, esc_1);                                                 //give motors 1000us pulse.
      actuator_write(m2, esc_2);
      actuator_write(m3, esc_3);
      actuator_write(m4, esc_4);
      motor_publish=false;
    }
}

// ======================================== UART 2 and UART 3 ===================================

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)
#define UART3 ((volatile _IODEV unsigned *)PATMOS_IO_UART3)

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


//Writes a byte to the uart2 (to be sent)
//Returns 0 is a character was sent, -1 otherwise.
int uart3_write(unsigned char data)
{
  if ((*UART3 & 0x00000001) != 0)
  {
    *UART3 = (unsigned int)data;
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
int uart3_read(unsigned char *data)
{
  if ((*UART3 & 0x00000002) != 0)
  {
    *data = (unsigned char)(*(UART3 + 1) & 0x000000FF);
    return 1;
  }
  else
  {
    *data = 0;
    return 0;
  }
}

// ===========================================================================


#endif //PREDICT_BASIC_H
