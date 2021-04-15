#include "PREDICTdummy.h"


int main(int argc, char **argv)
{
  unsigned int adc_val;
  printf("Analog reading test App.\n");

  printf("Config word --> read value\n");
  unsigned int ADC_CH0 = 1;

while(ADC_CH0<2147483648)
{

  for (int i=0; i<5; i++)
  {
    // First write the config word and send it to the ADC module
    write_adc(ADC_CH0);
  //  printf("config word = %d --> ", ADC_CH0);
    printf("    %d     -->     ", ADC_CH0);
   
    // Read what the module has measured acording to that configuration
    adc_val = read_adc();
    printf("%d\n",adc_val);
    millis(100);
  }

  ADC_CH0 = ADC_CH0*2;
}

  return 0;
}