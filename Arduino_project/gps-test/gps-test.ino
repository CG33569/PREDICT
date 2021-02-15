
#include <SoftwareSerial.h>
SoftwareSerial ss(4,3); // RX, TXi//nt16_t loop_counter;
uint8_t data, start, warning;
int16_t loop_counter;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);                                        //Set the serial output to 57600 kbps.
}

void loop() {

  if (Serial.available() > 0) {
    data = Serial.read();                                       //Read the incomming byte.
    delay(100);                                                 //Wait for any other bytes to come in.
    while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer.
  }

  if(data == 'i')
  {
//     put your main code here, to run repeatedly:/
    Serial.println(F("Checking raw GPS data."));
    check_gps();
  }//
}
  
