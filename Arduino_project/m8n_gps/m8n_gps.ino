
// Arduino Nano GPS (NEO 8MN)with Liquid Crystal I2C





#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h> 
//#include <LiquidCr/ystal_I2C.h>

  // Set the LCD address to 0x27 for a 16 chars and 2 line display
//  LiquidCryst/al_I2C lcd(0x27, 16, 2);
  SoftwareSerial serial_connection(2, 3); //RX=pin 10, TX=pin 11
  TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

void setup()
{
  Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started

  // initialize the LCD
//  lcd.begin();/

  // lcd.backlight(); // turns on the back light (with jumper installed)
//  lcd.noBacklight();/ // Turn off the blacklight (with jumper installed)
  
  
}

void loop()
{
  while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.println("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 6);
    Serial.println("Speed MPH:");
    Serial.println(gps.speed.mph());
    Serial.println("Altitude Feet:");
    Serial.println(gps.altitude.feet());
    Serial.println("");
//    lcd.setCursor(1,0);
//    lcd.print("Lat:");
//    lcd.setCursor(7,0);
//    lcd.print(gps.location.lat(),6);
//    lcd.setCursor(1,1);
//    lcd.print("Long:");
//    lcd.setCursor(6,1);
//    lcd.print(gps.location.lng(),6);
  }
}
