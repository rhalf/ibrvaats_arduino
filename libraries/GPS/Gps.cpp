#include "Gps.h"
#include "Arduino.h"
#include <ArduinoNmeaParser.h>


typedef void (* Callback)();


Gps::Gps(uint8_t rx = 16,uint8_t tx = 17){
  Serial2.begin(9600, SERIAL_8N1, rx, tx);
}

void Gps::run() { 
  while (Serial2.available()) {
    parser.encode((char) Serial2.read());
  }
}


void Gps::onRmcUpdate(nmea::RmcData const rmc) {
  Serial.print(rmc.time_utc.hour);
  Serial.print(":");
  Serial.print(rmc.time_utc.minute);
  Serial.print(":");
  Serial.print(rmc.time_utc.second);
  Serial.print(".");
  
  
  if (rmc.is_valid) {
    Serial.print(" : LON ");
    Serial.print(rmc.longitude, 6);
    Serial.print(" ° | LAT ");
    Serial.print(rmc.latitude, 6);
    Serial.print(" ° | VEL ");
    Serial.print(rmc.speed);
    Serial.print(" m/s | SPEED ");
    Serial.print(rmc.course);
    Serial.print(" ° | COURSE");
    Serial.print(rmc.magnetic_variation);
    // Serial.print(" ° | Date");
    // Serial.print(rmc.date);
  }

  Serial.println();
}