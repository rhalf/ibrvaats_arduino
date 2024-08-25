#include <ArduinoNmeaParser.h>
#define RXD2 16
#define TXD2 17

void onRmcUpdate(nmea::RmcData const rmc) {
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
/* ... */
ArduinoNmeaParser parser(onRmcUpdate, nullptr);


void setup() {
  // Open serial communications
  Serial.begin(9600);
  Serial.println("Neo6M GPS module test code");
  // set the data rate for the SoftwareSerial port
  Serial2.begin(9600);
}

void loop() {
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }
}