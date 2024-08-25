#ifndef _GPS_H
#define _GPS_H

#include "Arduino.h"
#include <ArduinoNmeaParser.h>

/// @author Rhalf Wendel D Caacbay <rhalfcaacbay@gmail.com>
typedef void (* Callback)();

class Gps {

public:

  static const uint8_t MPU_ADDRESS = 0b1101000;

  
  static const uint8_t REQUEST = 0b1101000;
  static const uint8_t BYTES_TO_READ = 6;

  double gForceX, gForceY, gForceZ;
  double rotX = 0;
  double rotY = 0;
  double rotZ = 0;

  ArduinoNmeaParser parser(onRmcUpdate, nullptr);

  Gps(uint8_t rx, uint8_t tx);
  void run();
  void onRmcUpdate(nmea::RmcData const rmc);

private:

  Callback _callback = NULL;
};
#endif // _GPS_H
