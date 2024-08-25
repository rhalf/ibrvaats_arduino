#ifndef _MPU6050_H
#define _MPU6050_H

#include "Arduino.h"

/// @author Rhalf Wendel D Caacbay <rhalfcaacbay@gmail.com>
typedef void (* Callback)();

class Mpu6050 {

public:

  static const uint8_t MPU_ADDRESS = 0b1101000;
  static const uint8_t POWER_ADDRESS = 0x6B;
  static const uint8_t POWER_SETTING = 0b00000000;
  static const uint8_t GYRO_ADDRESS = 0x1B;
  static const uint8_t GYRO_SETTING = 0x00000000;
  static const uint8_t GYRO_ACCESS = 0x43;
  static const uint8_t GYRO_REQUEST = 0b1101000;
  static const uint8_t ACCELEROMETER_ADDRESS = 0x1C;
  static const uint8_t ACCELEROMETER_SETTING = 0x00000000;
  static const uint8_t ACCELEROMETER_ACCESS = 0x3B;
  
  static const uint8_t REQUEST = 0b1101000;
  static const uint8_t BYTES_TO_READ = 6;

  double gForceX, gForceY, gForceZ;
  double rotX = 0;
  double rotY = 0;
  double rotZ = 0;


  Mpu6050();

  void initialized();
  void readData();
  void readGyroscopeData();
  void readAccelerometerData();

private:


  Callback _callback = NULL;
};
#endif // _MPU6050_H
