#include "Mpu6050.h"
#include "Arduino.h"
#include <Wire.h>

typedef void (* Callback)();


Mpu6050::Mpu6050(){

}

void Mpu6050::initialized() { 
    Wire.begin();

    //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.beginTransmission(MPU_ADDRESS); 
    //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(POWER_ADDRESS);
    //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.write(POWER_SETTING);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDRESS); //I2C address of the MPU
    Wire.write(GYRO_ACCESS); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
    Wire.write(GYRO_SETTING); //Setting the gyro to full scale +/- 250deg./s 
    Wire.endTransmission(); 

    Wire.beginTransmission(MPU_ADDRESS); //I2C address of the MPU
    Wire.write(ACCELEROMETER_ACCESS); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
    Wire.write(ACCELEROMETER_SETTING); //Setting the accel to +/- 2g
    Wire.endTransmission(); 
}

void Mpu6050::readAccelerometerData() { 
  //I2C address of the MPU
   Wire.beginTransmission(MPU_ADDRESS); 
   //Starting register for Accel Readings
  Wire.write(ACCELEROMETER_ACCESS); 
  Wire.endTransmission();

  long accelX, accelY, accelZ;
  //Request Accel Registers (3B - 40)
  Wire.requestFrom(REQUEST, BYTES_TO_READ); 
  while(Wire.available() < BYTES_TO_READ);
    accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
    accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
    accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void Mpu6050::readGyroscopeData() { 
  //I2C address of the MPU
  Wire.beginTransmission(MPU_ADDRESS); 
  //Starting register for Gyro Readings
  Wire.write(GYRO_ACCESS); 
  Wire.endTransmission();

  long gyroX, gyroY, gyroZ;
//Request Gyro Registers (43 - 48)
  Wire.requestFrom(REQUEST, BYTES_TO_READ); 
  while(Wire.available() < BYTES_TO_READ);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void Mpu6050::readData() {
  readGyroscopeData();
  readAccelerometerData();
} 