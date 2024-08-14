#include "Mpu6050.h"
Mpu6050 mpu6050 = Mpu6050();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu6050.initialized();
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.readData();
  printData();

  delay(1000);
}


void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(mpu6050.rotX);
  Serial.print(" Y=");
  Serial.print(mpu6050.rotY);
  Serial.print(" Z=");
  Serial.print(mpu6050.rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(mpu6050.gForceX);
  Serial.print(" Y=");
  Serial.print(mpu6050.gForceY);
  Serial.print(" Z=");
  Serial.println(mpu6050.gForceZ);
}