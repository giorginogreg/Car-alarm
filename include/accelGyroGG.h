#ifndef ACCELL_GYRO_H
#define ACCELL_GYRO_H

#define DEBUG false
void setupAccel();
void updateValues();
bool movementDetected();

#include <Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
#endif