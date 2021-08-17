#include "accelGyroGG.h"
int16_t AcX, AcY, AcZ;
float currentaX, currentaY, currentaZ;
float initXval, initYval, initZval;

float sogliaSpinta = 0.15;

void setupAccel()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  do
  {
    Serial.println("--- FIRST READ ---");
    updateValues();
    initXval = currentaX;
    initYval = currentaY;
    initZval = currentaZ;
  } while (AcX == 0 || AcY == 0 || AcZ == 0);
}

void updateValues()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  currentaX = AcX / 16384.;
  currentaY = AcY / 16384.;
  currentaZ = AcZ / 16384.;
  if (DEBUG)
  {
    Serial.print("AcX = ");
    Serial.println(currentaX - initXval);
    Serial.print(" | AcY = ");
    Serial.println(currentaY - initYval);
    Serial.print(" | AcZ = ");
    Serial.println(currentaZ - initZval);
  }

  delay(333);
}

bool movementDetected()
{
  return (abs(currentaX - initXval) > sogliaSpinta) ||
         (abs(currentaY - initYval) > sogliaSpinta) ||
         (abs(currentaZ - initZval) > sogliaSpinta);
}