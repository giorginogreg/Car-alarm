#include "accelGyroGG.h"
#include "logDebug.h"

void MyGyro::setupGyro()
{
  logDebug("Inizializzo il gyroscopio...");
  accelgyro.initialize();
  accelgyro.setAccelerometerPowerOnDelay(3);
  accelgyro.setDHPFMode(1);
  accelgyro.setMotionDetectionThreshold(2);
  accelgyro.setMotionDetectionDuration(40);
  accelgyro.setZeroMotionDetectionDuration(1);
  initXval = currentaX;
  initYval = currentaY;
  initZval = currentaZ;
  logDebug("Gyroscopio inizializzato correttamente!");
}

void MyGyro::updateAccellGyro()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.getAcceleration(&ax, &ay, &az);

  readAndUpdateValues();
}

void MyGyro::readAndUpdateValues()
{
  currentaX = ax / 16384.;
  currentaY = ay / 16384.;
  currentaZ = az / 16384.;
}

bool MyGyro::movementDetected()
{
  return (abs(currentaX) > abs(initXval) + sogliaSpinta) ||
         (abs(currentaY) > abs(initYval) + sogliaSpinta) ||
         (abs(currentaZ) > abs(initZval) + sogliaSpinta);
}