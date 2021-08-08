#ifndef ACCELL_GYRO_H
#define ACCELL_GYRO_H
#include <I2Cdev.h>

#include "MPU6050.h"

class MyGyro
{
public:
    MPU6050 accelgyro;
    float currentaX, currentaY, currentaZ;
    float initXval, initYval, initZval;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float sogliaSpinta = 0.5;

    // Prototypes
    void setupGyro();
    void readAndUpdateValues();
    void updateAccellGyro();
    bool movementDetected();
};

#endif