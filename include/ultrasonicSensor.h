#ifndef US_GG
#define US_GG

#include <SoftwareSerial.h>
#include <Arduino.h>

static const int US_TRIG_PIN = PB3, US_ECHO_PIN = PB4;
static const float sogliaSpinta = 0.5;
static const float sogliaDistanza = 10;

void setupUS();
int getDistance();

#endif