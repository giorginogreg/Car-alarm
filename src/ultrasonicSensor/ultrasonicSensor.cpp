#include "ultrasonicSensor.h"

void setupUS() {
  pinMode(US_TRIG_PIN, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(US_ECHO_PIN, INPUT);  // Sets the echoPin as an INPUT
}

// Ritorna la distanza in cm
int getDistance() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
  return pulseIn(US_ECHO_PIN, HIGH) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}