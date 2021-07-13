#include "Arduino.h"
#include "logDebug.h"

void logDebug(String msg)
{
  if (logDebugObj::isDebugActive)
    Serial.println(msg);
  delay(100);
}