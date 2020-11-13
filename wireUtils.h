#ifndef wireUtils_h
#define wireUtils_h

#include <Wire.h>

#include "Arduino.h"
#include "hardwareSerial.h"

// see http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
int clearI2CBus(int sdaPin, int sclPin, bool startupDelay = false);

bool checkI2CConnection();

void establishI2CConnectionTo(int sdaPort, int sclPort, bool startupDelay = false);

void sendI2CCommandWithParameter(int secondaryAddress, byte command, byte parameter, int delayTime = 75);

#endif
