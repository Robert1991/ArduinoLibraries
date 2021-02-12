#include "wireUtils.h"

int clearI2CBus(int sdaPin, int sclPin, bool startupDelay) {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); // Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(sdaPin, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(sclPin, INPUT_PULLUP);

  if (startupDelay) {
    delay(2500); // Wait 2.5 secs. This is strictly only necessary on the first power
    // up of the DS3231 module to allow it to initialize properly,
    // but is also assists in reliable programming of FioV3 boards as it gives the
    // IDE a chance to start uploaded the program
    // before existing sketch confuses the IDE by sending Serial data.
  }

  boolean sclPinIsLow = (digitalRead(sclPin) == LOW); // Check is SCL is Low.
  if (sclPinIsLow) { // If it is held low Arduno cannot become the I2C master.
    return 1;        // I2C bus error. Could not clear SCL clock line held low
  }

  boolean sdaPinIsLow = (digitalRead(sdaPin) == LOW); // vi. Check SDA input.
  int clockCount = 20;                                // > 2x9 clock

  while (sdaPinIsLow && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(sclPin, INPUT);        // release SCL pullup so that when made output it will be LOW
    pinMode(sclPin, OUTPUT);       // then clock SCL Low
    delayMicroseconds(10);         //  for >5uS
    pinMode(sclPin, INPUT);        // release SCL LOW
    pinMode(sclPin, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    sclPinIsLow = (digitalRead(sclPin) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (sclPinIsLow && (counter > 0)) { //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      sclPinIsLow = (digitalRead(sclPin) == LOW);
    }
    if (sclPinIsLow) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    sdaPinIsLow = (digitalRead(sdaPin) == LOW); //   and check SDA input again and loop
  }
  if (sdaPinIsLow) { // still low
    return 3;        // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(sdaPin, INPUT);  // remove pullup.
  pinMode(sdaPin, OUTPUT); // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the
  // bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10);         // wait >5uS
  pinMode(sdaPin, INPUT);        // remove output low
  pinMode(sdaPin, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10);         // x. wait >5uS
  pinMode(sdaPin, INPUT);        // and reset pins as tri-state inputs which is the default state on reset
  pinMode(sclPin, INPUT);
  return 0; // all ok
}

bool checkI2CConnection(int sdaPin, int sclPin) {
  bool connectionWasLost = false;
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  if (digitalRead(sdaPin) != 1) {
    Serial.println("SDA pin is low");
    connectionWasLost = true;
  }
  if (digitalRead(sclPin) != 1) {
    Serial.println("SCL pin is low");
    connectionWasLost = true;
  }
  return connectionWasLost;
}

void establishI2CConnectionTo(int sdaPort, int sclPort, bool startupDelay) {
  int rtn = clearI2CBus(sdaPort, sclPort, startupDelay);
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else {
    Wire.begin(sdaPort, sclPort);
    Serial.print("Wire connection successfully established on: {SDA : ");
    Serial.print(sdaPort);
    Serial.print("} ");
    Serial.print("{SCL : ");
    Serial.print(sclPort);
    Serial.println("}");
  }
}

void sendI2CCommandWithParameter(int secondaryAddress, byte command, byte parameter, int delayTime) {
  Serial.print("Sending I2C command {");
  Serial.print(command);
  Serial.print("} with parameter {");
  Serial.print(parameter);
  Serial.println("}");
  Wire.beginTransmission(secondaryAddress);
  Wire.write(command);
  Wire.write(parameter);
  Wire.endTransmission();
  delay(delayTime);
}
