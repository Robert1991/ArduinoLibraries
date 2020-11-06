
#ifndef SerialLogger_h
#define SerialLogger_h

#include "Arduino.h"

class SerialLogger {
  bool verboseLoggingIsEnabled = false;

 protected:
  void logLineToSerial(double value) {
    if (verboseLoggingIsEnabled) {
      Serial.println(value);
    }
  };

  void logLineToSerial(int value) {
    if (verboseLoggingIsEnabled) {
      Serial.println(value);
    }
  };

  void logLineToSerial(char* message) {
    if (verboseLoggingIsEnabled) {
      Serial.println(message);
    }
  };

  void logToSerial(char* message) {
    if (verboseLoggingIsEnabled) {
      Serial.print(message);
    }
  };

  void logToSerial(int value) {
    if (verboseLoggingIsEnabled) {
      Serial.print(value);
    }
  };

  void logToSerial(double value) {
    if (verboseLoggingIsEnabled) {
      Serial.print(value);
    }
  };

 public:
  void setVerbose(bool enableVerboseLogging) { this->verboseLoggingIsEnabled = enableVerboseLogging; }
};

#endif