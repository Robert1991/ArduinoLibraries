#ifndef DeviceUpdateService_h
#define DeviceUpdateService_h

#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <SerialLogger.h>

class DeviceUpdateService {
private:
  WiFiClient client;
  String updateServerUrl;
  int port;
  String currentDeviceVersion;

public:
  DeviceUpdateService(WiFiClient &client, String updateServerUrl, int port, String currentDeviceVersion);
  void setup();
  void installUpdateIfPossible();

  static void updateStarted();
  static void updateFinished();
  static void updateError(int errorCode);
  static void updateProgress(int current, int total);
};

#endif
