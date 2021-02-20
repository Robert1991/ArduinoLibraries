#include <DeviceUpdateService.h>

DeviceUpdateService::DeviceUpdateService(WiFiClient &client, String updateServerUrl, int port,
                                         String currentDeviceVersion) {
  this->client = client;
  this->updateServerUrl = updateServerUrl;
  this->port = port;
  this->currentDeviceVersion = currentDeviceVersion;
}

void DeviceUpdateService::setup() {
  ESPhttpUpdate.onStart(DeviceUpdateService::updateStarted);
  ESPhttpUpdate.onEnd(DeviceUpdateService::updateFinished);
  ESPhttpUpdate.onProgress(DeviceUpdateService::updateProgress);
  ESPhttpUpdate.onError(DeviceUpdateService::updateError);
}

void DeviceUpdateService::installUpdateIfPossible() {
  t_httpUpdate_return returnCode =
      ESPhttpUpdate.update(client, updateServerUrl, port, (String) "", currentDeviceVersion);
  switch (returnCode) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(),
                  ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void DeviceUpdateService::updateStarted() { Serial.println("CALLBACK:  HTTP update process started"); }

void DeviceUpdateService::updateFinished() { Serial.println("CALLBACK:  HTTP update process finished"); }

void DeviceUpdateService::updateError(int errorCode) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", errorCode);
}

void DeviceUpdateService::updateProgress(int current, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", current, total);
}
