#ifndef DeviceRootConfig_h
#define DeviceRootConfig_h

#include "Arduino.h"
#include <ArduinoJson.h>
#include <ESPFileUtils.h>
#include <EasyButton.h>
#include <MQTTDevices.h>

const String WIFI_CONFIG_PATH = "/wifi_config.json";
const String MQTT_CONFIG_PATH = "/mqtt_config.json";

class DeviceRootConfig {
private:
  DynamicJsonDocument deserializeFileContent(String path, int jsonDocCapacity);
  void writeJsonDocumentToFile(String path, DynamicJsonDocument document);
  MessageQueueClient *createMessageQueueClient(WiFiClient &wifiClient,
                                               MQTTClientCallbackSimple messageReceivedCallback,
                                               boolean verbose = true, int cacheSize = 750);

public:
  String wifiSSID;
  String wifiPasswd;
  String mqttBroker;
  int mqttBrokerPort = 1883;
  String mqttUser;
  String mqttPasswd;
  String deviceName;
  String homeassistantAutoConfigurePrefix;

  String getCleanedDeviceName();
  boolean deviceWasConfigured();
  boolean read();
  boolean writeCurrent();
  boolean deleteConfig();
  MQTTDeviceService *createMQTTDeviceService(WiFiClient &wifiClient,
                                             MQTTClientCallbackSimple messageReceivedCallback,
                                             boolean verbose = true, int publisherCacheSize = 10,
                                             int subscriberCacheSize = 10, int mqttCacheSize = 750);
  MQTTDeviceInfo createMQTTDeviceInfo(const String deviceId, const String deviceType = "Node MCU");
};

#endif
