#include <DeviceRootConfig.h>

String DeviceRootConfig::getCleanedDeviceName() {
  String deviceNameCleaned = deviceName;
  deviceNameCleaned.replace(" ", "_");
  deviceNameCleaned.toLowerCase();
  return deviceNameCleaned;
}

boolean DeviceRootConfig::updateServerSet() { return updateServer.length() > 0; }

boolean DeviceRootConfig::read() {
  if (fileExists(WIFI_CONFIG_PATH) && fileExists(MQTT_CONFIG_PATH)) {
    DynamicJsonDocument wifiConfig = deserializeFileContent(WIFI_CONFIG_PATH, 256);

    this->wifiSSID = (const char *)wifiConfig["ssid"];
    this->wifiPasswd = (const char *)wifiConfig["pass"];

    DynamicJsonDocument mqttConfig = deserializeFileContent(MQTT_CONFIG_PATH, 512);

    this->mqttBroker = (const char *)mqttConfig["mqtt_broker"];
    this->mqttBrokerPort = mqttConfig["mqtt_brokerport"];
    this->mqttUser = (const char *)mqttConfig["mqtt_user"];
    this->mqttPasswd = (const char *)mqttConfig["mqtt_pass"];
    this->deviceName = (const char *)mqttConfig["device_name"];
    this->homeassistantAutoConfigurePrefix = (const char *)mqttConfig["auto_conf_prefix"];
    this->updateServer = (const char *)mqttConfig["update_server"];
    this->updateServerPort = mqttConfig["update_server_port"];

    return true;
  }
  return false;
}

boolean DeviceRootConfig::writeCurrent() {
  if (fileExists(WIFI_CONFIG_PATH) || fileExists(MQTT_CONFIG_PATH)) {
    deleteConfig();
  }

  DynamicJsonDocument wifiConfig(256);
  wifiConfig["ssid"] = wifiSSID;
  wifiConfig["pass"] = wifiPasswd;

  writeJsonDocumentToFile(WIFI_CONFIG_PATH, wifiConfig);

  DynamicJsonDocument mqttConfig(512);
  mqttConfig["mqtt_broker"] = mqttBroker;
  mqttConfig["mqtt_brokerport"] = mqttBrokerPort;
  mqttConfig["mqtt_user"] = mqttUser;
  mqttConfig["mqtt_pass"] = mqttPasswd;
  mqttConfig["device_name"] = deviceName;
  mqttConfig["auto_conf_prefix"] = homeassistantAutoConfigurePrefix;
  mqttConfig["update_server"] = updateServer;
  mqttConfig["update_server_port"] = updateServerPort;

  writeJsonDocumentToFile(MQTT_CONFIG_PATH, mqttConfig);

  return true;
}

DynamicJsonDocument DeviceRootConfig::deserializeFileContent(String path, int jsonDocCapacity) {
  DynamicJsonDocument jsonDoc(jsonDocCapacity);
  String jsonContent = readFile(path);
  DeserializationError error = deserializeJson(jsonDoc, jsonContent);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }
  return jsonDoc;
}

void DeviceRootConfig::writeJsonDocumentToFile(String path, DynamicJsonDocument contentDocument) {
  String jsonStingContent;
  serializeJson(contentDocument, jsonStingContent);
  Serial.println("Writing: " + jsonStingContent);
  writeFile(path, jsonStingContent);
}

boolean DeviceRootConfig::deleteConfig() {
  return deleteFile(WIFI_CONFIG_PATH) && deleteFile(MQTT_CONFIG_PATH);
}

MQTTDeviceService *DeviceRootConfig::createMQTTDeviceService(WiFiClient &wifiClient,
                                                             MQTTClientCallbackSimple messageReceivedCallback,
                                                             boolean verbose, int publisherCacheSize,
                                                             int subscriberCacheSize, int mqttCacheSize) {
  MessageQueueClient *messageQueueClient =
      createMessageQueueClient(wifiClient, messageReceivedCallback, verbose, mqttCacheSize);
  return new MQTTDeviceService(messageQueueClient, publisherCacheSize, subscriberCacheSize);
}

MessageQueueClient *DeviceRootConfig::createMessageQueueClient(WiFiClient &wifiClient,
                                                               MQTTClientCallbackSimple messageReceived,
                                                               boolean verbose, int cacheSize) {
  MessageQueueClient *mqttClient = new MessageQueueClient(getCleanedDeviceName(), mqttUser, mqttPasswd);
  mqttClient->setupClient(wifiClient, messageReceived, mqttBroker, mqttBrokerPort, cacheSize);
  mqttClient->setVerbose(verbose);
  return mqttClient;
}

MQTTDeviceInfo DeviceRootConfig::createMQTTDeviceInfo(const String deviceId, const int buildNumber,
                                                      const String deviceType) {
  String deviceVersion = VERSION + "-" + buildNumber;
  MQTTDeviceInfo deviceInfo = {
      deviceId,   getCleanedDeviceName(), deviceName,   homeassistantAutoConfigurePrefix,
      deviceType, MANUFACTURER,           deviceVersion};
  return deviceInfo;
}
