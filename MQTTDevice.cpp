#include "MQTTDevice.h"

// Device
MQTTDeviceClassificationFactory::MQTTDeviceClassificationFactory(String deviceUniqueId) { this->deviceUniqueId = deviceUniqueId; }

MQTTDevice::MQTTDevice(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) {
  MQTTDeviceClassification deviceClass = deviceClassFactory->create();
  assignDeviceInfos(deviceClass, deviceInfo);
}

void MQTTDevice::assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo) {
  this->deviceEntityName = deviceInfo.deviceName + "_" + deviceClass.sensorType;
  String sensorHomeAssistantPath = deviceInfo.autoDiscoveryPrefix + "/" + deviceClass.deviceType + "/" + deviceEntityName;
  this->stateTopic = sensorHomeAssistantPath + "/state";
  this->autoDiscoveryMQTTConfigureTopic = sensorHomeAssistantPath + "/config";
  this->deviceClassification = deviceClass;
  this->deviceInfo = deviceInfo;
}

int MQTTDevice::publishAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  return publishJsonDocument(autoDiscoveryMQTTConfigureTopic, autoConfigureJsonDocument, true);
}

int MQTTDevice::publishTo(String topic, String payload, bool retain) { return mqttClient->publishMessage(topic, payload, retain); }

int MQTTDevice::publishState(String stateTopicPayload) { return publishTo(stateTopic, stateTopicPayload); }

int MQTTDevice::publishJsonDocument(String stateTopic, DynamicJsonDocument jsonDocument, bool retain) {
  String jsonMessage;
  serializeJson(jsonDocument, jsonMessage);
  logToSerial("Publishing to: ");
  logToSerial(stateTopic);
  logLineToSerial(" with payload: ");
  logLineToSerial(jsonMessage);
  mqttClient->publishMessage(stateTopic, jsonMessage, retain);
  return 1;
}

void MQTTDevice::subscribeTopic(String subscribeTopic) { return mqttClient->subscribeTopic(subscribeTopic); }

DynamicJsonDocument MQTTDevice::createDeviceInfoJsonObject() {
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(12) + 540;
  DynamicJsonDocument autoConfigureJsonDocument(capacity);
  autoConfigureJsonDocument["name"] = deviceEntityName;
  if (deviceClassification.deviceClassIsAvailable) {
    autoConfigureJsonDocument["dev_cla"] = deviceClassification.deviceClass;
  }
  autoConfigureJsonDocument["stat_t"] = stateTopic;
  autoConfigureJsonDocument["uniq_id"] = deviceClassification.sensorUniqueId;
  autoConfigureJsonDocument["dev"]["ids"][0] = deviceInfo.uniqueId;
  autoConfigureJsonDocument["dev"]["name"] = deviceInfo.deviceName;
  autoConfigureJsonDocument["dev"]["mdl"] = deviceInfo.deviceType;
  autoConfigureJsonDocument["dev"]["mf"] = deviceInfo.manufacturer;
  return autoConfigureJsonDocument;
}

void MQTTDevice::configureViaBroker() {
  if (deregisterDeviceInOrigin()) {
    logLineToSerial("Device deregistered!");
    delay(500);

    DynamicJsonDocument autoConfigureJsonDocument = createDeviceInfoJsonObject();
    autoConfigureJsonDocument = extendAutoDiscoveryInfo(autoConfigureJsonDocument);
    if (publishAutoDiscoveryInfo(autoConfigureJsonDocument)) {
      logLineToSerial("Configure message successfully sent!");
    }
  }
}

bool MQTTDevice::deregisterDeviceInOrigin() {
  String emptyConfigureMessage = "";
  return mqttClient->publishMessage(autoDiscoveryMQTTConfigureTopic, emptyConfigureMessage, true);
}

DynamicJsonDocument MQTTDevice::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) { return autoConfigureJsonDocument; }

// Sensor
MQTTSensor::MQTTSensor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) : MQTTDevice(deviceClassFactory, deviceInfo) {}

void MQTTSensor::initializePublisher(MessageQueueClient* mqttClient) {
  this->mqttClient = mqttClient;
  setupSensor();
}

void MQTTSensor::configureInTargetPlatform() { configureViaBroker(); }

void MQTTSensor::publishToTargetPlatform() { publishMeasurement(); }

void MQTTSensor::publishFloatValue(float floatValue) {
  char result[8];
  dtostrf(floatValue, 6, 2, result);
  mqttClient->publishMessage(stateTopic, result);
}

void MQTTSensor::publishBinaryMessage(bool on) {
  if (on) {
    String onPayload = "ON";
    mqttClient->publishMessage(stateTopic, onPayload);
  } else {
    String offPayload = "OFF";
    mqttClient->publishMessage(stateTopic, offPayload);
  }
}

void MQTTSensor::setupSensor() {}

bool MQTTSensor::areEqual(float value1, float value2, float maxDifference) {
  if (fabs(value1 - value2) > maxDifference) {
    return false;
  }
  return true;
}

// Actor
MQTTActor::MQTTActor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) : MQTTDevice(deviceClassFactory, deviceInfo) {}

void MQTTActor::configureInTargetPlatform() { configureViaBroker(); }

void MQTTActor::initializePublisher(MessageQueueClient* mqttClient) {
  this->mqttClient = mqttClient;
  setupActor();
}

void MQTTActor::publishToTargetPlatform() { reportStatus(); }

void MQTTActor::executeLoopMethod() {}

void MQTTActor::reportStatus() {
  if (actorStatusChanged) {
    reportStatusInformation();
    actorStatusChanged = false;
  }
}

void MQTTActor::reset() { actorStatusChanged = true; }