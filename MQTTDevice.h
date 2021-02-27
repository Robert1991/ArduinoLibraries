#ifndef MQTTDevice_h
#define MQTTDevice_h

#include <ArduinoJson.h>
#include <IOTClients.h>

#include "Arduino.h"
#include "SerialLogger.h"

struct MQTTDeviceClassification {
  String sensorUniqueId;
  String deviceClass;
  String deviceType;
  String sensorType;
  bool deviceClassIsAvailable;
};

struct MQTTDeviceInfo {
  String uniqueId;
  String deviceName;
  String deviceFriendlyName;
  String autoDiscoveryPrefix;
  String deviceType;
  String manufacturer;
  String firmwareVersion;
};

class MQTTDeviceClassificationFactory {
protected:
  String deviceUniqueId;

public:
  MQTTDeviceClassificationFactory(String deviceUniqueId);
  virtual MQTTDeviceClassification create() = 0;
};

class MQTTDevice : public SerialLogger {
private:
  MQTTDeviceInfo deviceInfo;
  MQTTDeviceClassification deviceClassification;
  String autoDiscoveryMQTTConfigureTopic;
  String deviceClass;

  DynamicJsonDocument createDeviceInfoJsonObject();
  void assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo);
  int publishAutoDiscoveryInfo(DynamicJsonDocument jsonDocument);

protected:
  String sensorHomeAssistantPath;
  String deviceEntityName;
  String stateTopic;
  MessageQueueClient *mqttClient;

  int publishState(String stateTopicPayload);
  int publishTo(String stateTopic, String stateTopicPayload, bool retain = false);
  int publishJsonDocument(String stateTopic, DynamicJsonDocument jsonDocument, bool retain = false);
  void subscribeTopic(String subscribeTopic);
  virtual DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  DynamicJsonDocument createJsonDocument(int capacity);

public:
  MQTTDevice(MQTTDeviceClassificationFactory *deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void configureViaBroker();
  virtual void reset() = 0;
};

class MQTTSensor : public MQTTDevice, public MQTTPublisher {
protected:
  bool areEqual(float value1, float value2, float maxDifference = 0.001);
  void publishFloatValue(float value);
  void publishBinaryMessage(bool on);

public:
  MQTTSensor(MQTTDeviceClassificationFactory *deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void initializePublisher(MessageQueueClient *mqttClient);
  void configureInTargetPlatform();
  void publishToTargetPlatform();
  virtual void setupSensor();
  virtual void publishMeasurement() = 0;
};

class MQTTActor : public MQTTDevice, public MQTTStateConsumer {
protected:
  bool actorStatusChanged = false;
  String commandTopic;
  String brightnessCommandTopic;

  virtual void reportStatusInformation() = 0;
  void reportStatus();

public:
  MQTTActor(MQTTDeviceClassificationFactory *deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void initializePublisher(MessageQueueClient *mqttClient);
  void configureInTargetPlatform();
  void publishToTargetPlatform();
  void reset();
  virtual void setupActor() = 0;
  virtual void executeLoopMethod();
};

#endif