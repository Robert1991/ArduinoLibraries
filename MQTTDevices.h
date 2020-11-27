#ifndef MQTTDevices_h
#define MQTTDevices_h

#include <ArduinoJson.h>
#include <IOTClients.h>
#include <wireUtils.h>

#include "Arduino.h"
#include "DHT.h"
#include "SerialLogger.h"

struct MQTTDeviceClassification {
  String sensorUniqueId;
  String deviceClass;
  String deviceType;
  String sensorType;
};

struct MQTTDeviceInfo {
  String uniqueId;
  String deviceName;
  String autoDiscoveryPrefix;
  String deviceType;
  String manufacturer;
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
  String sensorName;
  String autoDiscoveryMQTTConfigureTopic;
  String deviceClass;

  StaticJsonDocument<512> createDeviceInfoJsonObject();
  void assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo);
  int publishAutoDiscoveryInfo(StaticJsonDocument<512> jsonDocument);
  bool deregisterDeviceInOrigin();

 protected:
  String stateTopic;
  MessageQueueClient* mqttClient;

  int publishJsonDocument(String stateTopic, StaticJsonDocument<512> jsonDocument, bool retain = false);
  virtual StaticJsonDocument<512> extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument);

 public:
  MQTTDevice(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void configureViaBroker();
};

class MQTTSensor : public MQTTDevice {
 protected:
  bool areEqual(float value1, float value2, float maxDifference = 0.001);
  void publishFloatValue(float value);
  void publishBinaryMessage(bool on);

 public:
  MQTTSensor(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo);
  virtual void setupSensor();
  virtual void publishMeasurement() = 0;
  virtual void reset();
};

class MQTTDevicePingDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTDevicePingDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTDevicePing : public MQTTSensor {
 private:
  long pingTimeout;
  unsigned long startTime;
  bool pingWasSentInLastIteration = false;

  StaticJsonDocument<512> extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument);

 public:
  MQTTDevicePing(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String uniqueId, long pingTimeout = 60000);
  void publishMeasurement();
  void reset();
};

class MQTTPhotoLightSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTPhotoLightSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTPhotoLightSensor : public MQTTSensor {
 private:
  int analogPin;
  float lastVoltageValue = 0.0;

  StaticJsonDocument<512> extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument);

 public:
  MQTTPhotoLightSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin);

  void publishMeasurement();
  void reset();
};

class MQTTMotionSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTMotionSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTMotionSensor : public MQTTSensor {
 private:
  int motionSensorPin;
  bool lastMotionSensorState = false;
  int motionDetectionIterations;
  int motionDetectionTimeout;
  int motionDetectedThreshold;

 public:
  MQTTMotionSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin,
                   int motionDetectionIterations = 10, int motionDetectionTimeout = 15, int motionDetectedThreshold = 5);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

class MQTTDHTSensor : public MQTTSensor {
 private:
  int currentIteration = 0;
  int resetValuesIterations;

 protected:
  DHT* dhtSensor;
  bool resetIfRequired();

 public:
  MQTTDHTSensor(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo, DHT* dhtSensor,
                int resetValuesIterations = 5000);
  void setupSensor();
  virtual void publishMeasurement() = 0;
  virtual void reset() = 0;
};

class MQTTTemperatureSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTTemperatureSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTHumiditySensor : public MQTTDHTSensor {
 private:
  DHT* dhtSensor;
  float lastMeasuredHumidity = 0.0;

  void publishHumidity();

 public:
  MQTTHumiditySensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId,
                     int resetValuesIterations = 5000);
  void publishMeasurement();
  void reset();
};

class MQTTTemperatureSensor : public MQTTDHTSensor {
 private:
  DHT* dhtSensor;
  float lastMeasuredTemperature = 0.0;

  void publishTemperature();

 public:
  MQTTTemperatureSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId,
                        int resetValuesIterations = 5000);
  StaticJsonDocument<512> extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument);
  void publishMeasurement();
  void reset();
};

class MQTTHumiditySensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTHumiditySensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

struct MQTTSwitchConfiguration {
  char* switchSubscriptionTopic;
  char* switchStateTopic;
  int switchPin;
};

class MQTTActor : public SerialLogger {
  virtual void setupActor(MessageQueueClient* mqttClient) = 0;
  virtual bool consumeMessage(MessageQueueClient* mqttClient, String topic, String payload) = 0;
  virtual void executeDefaultAction(MessageQueueClient* mqttClient) = 0;
};

class MQTTSwitch : public MQTTActor {
 private:
  String SWITCH_PAYLOAD_ON = "ON";
  String SWITCH_PAYLOAD_OFF = "OFF";
  MQTTSwitchConfiguration configuration;
  MessageQueueClient* mqttClient;
  bool switchOn = false;

 public:
  MQTTSwitch(MQTTSwitchConfiguration configuration);

  void setupActor(MessageQueueClient* mqttClient);
  void applySwitchStatus();
  bool consumeMessage(MessageQueueClient* mqttClient, String topic, String payload);
  void executeDefaultAction(MessageQueueClient* mqttClient);
};

struct RGBPins {
  int red;
  int green;
  int blue;
};

struct MQTTRgbLightConfiguration {
  RGBPins pins;
  char* lightStateTopic;
  char* lightSwitchSubscriptionTopic;
  char* brightnessSwitchSubscriptionTopic;
  char* brightnessStateTopic;
  char* colorSetSubscriptionTopic;
  char* colorSetStateTopic;
};

class MQTTRgbLight : public MQTTActor {
 private:
  MQTTRgbLightConfiguration configuration;
  volatile bool stripOn = false;
  double currentBrightness = 0.0;
  volatile int redColorPart = 255;
  volatile int greenColorPart = 0;
  volatile int blueColorPart = 0;

  void reportStatus(MessageQueueClient* mqttClient);
  void processBrightnessPayload(String payload);
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTRgbLight(MQTTRgbLightConfiguration configuration);

  void setupActor(MessageQueueClient* mqttClient);
  bool consumeMessage(MessageQueueClient* mqttClient, String topic, String payload);
  void applyChoosenColorToLeds();
  void executeDefaultAction(MessageQueueClient* mqttClient);
};

struct WirePinSet {
  int i2cSecondaryAddress;
  int sdaPin;
  int sclPin;
};

struct MQTTRgbLightI2CCommands {
  int setOnOffCommand;
  int setColorCommand;
  int setBrightnessCommand;
};

struct MQTTI2CRgbLightConfiguration {
  MQTTRgbLightI2CCommands i2cConnectionCommands;
  WirePinSet wirePins;
  char* lightStateTopic;
  char* lightSwitchSubscriptionTopic;
  char* brightnessSwitchSubscriptionTopic;
  char* brightnessStateTopic;
  char* colorSetSubscriptionTopic;
  char* colorSetStateTopic;
};

class MQTTI2CRgbLight : public MQTTActor {
 private:
  bool stripOn = false;
  int currentBrightness = 0;
  int redColorPart = 0;
  int greenColorPart = 0;
  int blueColorPart = 0;

  bool colorChanged = true;
  bool onOffStatusChanged = true;
  bool brightnessChanged = true;

  MQTTI2CRgbLightConfiguration lightConfiguration;
  void sendRGBValuesToSecondary(byte redValue, byte greenValue, byte blueValue, int delayTime = 75);
  void refreshI2CConnection();
  void reportStatus(MessageQueueClient* mqttClient);
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTI2CRgbLight(MQTTI2CRgbLightConfiguration lightConfiguration);
  void setupActor(MessageQueueClient* mqttClient);
  bool consumeMessage(MessageQueueClient* mqttClient, String topic, String payload);
  void applyChoosenColorToLeds();

  void executeDefaultAction(MessageQueueClient* mqttClient);
};

#endif