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
  bool deviceClassIsAvailable;
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
  String autoDiscoveryMQTTConfigureTopic;
  String deviceClass;

  DynamicJsonDocument createDeviceInfoJsonObject();
  void assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo);
  int publishAutoDiscoveryInfo(DynamicJsonDocument jsonDocument);
  bool deregisterDeviceInOrigin();

 protected:
  String deviceEntityName;
  String stateTopic;
  MessageQueueClient* mqttClient;

  int publishState(String stateTopicPayload);
  int publishTo(String stateTopic, String stateTopicPayload, bool retain = false);
  int publishJsonDocument(String stateTopic, DynamicJsonDocument jsonDocument, bool retain = false);
  void subscribeTopic(String subscribeTopic);
  virtual DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);

 public:
  MQTTDevice(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void configureViaBroker();
  virtual void reset() = 0;
};

class MQTTSensor : public MQTTDevice, public MQTTPublisher {
 protected:
  bool areEqual(float value1, float value2, float maxDifference = 0.001);
  void publishFloatValue(float value);
  void publishBinaryMessage(bool on);

 public:
  MQTTSensor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void initializePublisher(MessageQueueClient* mqttClient);
  void configureInTargetPlatform();
  void publishToTargetPlatform();
  virtual void setupSensor();
  virtual void publishMeasurement() = 0;
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

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);

 public:
  MQTTDevicePing(MQTTDeviceInfo deviceInfo, String uniqueId, long pingTimeout = 60000);
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

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);

 public:
  MQTTPhotoLightSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin);

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
  MQTTMotionSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin, int motionDetectionIterations = 10,
                   int motionDetectionTimeout = 15, int motionDetectedThreshold = 5);
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
  MQTTDHTSensor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo, DHT* dhtSensor, int resetValuesIterations = 5000);
  void setupSensor();
  virtual void publishMeasurement() = 0;
  virtual void reset() = 0;
};

class MQTTTemperatureSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTTemperatureSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTHumiditySensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTHumiditySensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTHumiditySensor : public MQTTDHTSensor {
 private:
  DHT* dhtSensor;
  float lastMeasuredHumidity = 0.0;

  void publishHumidity();

 public:
  MQTTHumiditySensor(MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId, int resetValuesIterations = 5000);
  void publishMeasurement();
  void reset();
};

class MQTTTemperatureSensor : public MQTTDHTSensor {
 private:
  DHT* dhtSensor;
  float lastMeasuredTemperature = 0.0;

  void publishTemperature();

 public:
  MQTTTemperatureSensor(MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId, int resetValuesIterations = 5000);
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void publishMeasurement();
  void reset();
};

class MQTTActor : public MQTTDevice, public MQTTStateConsumer {
 protected:
  bool actorStatusChanged = false;
  String commandTopic;
  String brightnessCommandTopic;

  virtual void reportStatusInformation() = 0;
  void reportStatus();

 public:
  MQTTActor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo);
  void initializePublisher(MessageQueueClient* mqttClient);
  void configureInTargetPlatform();
  void publishToTargetPlatform();
  void reset();
  virtual void setupActor() = 0;
  virtual void executeLoopMethod();
};

class MQTTSwitchDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 private:
  String deviceName;
  String deviceType;

 public:
  MQTTSwitchDeviceClassificationFactory(String deviceUniqueId, String deviceType, String deviceName);
  MQTTDeviceClassification create();
};

class MQTTSwitch : public MQTTActor {
 private:
  String SWITCH_PAYLOAD_ON = "ON";
  String SWITCH_PAYLOAD_OFF = "OFF";
  int switchPin;

 protected:
  bool switchOn = false;
  void reportStatusInformation();

 public:
  MQTTSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, int switchPin, String deviceName = "relais_switch", String deviceType = "light");

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  virtual void setupActor();
  virtual void executeLoopMethod();
  void setupSubscriptions();
  bool consumeMessage(String topic, String payload);
};

class MQTTDeviceResetSwitch : public MQTTSwitch {
 public:
  MQTTDeviceResetSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, String deviceName = "reset_switch");
  void executeLoopMethod();
  void setupActor();
};

struct RGBPins {
  int red;
  int green;
  int blue;
};

struct MQTTRgbLightConfiguration {};

class MQTTRgbLightDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTRgbLightDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTRgbLight : public MQTTActor {
 private:
  RGBPins pins;
  volatile bool stripOn = false;
  double currentBrightness = 0.0;
  volatile int redColorPart = 255;
  volatile int greenColorPart = 0;
  volatile int blueColorPart = 0;

  String brightnessStateTopic;
  String brightnessCommandTopic;
  String colorCommandTopic;
  String colorStateTopic;

  void reportStatusInformation();
  void processBrightnessPayload(String payload);
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTRgbLight(MQTTDeviceInfo deviceInfo, String uniqueId, RGBPins pins);

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void setupActor();
  void setupSubscriptions();
  bool consumeMessage(String topic, String payload);
  void applyChoosenColorToLeds();
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
};

class MQTTI2CRgbLightDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
 public:
  MQTTI2CRgbLightDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTI2CRgbLight : public MQTTActor {
 private:
  bool stripOn = false;
  int currentBrightness = 0;
  int redColorPart = 0;
  int greenColorPart = 0;
  int blueColorPart = 0;

  String brightnessStateTopic;
  String brightnessCommandTopic;
  String colorCommandTopic;
  String colorStateTopic;

  bool colorChanged = true;
  bool onOffStatusChanged = true;
  bool brightnessChanged = true;

  MQTTI2CRgbLightConfiguration lightConfiguration;
  void sendRGBValuesToSecondary(byte redValue, byte greenValue, byte blueValue, int delayTime = 75);
  void refreshI2CConnection();
  void reportStatusInformation();
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTI2CRgbLight(MQTTDeviceInfo deviceInfo, String uniqueId, MQTTI2CRgbLightConfiguration lightConfiguration);
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void setupActor();
  void setupSubscriptions();
  bool consumeMessage(String topic, String payload);
  void applyChoosenColorToLeds();
};

#endif