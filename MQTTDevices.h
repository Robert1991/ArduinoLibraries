#ifndef MQTTDevices_h
#define MQTTDevices_h

#include <IOTClients.h>
#include <wireUtils.h>

#include "Arduino.h"
#include "DHT.h"
#include "SerialLogger.h"

class MQTTSensor : public SerialLogger {
 private:
  MQTTClient* mqttClient;

 public:
  MQTTSensor(MQTTClient* mqttClient);
  virtual void setupSensor() = 0;
  virtual void publishMeasurement() = 0;
  virtual void reset() = 0;

 protected:
  void publishFloatValue(char* stateTopic, float value);
  void publishBinaryMessage(char* stateTopic, bool on);
};

class MQTTMotionSensor : public MQTTSensor {
 private:
  char* stateTopic;
  int motionSensorPin;
  bool lastMotionSensorState = false;

 public:
  MQTTMotionSensor(MQTTClient* mqttClient, char* stateTopic, int motionSensorPin);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

class MQTTDhtSensor : public MQTTSensor {
 private:
  DHT* dhtSensor;
  char* temperatureStateTopic;
  char* humidityStateTopic;
  float lastMeasuredTemperature = 0.0;
  float lastMeasuredHumidity = 0.0;

  bool areEqual(float value1, float value2);
  void publishTemperature();
  void publishHumidity();

 public:
  MQTTDhtSensor(MQTTClient* mqttClient, DHT* dhtSensor, char* temperatureStateTopic, char* humidityStateTopic);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

struct MQTTSwitchConfiguration {
  char* switchSubscriptionTopic;
  char* switchStateTopic;
  int switchPin;
};

class MQTTDevice : public SerialLogger {
  virtual void setupActor(MQTTClient* mqttClient) = 0;
  virtual bool consumeMessage(MQTTClient* mqttClient, String topic, String payload) = 0;
  virtual void executeDefaultAction(MQTTClient* mqttClient) = 0;
};

class MQTTSwitch : public MQTTDevice {
 private:
  char* SWITCH_PAYLOAD_ON = "ON";
  char* SWITCH_PAYLOAD_OFF = "OFF";
  MQTTSwitchConfiguration configuration;
  MQTTClient* mqttClient;
  bool switchOn = false;

 public:
  MQTTSwitch(MQTTSwitchConfiguration configuration);

  void setupActor(MQTTClient* mqttClient);
  void applySwitchStatus();
  bool consumeMessage(MQTTClient* mqttClient, String topic, String payload);
  void executeDefaultAction(MQTTClient* mqttClient);
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

class MQTTRgbLight : public MQTTDevice {
 private:
  MQTTRgbLightConfiguration configuration;
  volatile bool stripOn = false;
  double currentBrightness = 0.0;
  volatile int redColorPart = 255;
  volatile int greenColorPart = 0;
  volatile int blueColorPart = 0;

  void reportStatus(MQTTClient* mqttClient);
  void processBrightnessPayload(String payload);
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTRgbLight(MQTTRgbLightConfiguration configuration);

  void setupActor(MQTTClient* mqttClient);
  bool consumeMessage(MQTTClient* mqttClient, String topic, String payload);
  void applyChoosenColorToLeds();
  void executeDefaultAction(MQTTClient* mqttClient);
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

class MQTTI2CRgbLight : public MQTTDevice {
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
  void reportStatus(MQTTClient* mqttClient);
  void processColorCommandPayload(String payload);
  bool processIncomingMessage(String topic, String payload);

 public:
  MQTTI2CRgbLight(MQTTI2CRgbLightConfiguration lightConfiguration);
  void setupActor(MQTTClient* mqttClient);
  bool consumeMessage(MQTTClient* mqttClient, String topic, String payload);
  void applyChoosenColorToLeds();

  void executeDefaultAction(MQTTClient* mqttClient);
};

#endif