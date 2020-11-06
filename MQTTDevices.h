#ifndef MQTTDevices_h
#define MQTTDevices_h

#include <IOTClients.h>

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

 protected:
  void publishFloatValue(char* stateTopic, float value);
  void publishBinaryMessage(char* stateTopic, bool on);
};

class MQTTMotionSensor : public MQTTSensor {
 private:
  char* stateTopic;
  int motionSensorPin;

 public:
  MQTTMotionSensor(MQTTClient* mqttClient, char* stateTopic, int motionSensorPin);
  void setupSensor();
  void publishMeasurement();
};

class MQTTDhtSensor : public MQTTSensor {
 private:
  DHT* dhtSensor;
  char* temperatureStateTopic;
  char* humidityStateTopic;

  void publishTemperature();
  void publishHumidity();

 public:
  MQTTDhtSensor(MQTTClient* mqttClient, DHT* dhtSensor, char* temperatureStateTopic, char* humidityStateTopic);
  void setupSensor();
  void publishMeasurement();
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
#endif