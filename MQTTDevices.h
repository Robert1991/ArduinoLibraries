#ifndef MQTTDevices_h
#define MQTTDevices_h

#include <IOTClients.h>

struct MQTTSwitchConfiguration {
  char* switchSubscriptionTopic;
  char* switchStateTopic;
  int switchPin;
};

class MQTTDevice {
  virtual void setupSubscription(MQTTClient* mqttClient) = 0;
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

  void setupSubscription(MQTTClient* mqttClient);
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
  void setupSubscription(MQTTClient* mqttClient);
  bool consumeMessage(MQTTClient* mqttClient, String topic, String payload);
  void applyChoosenColorToLeds();
  void executeDefaultAction(MQTTClient* mqttClient);
};
#endif