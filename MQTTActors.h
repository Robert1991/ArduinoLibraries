#ifndef MQTTActors_h
#define MQTTActors_h

#include "LiquidCrystal.h"
#include <wireUtils.h>

#include "MQTTDevice.h"

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
  bool invertSignal = false;

  void turnOn();
  void turnOff();

protected:
  bool switchOn = false;
  void reportStatusInformation();

public:
  MQTTSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, int switchPin, String deviceName = "relais_switch",
             String deviceType = "light", bool invertSignal = false);

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  virtual void setupActor();
  virtual void executeLoopMethod();
  void setupSubscriptions();
  bool consumeMessage(String topic, String payload);
};

class MQTTDeviceResetSwitch : public MQTTSwitch {
public:
  MQTTDeviceResetSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, String deviceName = "reset_switch");
  void setupActor();
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void executeLoopMethod();
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
  MQTTI2CRgbLight(MQTTDeviceInfo deviceInfo, String uniqueId,
                  MQTTI2CRgbLightConfiguration lightConfiguration);
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void setupActor();
  void setupSubscriptions();
  bool consumeMessage(String topic, String payload);
  void applyChoosenColorToLeds();
};

class MQTTLcdDisplayDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
private:
  String deviceName;

public:
  MQTTLcdDisplayDeviceClassificationFactory(String deviceUniqueId, String deviceName);
  MQTTDeviceClassification create();
};

class MQTTLcdDisplay : public MQTTActor {
private:
  LiquidCrystal *display;
  String lastText = "";
  void reportStatusInformation();
  void displayMessage(String payload);

public:
  MQTTLcdDisplay(MQTTDeviceInfo deviceInfo, String uniqueId, LiquidCrystal *display,
                 String deviceName = "show");

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void setupActor();
  void executeLoopMethod();
  void setupSubscriptions();
  void configureInTargetPlatform();
  bool deregisterFromTargetPlatform();
  bool consumeMessage(String topic, String payload);
};

#endif