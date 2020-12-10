#include "MQTTActors.h"

MQTTSwitchDeviceClassificationFactory::MQTTSwitchDeviceClassificationFactory(String deviceUniqueId, String deviceName, String deviceType)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {
  this->deviceName = deviceName;
  this->deviceType = deviceType;
}

MQTTDeviceClassification MQTTSwitchDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "", deviceType, deviceName, false};
  return deviceClass;
}

MQTTSwitch::MQTTSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, int switchPin, String deviceName, String deviceType)
    : MQTTActor(new MQTTSwitchDeviceClassificationFactory(uniqueId, deviceName, deviceType), deviceInfo) {
  this->commandTopic = deviceEntityName + "/" + "switch";
  this->switchPin = switchPin;
}

DynamicJsonDocument MQTTSwitch::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["cmd_t"] = commandTopic;
  autoConfigureJsonDocument["ret"] = true;
  return autoConfigureJsonDocument;
}

void MQTTSwitch::setupActor() { pinMode(switchPin, OUTPUT); }

void MQTTSwitch::setupSubscriptions() { subscribeTopic(commandTopic); }

void MQTTSwitch::executeLoopMethod() {
  if (switchOn) {
    digitalWrite(switchPin, HIGH);
  } else {
    digitalWrite(switchPin, LOW);
  }
}

void MQTTSwitch::reportStatusInformation() {
  if (actorStatusChanged) {
    if (switchOn) {
      publishState(SWITCH_PAYLOAD_ON);
    } else {
      publishState(SWITCH_PAYLOAD_OFF);
    }
  }
}

bool MQTTSwitch::consumeMessage(String topic, String payload) {
  if (topic.equals(commandTopic)) {
    if (payload.equals(SWITCH_PAYLOAD_ON)) {
      logLineToSerial("Switch was turned on");
      switchOn = true;
    } else {
      logLineToSerial("Switch was turned off");
      switchOn = false;
    }
    actorStatusChanged = true;
    executeLoopMethod();
    return true;
  }
  return false;
}

MQTTDeviceResetSwitch::MQTTDeviceResetSwitch(MQTTDeviceInfo deviceInfo, String uniqueId, String deviceName)
    : MQTTSwitch(deviceInfo, uniqueId, 0, deviceName, "switch") {}

void MQTTDeviceResetSwitch::setupActor() {}

void MQTTDeviceResetSwitch::executeLoopMethod() {
  if (switchOn) {
    switchOn = false;
    reportStatusInformation();
    delay(100);
  }
}

MQTTRgbLightDeviceClassificationFactory::MQTTRgbLightDeviceClassificationFactory(String uniqueId) : MQTTDeviceClassificationFactory(uniqueId) {}

MQTTDeviceClassification MQTTRgbLightDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "", "light", "rgb", false};
  return deviceClass;
}

MQTTRgbLight::MQTTRgbLight(MQTTDeviceInfo deviceInfo, String uniqueId, RGBPins pins)
    : MQTTActor(new MQTTRgbLightDeviceClassificationFactory(uniqueId), deviceInfo) {
  this->pins = pins;
  commandTopic = deviceEntityName + "/switch";
  brightnessCommandTopic = deviceEntityName + "/brightness/set";
  brightnessStateTopic = deviceEntityName + "/brightness/status";
  colorCommandTopic = deviceEntityName + "/color/set";
  colorStateTopic = deviceEntityName + "/color/status";
}

DynamicJsonDocument MQTTRgbLight::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["cmd_t"] = commandTopic;
  autoConfigureJsonDocument["bri_cmd_t"] = brightnessCommandTopic;
  autoConfigureJsonDocument["bri_stat_t"] = brightnessStateTopic;
  autoConfigureJsonDocument["rgb_cmd_t"] = colorCommandTopic;
  autoConfigureJsonDocument["rgb_stat_t"] = colorStateTopic;
  autoConfigureJsonDocument["rgb_val_tpl"] = "{{ value_json.rgb | join(',') }}";
  autoConfigureJsonDocument["stat_val_tpl"] = "{{ value_json.state }}";
  autoConfigureJsonDocument["ret"] = true;
  return autoConfigureJsonDocument;
}

void MQTTRgbLight::setupActor() {
  pinMode(pins.red, OUTPUT);
  pinMode(pins.green, OUTPUT);
  pinMode(pins.blue, OUTPUT);
}

void MQTTRgbLight::setupSubscriptions() {
  subscribeTopic(commandTopic);
  subscribeTopic(brightnessCommandTopic);
  subscribeTopic(colorCommandTopic);
}

void MQTTRgbLight::applyChoosenColorToLeds() {
  if (stripOn) {
    analogWrite(pins.red, (int)(redColorPart * currentBrightness));
    analogWrite(pins.green, (int)(greenColorPart * currentBrightness));
    analogWrite(pins.blue, (int)(blueColorPart * currentBrightness));
  } else {
    analogWrite(pins.red, LOW);
    analogWrite(pins.green, LOW);
    analogWrite(pins.blue, LOW);
  }
}

void MQTTRgbLight::reportStatusInformation() {
  if (stripOn) {
    publishTo(stateTopic, "{\"state\":\"ON\"}", true);
  } else {
    publishTo(stateTopic, "{\"state\":\"OFF\"}", true);
  }
  int brightnessValue = (int)(currentBrightness * 255);
  String brightnessValueString = String(brightnessValue);
  publishTo(brightnessStateTopic, brightnessValueString);

  char concatination[256];
  sprintf(concatination, "{ \"rgb\":[%i,%i,%i]}", redColorPart, greenColorPart, blueColorPart);
  publishTo(colorStateTopic, concatination);
}

void MQTTRgbLight::processBrightnessPayload(String payload) {
  currentBrightness = (double)(payload.toFloat() / 255.0);
  logToSerial("brightness is now set to: ");
  logLineToSerial(currentBrightness);
}

void MQTTRgbLight::processColorCommandPayload(String payload) {
  char buffer[32];
  payload.toCharArray(buffer, sizeof(buffer));
  char* p = buffer;
  char* str;
  int iteration = 0;
  while ((str = strtok_r(p, ",", &p)) != NULL) {
    if (iteration == 0) {
      redColorPart = atoi(str);
      logToSerial("red color part was set to: ");
      logLineToSerial(redColorPart);
    } else if (iteration == 1) {
      greenColorPart = atoi(str);
      logToSerial("green color part was set to: ");
      logLineToSerial(greenColorPart);
    } else if (iteration == 2) {
      blueColorPart = atoi(str);
      logToSerial("blue color part was set to: ");
      logLineToSerial(blueColorPart);
      break;
    }
    iteration++;
  }
}

bool MQTTRgbLight::processIncomingMessage(String topic, String payload) {
  if (topic.equals(commandTopic)) {
    if (payload.equals("ON")) {
      logLineToSerial("Strip was turned on");
      stripOn = true;
    } else {
      logLineToSerial("Strip was turned off");
      stripOn = false;
    }
    return true;
  } else if (topic.equals(brightnessCommandTopic)) {
    processBrightnessPayload(payload);
    return true;
  } else if (topic.equals(colorCommandTopic)) {
    processColorCommandPayload(payload);
    return true;
  }
  return false;
}

bool MQTTRgbLight::consumeMessage(String topic, String payload) {
  actorStatusChanged = processIncomingMessage(topic, payload);
  if (actorStatusChanged) {
    applyChoosenColorToLeds();
  }
  return actorStatusChanged;
}

MQTTI2CRgbLightDeviceClassificationFactory::MQTTI2CRgbLightDeviceClassificationFactory(String uniqueId) : MQTTDeviceClassificationFactory(uniqueId) {}

MQTTDeviceClassification MQTTI2CRgbLightDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "", "light", "i2c_rgb", false};
  return deviceClass;
}

MQTTI2CRgbLight::MQTTI2CRgbLight(MQTTDeviceInfo deviceInfo, String uniqueId, MQTTI2CRgbLightConfiguration lightConfiguration)
    : MQTTActor(new MQTTI2CRgbLightDeviceClassificationFactory(uniqueId), deviceInfo) {
  this->lightConfiguration = lightConfiguration;
  commandTopic = deviceEntityName + "/switch";
  brightnessCommandTopic = deviceEntityName + "/brightness/set";
  brightnessStateTopic = deviceEntityName + "/brightness/status";
  colorCommandTopic = deviceEntityName + "/color/set";
  colorStateTopic = deviceEntityName + "/color/status";
}

DynamicJsonDocument MQTTI2CRgbLight::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["cmd_t"] = commandTopic;
  autoConfigureJsonDocument["bri_cmd_t"] = brightnessCommandTopic;
  autoConfigureJsonDocument["bri_stat_t"] = brightnessStateTopic;
  autoConfigureJsonDocument["rgb_cmd_t"] = colorCommandTopic;
  autoConfigureJsonDocument["rgb_stat_t"] = colorStateTopic;
  autoConfigureJsonDocument["rgb_val_tpl"] = "{{ value_json.rgb | join(',') }}";
  autoConfigureJsonDocument["stat_val_tpl"] = "{{ value_json.state }}";
  autoConfigureJsonDocument["ret"] = true;
  return autoConfigureJsonDocument;
}

void MQTTI2CRgbLight::setupActor() { establishI2CConnectionTo(lightConfiguration.wirePins.sdaPin, lightConfiguration.wirePins.sclPin, true); }

void MQTTI2CRgbLight::setupSubscriptions() {
  subscribeTopic(commandTopic);
  subscribeTopic(brightnessCommandTopic);
  subscribeTopic(colorCommandTopic);
}

void MQTTI2CRgbLight::processColorCommandPayload(String payload) {
  char buffer[32];
  payload.toCharArray(buffer, sizeof(buffer));
  char* p = buffer;
  char* str;
  int iteration = 0;
  while ((str = strtok_r(p, ",", &p)) != NULL) {
    if (iteration == 0) {
      int requestedRedColorPart = atoi(str);
      if (requestedRedColorPart != redColorPart) {
        colorChanged = true;
        redColorPart = requestedRedColorPart;
        logToSerial("red color part was set to: ");
        logLineToSerial(redColorPart);
      }
    } else if (iteration == 1) {
      int requestedGreenColorPart = atoi(str);
      if (requestedGreenColorPart != greenColorPart) {
        colorChanged = true;
        greenColorPart = requestedGreenColorPart;
        logToSerial("red color part was set to: ");
        logLineToSerial(greenColorPart);
      }
    } else if (iteration == 2) {
      int requestedBlueColorPart = atoi(str);
      if (requestedBlueColorPart != blueColorPart) {
        colorChanged = true;
        blueColorPart = requestedBlueColorPart;
        logToSerial("red color part was set to: ");
        logLineToSerial(blueColorPart);
      }
      break;
    }
    iteration++;
  }
}

bool MQTTI2CRgbLight::processIncomingMessage(String topic, String payload) {
  if (topic.equals(commandTopic)) {
    if (payload.equals("ON") && !stripOn || payload.equals("OFF") && stripOn) {
      onOffStatusChanged = true;
      if (payload.equals("ON")) {
        logLineToSerial("Strip was turned on");
        stripOn = true;
      } else {
        logLineToSerial("Strip was turned off");
        stripOn = false;
      }
      return true;
    }
    return false;
  } else if (topic.equals(brightnessCommandTopic)) {
    int requestedBrightness = payload.toInt();
    if (requestedBrightness != currentBrightness) {
      brightnessChanged = true;
      currentBrightness = requestedBrightness;
      return true;
    }
    return false;
  } else if (topic.equals(colorCommandTopic)) {
    processColorCommandPayload(payload);
    return colorChanged;
  }
  return false;
}

void MQTTI2CRgbLight::reportStatusInformation() {
  if (stripOn) {
    publishTo(stateTopic, "{\"state\":\"ON\"}", true);
  } else {
    publishTo(stateTopic, "{\"state\":\"OFF\"}", true);
  }
  String brightnessValueString = String(currentBrightness);
  publishTo(brightnessStateTopic, brightnessValueString);

  char concatination[256];
  sprintf(concatination, "{ \"rgb\":[%i,%i,%i]}", redColorPart, greenColorPart, blueColorPart);
  publishTo(colorStateTopic, concatination);
}

bool MQTTI2CRgbLight::consumeMessage(String topic, String payload) {
  actorStatusChanged = processIncomingMessage(topic, payload);
  if (actorStatusChanged) {
    applyChoosenColorToLeds();
  }
  return actorStatusChanged;
}

void MQTTI2CRgbLight::refreshI2CConnection() {
  if (checkI2CConnection(lightConfiguration.wirePins.sdaPin, lightConfiguration.wirePins.sclPin)) {
    Serial.println("connection to slave got lost. trying to reestablish connection...");
    establishI2CConnectionTo(lightConfiguration.wirePins.sdaPin, lightConfiguration.wirePins.sclPin);
  } else {
    while (Wire.available()) {
      Serial.println("flushing wire");
    }
  }
}

void MQTTI2CRgbLight::sendRGBValuesToSecondary(byte redValue, byte greenValue, byte blueValue, int delayTime) {
  refreshI2CConnection();
  Serial.print("Sending I2C command {");
  Serial.print(lightConfiguration.i2cConnectionCommands.setColorCommand);
  Serial.print("} with parameter {");
  Serial.print(redValue);
  Serial.print(";");
  Serial.print(greenValue);
  Serial.print(";");
  Serial.print(blueValue);
  Serial.println("}");
  Wire.beginTransmission(lightConfiguration.wirePins.i2cSecondaryAddress);
  Wire.write(lightConfiguration.i2cConnectionCommands.setColorCommand);
  Wire.write(redValue);
  Wire.write(greenValue);
  Wire.write(blueValue);
  Wire.endTransmission();
  delay(delayTime);
}

void MQTTI2CRgbLight::applyChoosenColorToLeds() {
  if (stripOn) {
    if (onOffStatusChanged) {
      sendI2CCommandWithParameter(lightConfiguration.wirePins.i2cSecondaryAddress, lightConfiguration.i2cConnectionCommands.setOnOffCommand, 1);
      onOffStatusChanged = false;
    }
    if (colorChanged) {
      sendRGBValuesToSecondary(redColorPart, greenColorPart, blueColorPart);
      colorChanged = false;
    }
    if (brightnessChanged) {
      sendI2CCommandWithParameter(lightConfiguration.wirePins.i2cSecondaryAddress, lightConfiguration.i2cConnectionCommands.setBrightnessCommand,
                                  currentBrightness);
      brightnessChanged = false;
    }
  } else {
    if (onOffStatusChanged) {
      sendI2CCommandWithParameter(lightConfiguration.wirePins.i2cSecondaryAddress, lightConfiguration.i2cConnectionCommands.setOnOffCommand, 0);
      onOffStatusChanged = false;
    }
  }
}