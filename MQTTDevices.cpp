#include "MQTTDevices.h"

MQTTSensor::MQTTSensor(MQTTClient* mqttClient) { this->mqttClient = mqttClient; }

void MQTTSensor::publishFloatValue(char* stateTopic, float floatValue) {
  char result[8];
  dtostrf(floatValue, 6, 2, result);
  mqttClient->publishMessage(stateTopic, result);
}

void MQTTSensor::publishBinaryMessage(char* stateTopic, bool on) {
  if (on) {
    mqttClient->publishMessage(stateTopic, "on");
  } else {
    mqttClient->publishMessage(stateTopic, "off");
  }
}

MQTTMotionSensor::MQTTMotionSensor(MQTTClient* mqttClient, char* stateTopic, int motionSensorPin) : MQTTSensor(mqttClient) {
  this->stateTopic = stateTopic;
  this->motionSensorPin = motionSensorPin;
}

void MQTTMotionSensor::setupSensor() { pinMode(motionSensorPin, INPUT); }

void MQTTMotionSensor::publishMeasurement() {
  int state = digitalRead(motionSensorPin);
  bool motionDetected = false;
  if (state == HIGH) {
    logLineToSerial("Motion detected");
    motionDetected = true;
  }

  if (lastMotionSensorState != motionDetected) {
    publishBinaryMessage(stateTopic, motionDetected);
    lastMotionSensorState = motionDetected;
  }
}

void MQTTMotionSensor::reset() { lastMotionSensorState = false; }

MQTTDhtSensor::MQTTDhtSensor(MQTTClient* mqttClient, DHT* dhtSensor, char* temperatureStateTopic, char* humidityStateTopic) : MQTTSensor(mqttClient) {
  this->dhtSensor = dhtSensor;
  this->temperatureStateTopic = temperatureStateTopic;
  this->humidityStateTopic = humidityStateTopic;
}

void MQTTDhtSensor::setupSensor() { dhtSensor->begin(); }
void MQTTDhtSensor::publishMeasurement() {
  publishTemperature();
  publishHumidity();
}

void MQTTDhtSensor::publishTemperature() {
  float currentTempSensorValue = dhtSensor->readTemperature();
  if (!isnan(currentTempSensorValue)) {
    if (!areEqual(lastMeasuredTemperature, currentTempSensorValue)) {
      logToSerial("Temperature: ");
      logToSerial(currentTempSensorValue);
      logLineToSerial(" degrees celcius");

      publishFloatValue(temperatureStateTopic, currentTempSensorValue);
      lastMeasuredTemperature = currentTempSensorValue;
    }
  }
}

void MQTTDhtSensor::publishHumidity() {
  float currentHumiditySensorValue = dhtSensor->readHumidity();
  if (!isnan(currentHumiditySensorValue)) {
    if (!areEqual(lastMeasuredHumidity, currentHumiditySensorValue)) {
      logToSerial("Humidity: ");
      logToSerial(currentHumiditySensorValue);
      logLineToSerial("%\t");

      publishFloatValue(humidityStateTopic, currentHumiditySensorValue);
      lastMeasuredHumidity = currentHumiditySensorValue;
    }
  }
}

void MQTTDhtSensor::reset() {
  lastMeasuredTemperature = 0.0;
  lastMeasuredHumidity = 0.0;
}

bool MQTTDhtSensor::areEqual(float value1, float value2) {
  if (abs(value1 - value2) > 0.001) {
    return false;
  }
  return true;
}

MQTTSwitch::MQTTSwitch(MQTTSwitchConfiguration configuration) {
  this->mqttClient = mqttClient;
  this->configuration = configuration;
}

void MQTTSwitch::setupActor(MQTTClient* mqttClient) {
  pinMode(configuration.switchPin, OUTPUT);
  mqttClient->subscribeTopic(configuration.switchSubscriptionTopic);
}

void MQTTSwitch::executeDefaultAction(MQTTClient* mqttClient) {
  switchOn = false;
  mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_OFF);
  applySwitchStatus();
}

void MQTTSwitch::applySwitchStatus() {
  if (switchOn) {
    digitalWrite(configuration.switchPin, HIGH);
  } else {
    digitalWrite(configuration.switchPin, LOW);
  }
}

bool MQTTSwitch::consumeMessage(MQTTClient* mqttClient, String topic, String payload) {
  if (topic.equals(configuration.switchSubscriptionTopic)) {
    if (payload.equals(SWITCH_PAYLOAD_ON)) {
      logLineToSerial("Switch was turned on");
      switchOn = true;
      mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_ON);
    } else {
      logLineToSerial("Switch was turned off");
      switchOn = false;
      mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_OFF);
    }
    applySwitchStatus();
    return true;
  }
  return false;
}

MQTTRgbLight::MQTTRgbLight(MQTTRgbLightConfiguration configuration) { this->configuration = configuration; }

void MQTTRgbLight::setupActor(MQTTClient* mqttClient) {
  pinMode(configuration.pins.red, OUTPUT);
  pinMode(configuration.pins.green, OUTPUT);
  pinMode(configuration.pins.blue, OUTPUT);
  mqttClient->subscribeTopic(configuration.lightSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(configuration.brightnessSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(configuration.colorSetSubscriptionTopic);
  mqttClient->subscribeTopic(configuration.lightStateTopic);
}

void MQTTRgbLight::applyChoosenColorToLeds() {
  if (stripOn) {
    analogWrite(configuration.pins.red, (int)(redColorPart * currentBrightness));
    analogWrite(configuration.pins.green, (int)(greenColorPart * currentBrightness));
    analogWrite(configuration.pins.blue, (int)(blueColorPart * currentBrightness));
  } else {
    analogWrite(configuration.pins.red, LOW);
    analogWrite(configuration.pins.green, LOW);
    analogWrite(configuration.pins.blue, LOW);
  }
}

void MQTTRgbLight::executeDefaultAction(MQTTClient* mqttClient) {
  currentBrightness = 1.0;
  redColorPart = 0;
  greenColorPart = 0;
  blueColorPart = 0;
  stripOn = false;
  reportStatus(mqttClient);
}

void MQTTRgbLight::reportStatus(MQTTClient* mqttClient) {
  if (stripOn) {
    mqttClient->publishMessage(configuration.lightStateTopic, "{\"state\":\"ON\"}", true);
  } else {
    mqttClient->publishMessage(configuration.lightStateTopic, "{\"state\":\"OFF\"}", true);
  }
  int brightnessValue = (int)(currentBrightness * 255);
  char brightnessValueStringBuffer[16];
  itoa(brightnessValue, brightnessValueStringBuffer, 10);
  mqttClient->publishMessage(configuration.brightnessStateTopic, brightnessValueStringBuffer);

  char concatination[256];
  sprintf(concatination, "{ \"rgb\":[%i,%i,%i]}", redColorPart, greenColorPart, blueColorPart);
  mqttClient->publishMessage(configuration.colorSetStateTopic, concatination);
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
  if (topic.equals(configuration.lightSwitchSubscriptionTopic)) {
    if (payload.equals("ON")) {
      logLineToSerial("Strip was turned on");
      stripOn = true;
    } else {
      logLineToSerial("Strip was turned off");
      stripOn = false;
    }
    return true;
  } else if (topic.equals(configuration.brightnessSwitchSubscriptionTopic)) {
    processBrightnessPayload(payload);
    return true;
  } else if (topic.equals(configuration.colorSetSubscriptionTopic)) {
    processColorCommandPayload(payload);
    return true;
  }
  return false;
}

bool MQTTRgbLight::consumeMessage(MQTTClient* mqttClient, String topic, String payload) {
  bool stateChanged = processIncomingMessage(topic, payload);
  if (stateChanged) {
    applyChoosenColorToLeds();
    reportStatus(mqttClient);
  }
  return stateChanged;
}

MQTTI2CRgbLight::MQTTI2CRgbLight(MQTTI2CRgbLightConfiguration lightConfiguration) { this->lightConfiguration = lightConfiguration; }

void MQTTI2CRgbLight::executeDefaultAction(MQTTClient* mqttClient) {}

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
  if (topic.equals(lightConfiguration.lightSwitchSubscriptionTopic)) {
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
  } else if (topic.equals(lightConfiguration.brightnessSwitchSubscriptionTopic)) {
    int requestedBrightness = payload.toInt();
    if (requestedBrightness != currentBrightness) {
      brightnessChanged = true;
      currentBrightness = requestedBrightness;
      return true;
    }
    return false;
  } else if (topic.equals(lightConfiguration.colorSetSubscriptionTopic)) {
    processColorCommandPayload(payload);
    return colorChanged;
  }
  return false;
}

void MQTTI2CRgbLight::reportStatus(MQTTClient* mqttClient) {
  if (stripOn) {
    mqttClient->publishMessage(lightConfiguration.lightStateTopic, "{\"state\":\"ON\"}");
  } else {
    mqttClient->publishMessage(lightConfiguration.lightStateTopic, "{\"state\":\"OFF\"}");
  }
  char brightnessValueStringBuffer[16];
  itoa(currentBrightness, brightnessValueStringBuffer, 10);
  mqttClient->publishMessage(lightConfiguration.brightnessStateTopic, brightnessValueStringBuffer);

  char concatination[256];
  sprintf(concatination, "{ \"rgb\":[%i,%i,%i]}", redColorPart, greenColorPart, blueColorPart);
  mqttClient->publishMessage(lightConfiguration.colorSetStateTopic, concatination);
}

bool MQTTI2CRgbLight::consumeMessage(MQTTClient* mqttClient, String topic, String payload) {
  bool stateChanged = processIncomingMessage(topic, payload);
  if (stateChanged) {
    reportStatus(mqttClient);
    applyChoosenColorToLeds();
  }
  return stateChanged;
}

void MQTTI2CRgbLight::setupActor(MQTTClient* mqttClient) {
  establishI2CConnectionTo(lightConfiguration.wirePins.sdaPin, lightConfiguration.wirePins.sclPin, true);
  mqttClient->subscribeTopic(lightConfiguration.lightSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.brightnessSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.colorSetSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.lightStateTopic);
}

void MQTTI2CRgbLight::refreshI2CConnection() {
  if (checkI2CConnection()) {
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