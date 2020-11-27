#include "MQTTDevices.h"

MQTTDeviceClassificationFactory::MQTTDeviceClassificationFactory(String deviceUniqueId) { this->deviceUniqueId = deviceUniqueId; }

MQTTDevice::MQTTDevice(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) {
  this->mqttClient = mqttClient;
  MQTTDeviceClassification deviceClass = deviceClassFactory->create();
  assignDeviceInfos(deviceClass, deviceInfo);
}

void MQTTDevice::assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo) {
  this->sensorName = deviceInfo.deviceName + "_" + deviceClass.sensorType;
  String sensorHomeAssistantPath = deviceInfo.autoDiscoveryPrefix + "/" + deviceClass.deviceType + "/" + sensorName;
  this->stateTopic = sensorHomeAssistantPath + "/state";
  this->autoDiscoveryMQTTConfigureTopic = sensorHomeAssistantPath + "/config";
  this->deviceClassification = deviceClass;
  this->deviceInfo = deviceInfo;
}

int MQTTDevice::publishAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument) {
  return publishJsonDocument(autoDiscoveryMQTTConfigureTopic, autoConfigureJsonDocument, true);
}

int MQTTDevice::publishJsonDocument(String stateTopic, StaticJsonDocument<512> jsonDocument, bool retain) {
  String jsonMessage;
  serializeJson(jsonDocument, jsonMessage);
  logToSerial("Publishing to: ");
  logToSerial(stateTopic);
  logLineToSerial(" with payload: ");
  logLineToSerial(jsonMessage);
  mqttClient->publishMessage(stateTopic, jsonMessage, retain);
  return 1;
}

StaticJsonDocument<512> MQTTDevice::createDeviceInfoJsonObject() {
  StaticJsonDocument<512> autoConfigureJsonDocument;
  autoConfigureJsonDocument["name"] = sensorName;
  autoConfigureJsonDocument["dev_cla"] = deviceClassification.deviceClass;
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
    delay(1500);

    StaticJsonDocument<512> autoConfigureJsonDocument = createDeviceInfoJsonObject();
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

StaticJsonDocument<512> MQTTDevice::extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument) { return autoConfigureJsonDocument; }

MQTTSensor::MQTTSensor(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo)
    : MQTTDevice(mqttClient, deviceClassFactory, deviceInfo) {}

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

void MQTTSensor::reset() {}

bool MQTTSensor::areEqual(float value1, float value2, float maxDifference) {
  if (fabs(value1 - value2) > maxDifference) {
    return false;
  }
  return true;
}

MQTTDevicePingDeviceClassificationFactory::MQTTDevicePingDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTDevicePingDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "connectivity", "binary_sensor", "ping"};
  return deviceClass;
}

MQTTDevicePing::MQTTDevicePing(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String uniqueId, long pingTimeout)
    : MQTTSensor(mqttClient, new MQTTDevicePingDeviceClassificationFactory(uniqueId), deviceInfo) {
  this->pingTimeout = pingTimeout;
  startTime = millis();
}

void MQTTDevicePing::publishMeasurement() {
  unsigned long currentTime = millis();
  if (currentTime - startTime > pingTimeout) {
    publishBinaryMessage(true);
    logLineToSerial("Ping published!");
    pingWasSentInLastIteration = true;
    startTime = millis();
  }
  if (pingWasSentInLastIteration) {
    publishBinaryMessage(false);
    pingWasSentInLastIteration = false;
  }
}

StaticJsonDocument<512> MQTTDevicePing::extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument) {
  int pingTimeoutInSeconds = pingTimeout / 1000;
  autoConfigureJsonDocument["exp_aft"] = 3 * pingTimeoutInSeconds;
  autoConfigureJsonDocument["off_dly"] = pingTimeoutInSeconds + 1;
  return autoConfigureJsonDocument;
}

void MQTTDevicePing::reset() { startTime = 0; }

MQTTPhotoLightSensorDeviceClassificationFactory::MQTTPhotoLightSensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTPhotoLightSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "voltage", "sensor", "light_intensity"};
  return deviceClass;
}

MQTTPhotoLightSensor::MQTTPhotoLightSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin)
    : MQTTSensor(mqttClient, new MQTTPhotoLightSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->analogPin = analogPin;
}

void MQTTPhotoLightSensor::publishMeasurement() {
  int sensorValue = analogRead(A0);
  float currentVoltage = sensorValue * (3.3 / 1023.0);

  if (!areEqual(lastVoltageValue, currentVoltage, 0.1)) {
    lastVoltageValue = currentVoltage;
    logToSerial("light sensitivity voltage output changed to: ");
    logLineToSerial(lastVoltageValue);
    publishFloatValue(lastVoltageValue);
  }
}

StaticJsonDocument<512> MQTTPhotoLightSensor::extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_measurement"] = "V";
  autoConfigureJsonDocument["expire_after"] = 180;
  return autoConfigureJsonDocument;
}

void MQTTPhotoLightSensor::reset() { lastVoltageValue = 0.0; }

MQTTMotionSensorDeviceClassificationFactory::MQTTMotionSensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTMotionSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "motion", "binary_sensor", "motion_detected"};
  return deviceClass;
}

MQTTMotionSensor::MQTTMotionSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin,
                                   int motionDetectionIterations, int motionDetectionTimeout, int motionDetectedThreshold)
    : MQTTSensor(mqttClient, new MQTTMotionSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->motionSensorPin = motionSensorPin;
  this->motionDetectionIterations = motionDetectionIterations;
  this->motionDetectionTimeout = motionDetectionTimeout;
  this->motionDetectedThreshold = motionDetectedThreshold;
}

void MQTTMotionSensor::setupSensor() { pinMode(motionSensorPin, INPUT); }

void MQTTMotionSensor::publishMeasurement() {
  int motionsDetected = 0;
  for (int detection = 0; detection < motionDetectionIterations; detection++) {
    int state = digitalRead(motionSensorPin);
    if (state == HIGH) {
      motionsDetected++;
    }
    if (motionsDetected > motionDetectedThreshold) {
      break;
    }
    delay(motionDetectionTimeout);
  }

  bool motionDetected = false;
  if (motionsDetected > motionDetectedThreshold) {
    motionDetected = true;
  }
  if (lastMotionSensorState != motionDetected) {
    publishBinaryMessage(motionDetected);
    logLineToSerial("Motion detected");
    lastMotionSensorState = motionDetected;
  }
}

void MQTTMotionSensor::reset() { lastMotionSensorState = false; }

MQTTDHTSensor::MQTTDHTSensor(MessageQueueClient* mqttClient, MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo,
                             DHT* dhtSensor, int resetValuesIterations)
    : MQTTSensor(mqttClient, deviceClassFactory, deviceInfo) {
  this->dhtSensor = dhtSensor;
  this->resetValuesIterations = resetValuesIterations;
}

void MQTTDHTSensor::setupSensor() { dhtSensor->begin(); }

bool MQTTDHTSensor::resetIfRequired() {
  if (currentIteration < resetValuesIterations) {
    logToSerial("current iteration: ");
    logLineToSerial(currentIteration);
    currentIteration++;
    return false;
  }
  logLineToSerial("Resetting stored sensor values");
  currentIteration = 0;
  reset();
  return true;
}

MQTTHumiditySensorDeviceClassificationFactory::MQTTHumiditySensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTHumiditySensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "humidity", "sensor", "humidity"};
  return deviceClass;
}

MQTTHumiditySensor::MQTTHumiditySensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId,
                                       int resetValuesIterations)
    : MQTTDHTSensor(mqttClient, new MQTTHumiditySensorDeviceClassificationFactory(sensorUniqueId), deviceInfo, dhtSensor, resetValuesIterations) {
  this->dhtSensor = dhtSensor;
}

void MQTTHumiditySensor::publishMeasurement() {
  resetIfRequired();
  publishHumidity();
}

void MQTTHumiditySensor::publishHumidity() {
  float currentHumiditySensorValue = dhtSensor->readHumidity();
  if (!isnan(currentHumiditySensorValue)) {
    if (!areEqual(lastMeasuredHumidity, currentHumiditySensorValue)) {
      logToSerial("Humidity: ");
      logToSerial(currentHumiditySensorValue);
      logLineToSerial("%\t");

      publishFloatValue(currentHumiditySensorValue);
      lastMeasuredHumidity = currentHumiditySensorValue;
    }
  }
}

void MQTTHumiditySensor::reset() { lastMeasuredHumidity = 0.0; }

MQTTTemperatureSensorDeviceClassificationFactory::MQTTTemperatureSensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTTemperatureSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "temperature", "sensor", "temperature"};
  return deviceClass;
}

MQTTTemperatureSensor::MQTTTemperatureSensor(MessageQueueClient* mqttClient, MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId,
                                             int resetValuesIterations)
    : MQTTDHTSensor(mqttClient, new MQTTTemperatureSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo, dhtSensor, resetValuesIterations) {
  this->dhtSensor = dhtSensor;
}

void MQTTTemperatureSensor::publishMeasurement() {
  resetIfRequired();
  publishTemperature();
}

void MQTTTemperatureSensor::publishTemperature() {
  float currentTempSensorValue = dhtSensor->readTemperature();
  if (!isnan(currentTempSensorValue)) {
    if (!areEqual(lastMeasuredTemperature, currentTempSensorValue)) {
      logToSerial("Temperature: ");
      logToSerial(currentTempSensorValue);
      logLineToSerial(" degrees celcius");

      publishFloatValue(currentTempSensorValue);
      lastMeasuredTemperature = currentTempSensorValue;
    }
  }
}

void MQTTTemperatureSensor::reset() { lastMeasuredTemperature = 0.0; }

StaticJsonDocument<512> MQTTTemperatureSensor::extendAutoDiscoveryInfo(StaticJsonDocument<512> autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_measurement"] = "°C";
  autoConfigureJsonDocument["expire_after"] = 300;
  return autoConfigureJsonDocument;
}

MQTTSwitch::MQTTSwitch(MQTTSwitchConfiguration configuration) {
  this->mqttClient = mqttClient;
  this->configuration = configuration;
}

void MQTTSwitch::setupActor(MessageQueueClient* mqttClient) {
  pinMode(configuration.switchPin, OUTPUT);
  mqttClient->subscribeTopic(configuration.switchSubscriptionTopic);
}

void MQTTSwitch::executeDefaultAction(MessageQueueClient* mqttClient) {
  switchOn = true;
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

bool MQTTSwitch::consumeMessage(MessageQueueClient* mqttClient, String topic, String payload) {
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

void MQTTRgbLight::setupActor(MessageQueueClient* mqttClient) {
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

void MQTTRgbLight::executeDefaultAction(MessageQueueClient* mqttClient) {
  currentBrightness = 1.0;
  redColorPart = 0;
  greenColorPart = 0;
  blueColorPart = 0;
  stripOn = false;
  reportStatus(mqttClient);
}

void MQTTRgbLight::reportStatus(MessageQueueClient* mqttClient) {
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

bool MQTTRgbLight::consumeMessage(MessageQueueClient* mqttClient, String topic, String payload) {
  bool stateChanged = processIncomingMessage(topic, payload);
  if (stateChanged) {
    applyChoosenColorToLeds();
    reportStatus(mqttClient);
  }
  return stateChanged;
}

MQTTI2CRgbLight::MQTTI2CRgbLight(MQTTI2CRgbLightConfiguration lightConfiguration) { this->lightConfiguration = lightConfiguration; }

void MQTTI2CRgbLight::executeDefaultAction(MessageQueueClient* mqttClient) {}

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

void MQTTI2CRgbLight::reportStatus(MessageQueueClient* mqttClient) {
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

bool MQTTI2CRgbLight::consumeMessage(MessageQueueClient* mqttClient, String topic, String payload) {
  bool stateChanged = processIncomingMessage(topic, payload);
  if (stateChanged) {
    reportStatus(mqttClient);
    applyChoosenColorToLeds();
  }
  return stateChanged;
}

void MQTTI2CRgbLight::setupActor(MessageQueueClient* mqttClient) {
  establishI2CConnectionTo(lightConfiguration.wirePins.sdaPin, lightConfiguration.wirePins.sclPin, true);
  mqttClient->subscribeTopic(lightConfiguration.lightSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.brightnessSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.colorSetSubscriptionTopic);
  mqttClient->subscribeTopic(lightConfiguration.lightStateTopic);
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