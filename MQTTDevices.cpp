#include "MQTTDevices.h"

MQTTDeviceClassificationFactory::MQTTDeviceClassificationFactory(String deviceUniqueId) { this->deviceUniqueId = deviceUniqueId; }

MQTTDevice::MQTTDevice(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) {
  MQTTDeviceClassification deviceClass = deviceClassFactory->create();
  assignDeviceInfos(deviceClass, deviceInfo);
}

void MQTTDevice::assignDeviceInfos(MQTTDeviceClassification deviceClass, MQTTDeviceInfo deviceInfo) {
  this->deviceEntityName = deviceInfo.deviceName + "_" + deviceClass.sensorType;
  String sensorHomeAssistantPath = deviceInfo.autoDiscoveryPrefix + "/" + deviceClass.deviceType + "/" + deviceEntityName;
  this->stateTopic = sensorHomeAssistantPath + "/state";
  this->autoDiscoveryMQTTConfigureTopic = sensorHomeAssistantPath + "/config";
  this->deviceClassification = deviceClass;
  this->deviceInfo = deviceInfo;
}

int MQTTDevice::publishAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  return publishJsonDocument(autoDiscoveryMQTTConfigureTopic, autoConfigureJsonDocument, true);
}

int MQTTDevice::publishTo(String topic, String payload, bool retain) { return mqttClient->publishMessage(topic, payload, retain); }

int MQTTDevice::publishState(String stateTopicPayload) { return publishTo(stateTopic, stateTopicPayload); }

int MQTTDevice::publishJsonDocument(String stateTopic, DynamicJsonDocument jsonDocument, bool retain) {
  String jsonMessage;
  serializeJson(jsonDocument, jsonMessage);
  logToSerial("Publishing to: ");
  logToSerial(stateTopic);
  logLineToSerial(" with payload: ");
  logLineToSerial(jsonMessage);
  mqttClient->publishMessage(stateTopic, jsonMessage, retain);
  return 1;
}

void MQTTDevice::subscribeTopic(String subscribeTopic) { return mqttClient->subscribeTopic(subscribeTopic); }

DynamicJsonDocument MQTTDevice::createDeviceInfoJsonObject() {
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(12) + 540;
  DynamicJsonDocument autoConfigureJsonDocument(capacity);
  autoConfigureJsonDocument["name"] = deviceEntityName;
  if (deviceClassification.deviceClassIsAvailable) {
    autoConfigureJsonDocument["dev_cla"] = deviceClassification.deviceClass;
  }
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
    delay(500);

    DynamicJsonDocument autoConfigureJsonDocument = createDeviceInfoJsonObject();
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

DynamicJsonDocument MQTTDevice::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) { return autoConfigureJsonDocument; }

MQTTSensor::MQTTSensor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) : MQTTDevice(deviceClassFactory, deviceInfo) {}

void MQTTSensor::initializePublisher(MessageQueueClient* mqttClient) {
  this->mqttClient = mqttClient;
  setupSensor();
}

void MQTTSensor::configureInTargetPlatform() { configureViaBroker(); }

void MQTTSensor::publishToTargetPlatform() { publishMeasurement(); }

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

bool MQTTSensor::areEqual(float value1, float value2, float maxDifference) {
  if (fabs(value1 - value2) > maxDifference) {
    return false;
  }
  return true;
}

MQTTDevicePingDeviceClassificationFactory::MQTTDevicePingDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTDevicePingDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "connectivity", "binary_sensor", "ping", true};
  return deviceClass;
}

MQTTDevicePing::MQTTDevicePing(MQTTDeviceInfo deviceInfo, String uniqueId, long pingTimeout)
    : MQTTSensor(new MQTTDevicePingDeviceClassificationFactory(uniqueId), deviceInfo) {
  this->pingTimeout = pingTimeout;
  // We want an immediate ping
  startTime = millis() + pingTimeout + 1;
}

void MQTTDevicePing::publishMeasurement() {
  unsigned long currentTime = millis();
  if (currentTime - startTime > pingTimeout) {
    publishBinaryMessage(true);
    logLineToSerial("Ping published!");
    pingWasSentInLastIteration = true;
    startTime = millis();
  }
}

DynamicJsonDocument MQTTDevicePing::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  int pingTimeoutInSeconds = pingTimeout / 1000;
  autoConfigureJsonDocument["off_dly"] = pingTimeoutInSeconds * 2 + 1;
  return autoConfigureJsonDocument;
}

void MQTTDevicePing::reset() { startTime = millis() + pingTimeout + 1; }

MQTTPhotoLightSensorDeviceClassificationFactory::MQTTPhotoLightSensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTPhotoLightSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "voltage", "sensor", "light_intensity", true};
  return deviceClass;
}

MQTTPhotoLightSensor::MQTTPhotoLightSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin)
    : MQTTSensor(new MQTTPhotoLightSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->analogPin = analogPin;
}

void MQTTPhotoLightSensor::publishMeasurement() {
  int sensorValue = analogRead(A0);
  float currentVoltage = sensorValue * (3.3 / 1023.0);

  if (!areEqual(lastVoltageValue, currentVoltage, 0.2)) {
    lastVoltageValue = currentVoltage;
    logToSerial("light sensitivity voltage output changed to: ");
    logLineToSerial(lastVoltageValue);
    publishFloatValue(lastVoltageValue);
  }
}

DynamicJsonDocument MQTTPhotoLightSensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_measurement"] = "V";
  return autoConfigureJsonDocument;
}

void MQTTPhotoLightSensor::reset() { lastVoltageValue = 0.0; }

MQTTMotionSensorDeviceClassificationFactory::MQTTMotionSensorDeviceClassificationFactory(String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTMotionSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "motion", "binary_sensor", "motion_detected", true};
  return deviceClass;
}

MQTTMotionSensor::MQTTMotionSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin, int motionDetectionIterations,
                                   int motionDetectionTimeout, int motionDetectedThreshold)
    : MQTTSensor(new MQTTMotionSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
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

MQTTDHTSensor::MQTTDHTSensor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo, DHT* dhtSensor,
                             int resetValuesIterations)
    : MQTTSensor(deviceClassFactory, deviceInfo) {
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
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "humidity", "sensor", "humidity", true};
  return deviceClass;
}

MQTTHumiditySensor::MQTTHumiditySensor(MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId, int resetValuesIterations)
    : MQTTDHTSensor(new MQTTHumiditySensorDeviceClassificationFactory(sensorUniqueId), deviceInfo, dhtSensor, resetValuesIterations) {
  this->dhtSensor = dhtSensor;
}

void MQTTHumiditySensor::publishMeasurement() {
  resetIfRequired();
  publishHumidity();
}

void MQTTHumiditySensor::publishHumidity() {
  float currentHumiditySensorValue = dhtSensor->readHumidity();
  if (!isnan(currentHumiditySensorValue) && currentHumiditySensorValue > 0 && currentHumiditySensorValue < 100) {
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
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "temperature", "sensor", "temperature", true};
  return deviceClass;
}

MQTTTemperatureSensor::MQTTTemperatureSensor(MQTTDeviceInfo deviceInfo, DHT* dhtSensor, String sensorUniqueId, int resetValuesIterations)
    : MQTTDHTSensor(new MQTTTemperatureSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo, dhtSensor, resetValuesIterations) {
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

DynamicJsonDocument MQTTTemperatureSensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_measurement"] = "°C";
  autoConfigureJsonDocument["expire_after"] = 300;
  return autoConfigureJsonDocument;
}

MQTTActor::MQTTActor(MQTTDeviceClassificationFactory* deviceClassFactory, MQTTDeviceInfo deviceInfo) : MQTTDevice(deviceClassFactory, deviceInfo) {}

void MQTTActor::configureInTargetPlatform() { configureViaBroker(); }

void MQTTActor::initializePublisher(MessageQueueClient* mqttClient) {
  this->mqttClient = mqttClient;
  setupActor();
}

void MQTTActor::publishToTargetPlatform() { reportStatus(); }

void MQTTActor::executeLoopMethod() {}

void MQTTActor::reportStatus() {
  if (actorStatusChanged) {
    reportStatusInformation();
    actorStatusChanged = false;
  }
}

void MQTTActor::reset() { actorStatusChanged = true; }

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