#include "MQTTSensors.h"

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
  if (!alivePingWasSent) {
    publishPing();
    alivePingWasSent = true;
  }
  unsigned long currentTime = millis();
  if (currentTime - startTime > pingTimeout) {
    publishPing();
    startTime = millis();
  }
}

void MQTTDevicePing::publishPing() {
  publishBinaryMessage(true);
  logLineToSerial("Ping published!");
}

DynamicJsonDocument MQTTDevicePing::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  int pingTimeoutInSeconds = pingTimeout / 1000;
  autoConfigureJsonDocument["off_dly"] = pingTimeoutInSeconds * 2 + 1;
  return autoConfigureJsonDocument;
}

void MQTTDevicePing::reset() { startTime = millis() + pingTimeout + 1; }

MQTTInputPullUpSensorDeviceClassificationFactory::MQTTInputPullUpSensorDeviceClassificationFactory(
    String deviceUniqueId, String deviceClass, String sensorStateName)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {
  this->deviceClass = deviceClass;
  this->sensorStateName = sensorStateName;
}

MQTTDeviceClassification MQTTInputPullUpSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, this->deviceClass, "binary_sensor", sensorStateName,
                                          true};
  return deviceClass;
}

MQTTInputPullUpSensor::MQTTInputPullUpSensor(MQTTDeviceInfo deviceInfo, String uniqueId, int inputPin,
                                             String deviceClass, String sensorStateName)
    : MQTTSensor(new MQTTInputPullUpSensorDeviceClassificationFactory(uniqueId, deviceClass, sensorStateName),
                 deviceInfo) {
  this->inputPin = inputPin;
}

void MQTTInputPullUpSensor::setupSensor() { pinMode(inputPin, INPUT_PULLUP); }

void MQTTInputPullUpSensor::publishMeasurement() {
  int inputPinState = digitalRead(inputPin);

  if (inputPinState == HIGH && !inputPinIsHigh) {
    inputPinIsHigh = true;
    publishBinaryMessage(true);
  } else if (inputPinState == LOW && inputPinIsHigh) {
    inputPinIsHigh = false;
    publishBinaryMessage(false);
  }
}

void MQTTInputPullUpSensor::reset() {
  if (inputPinIsHigh) {
    inputPinIsHigh = false;
  } else {
    inputPinIsHigh = true;
  }
}

MQTTPhotoLightSensorDeviceClassificationFactory::MQTTPhotoLightSensorDeviceClassificationFactory(
    String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTPhotoLightSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "voltage", "sensor", "light_intensity", true};
  return deviceClass;
}

MQTTPhotoLightSensor::MQTTPhotoLightSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin,
                                           float sensorVoltage)
    : MQTTSensor(new MQTTPhotoLightSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->analogPin = analogPin;
  this->sensorVoltage = sensorVoltage;
}

void MQTTPhotoLightSensor::publishMeasurement() {
  int sensorValue = analogRead(analogPin);
  float currentVoltage = sensorValue * (sensorVoltage / 1023.0);

  if (!areEqual(lastVoltageValue, currentVoltage, 0.1)) {
    lastVoltageValue = currentVoltage;
    logToSerial("light sensitivity voltage output changed to: ");
    logLineToSerial(lastVoltageValue);
    publishFloatValue(lastVoltageValue);
  }
}

DynamicJsonDocument
MQTTPhotoLightSensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_meas"] = "V";
  return autoConfigureJsonDocument;
}

void MQTTPhotoLightSensor::reset() { lastVoltageValue = -1.0; }

MQTTBatterySensorDeviceClassificationFactory::MQTTBatterySensorDeviceClassificationFactory(
    String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTBatterySensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "battery", "sensor", "battery_percentage", true};
  return deviceClass;
}

MQTTBatterySensor::MQTTBatterySensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin,
                                     float batteryNamedVoltage, float batteryMinimumVoltage,
                                     float batteryMaximumVoltage)
    : MQTTSensor(new MQTTBatterySensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->analogPin = analogPin;
  this->batteryMinimumVoltage = batteryMinimumVoltage;
  this->batteryMaximumVoltage = batteryMaximumVoltage;
  this->batteryNamedVoltage = batteryNamedVoltage;

  this->attributesTopic = sensorHomeAssistantPath + "/attributes";
}

DynamicJsonDocument
MQTTBatterySensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["json_attr_t"] = attributesTopic;
  autoConfigureJsonDocument["unit_of_meas"] = "%";
  return autoConfigureJsonDocument;
}

void MQTTBatterySensor::publishMeasurement() {
  int sensorValue = analogRead(analogPin);

  float currentVoltage = sensorValue * (batteryNamedVoltage / 1023.00);
  float currentPercentage =
      100 * (currentVoltage - batteryMinimumVoltage) / (batteryMaximumVoltage - batteryMinimumVoltage);
  if (!areEqual(lastMeasuredPercentage, currentPercentage, 0.5)) {
    lastMeasuredPercentage = currentPercentage;
    publishFloatValue(currentPercentage);
    publishAbsoluteVoltage(currentVoltage);
  }
}

void MQTTBatterySensor::publishAbsoluteVoltage(float currentVoltage) {
  DynamicJsonDocument stateAttributes = createJsonDocument(256);
  stateAttributes["current_voltage"] = currentVoltage;
  stateAttributes["battery_full_voltage"] = batteryNamedVoltage;
  stateAttributes["battery_empty_voltage"] = batteryMinimumVoltage;
  publishJsonDocument(attributesTopic, stateAttributes);
}

void MQTTBatterySensor::reset() { this->lastMeasuredPercentage = 0.0; }

MQTTMotionSensorDeviceClassificationFactory::MQTTMotionSensorDeviceClassificationFactory(
    String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTMotionSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "motion", "binary_sensor", "motion_detected", true};
  return deviceClass;
}

MQTTMotionSensor::MQTTMotionSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin,
                                   int motionDetectionIterations, int motionDetectionTimeout,
                                   int motionDetectedThreshold)
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

MQTTDHTSensor::MQTTDHTSensor(MQTTDeviceClassificationFactory *deviceClassFactory, MQTTDeviceInfo deviceInfo,
                             DHT *dhtSensor, int resetValuesIterations)
    : MQTTSensor(deviceClassFactory, deviceInfo) {
  this->dhtSensor = dhtSensor;
  this->resetValuesIterations = resetValuesIterations;
}

void MQTTDHTSensor::setupSensor() { dhtSensor->begin(); }

bool MQTTDHTSensor::resetIfRequired() {
  if (currentIteration < resetValuesIterations) {
    currentIteration++;
    return false;
  }
  logLineToSerial("Resetting stored sensor values");
  currentIteration = 0;
  reset();
  return true;
}

MQTTHumiditySensorDeviceClassificationFactory::MQTTHumiditySensorDeviceClassificationFactory(
    String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTHumiditySensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "humidity", "sensor", "humidity", true};
  return deviceClass;
}

MQTTHumiditySensor::MQTTHumiditySensor(MQTTDeviceInfo deviceInfo, DHT *dhtSensor, String sensorUniqueId,
                                       int resetValuesIterations)
    : MQTTDHTSensor(new MQTTHumiditySensorDeviceClassificationFactory(sensorUniqueId), deviceInfo, dhtSensor,
                    resetValuesIterations) {
  this->dhtSensor = dhtSensor;
}

void MQTTHumiditySensor::publishMeasurement() {
  resetIfRequired();
  publishHumidity();
}

void MQTTHumiditySensor::publishHumidity() {
  float currentHumiditySensorValue = dhtSensor->readHumidity();
  if (!isnan(currentHumiditySensorValue) && currentHumiditySensorValue > 0 &&
      currentHumiditySensorValue < 100) {
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

MQTTTemperatureSensorDeviceClassificationFactory::MQTTTemperatureSensorDeviceClassificationFactory(
    String deviceUniqueId)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {}

MQTTDeviceClassification MQTTTemperatureSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "temperature", "sensor", "temperature", true};
  return deviceClass;
}

MQTTTemperatureSensor::MQTTTemperatureSensor(MQTTDeviceInfo deviceInfo, DHT *dhtSensor, String sensorUniqueId,
                                             int resetValuesIterations)
    : MQTTDHTSensor(new MQTTTemperatureSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo,
                    dhtSensor, resetValuesIterations) {
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

DynamicJsonDocument
MQTTTemperatureSensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["unit_of_meas"] = "°C";
  return autoConfigureJsonDocument;
}

MQTTAnalogConverterSensorDeviceClassificationFactory::MQTTAnalogConverterSensorDeviceClassificationFactory(
    String deviceUniqueId, String sensorName)
    : MQTTDeviceClassificationFactory(deviceUniqueId) {
  this->sensorName = sensorName;
}

MQTTDeviceClassification MQTTAnalogConverterSensorDeviceClassificationFactory::create() {
  MQTTDeviceClassification deviceClass = {deviceUniqueId, "humidity", "sensor", sensorName, true};
  return deviceClass;
}

MQTTAnalogConverterSensor::MQTTAnalogConverterSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId,
                                                     Adafruit_ADS1015 *converter, int analogPort,
                                                     String sensorName, int minValue, int maxValue,
                                                     int valueDelta)
    : MQTTSensor(new MQTTAnalogConverterSensorDeviceClassificationFactory(sensorUniqueId, sensorName),
                 deviceInfo) {
  this->converter = converter;
  this->analogPort = analogPort;
  this->valueDelta = valueDelta;
  this->minValue = minValue;
  this->maxValue = maxValue;

  this->attributesTopic = sensorHomeAssistantPath + "/attributes";
}

void MQTTAnalogConverterSensor::setupSensor() { converter->begin(); }

DynamicJsonDocument
MQTTAnalogConverterSensor::extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument) {
  autoConfigureJsonDocument["json_attr_t"] = attributesTopic;
  autoConfigureJsonDocument["unit_of_meas"] = "%";
  return autoConfigureJsonDocument;
}

void MQTTAnalogConverterSensor::publishMeasurement() {
  int16_t currentSensorValue = converter->readADC_SingleEnded(analogPort);
  if (abs(lastValue - currentSensorValue) > valueDelta) {
    lastValue = currentSensorValue;
    float percentage = calculatePercentageValue(currentSensorValue);
    logToSerial("Current Percentage: ");
    logLineToSerial(percentage);
    logToSerial("Analog sensor value from port ");
    logToSerial(analogPort);
    logToSerial(": ");
    logLineToSerial(currentSensorValue);
    publishFloatValue(percentage);
    publishAbsoluteValues(currentSensorValue);
  }
}

float MQTTAnalogConverterSensor::calculatePercentageValue(int currentSensorValue) {
  float percentage = 100.0 * (1.0 * currentSensorValue - minValue) / (maxValue - minValue);
  // Value needs to be inverted, because 100.0 is dry
  if (percentage >= 100.0) {
    return 0.0;
  } else if (percentage <= 0.0) {
    return 100.0;
  }
  return 100.0 - percentage;
}

void MQTTAnalogConverterSensor::publishAbsoluteValues(int currentValue) {
  DynamicJsonDocument stateAttributes = createJsonDocument(256);
  stateAttributes["raw"] = currentValue;
  stateAttributes["min_value"] = minValue;
  stateAttributes["max_value"] = maxValue;
  publishJsonDocument(attributesTopic, stateAttributes);
}

void MQTTAnalogConverterSensor::reset() { lastValue = 0; }
