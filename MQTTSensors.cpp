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

MQTTPhotoLightSensor::MQTTPhotoLightSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin, float sensorVoltage)
    : MQTTSensor(new MQTTPhotoLightSensorDeviceClassificationFactory(sensorUniqueId), deviceInfo) {
  this->analogPin = analogPin;
  this->sensorVoltage = sensorVoltage;
}

void MQTTPhotoLightSensor::publishMeasurement() {
  int sensorValue = analogRead(A0);
  float currentVoltage = sensorValue * (sensorVoltage / 1023.0);

  if (!areEqual(lastVoltageValue, currentVoltage, 0.1)) {
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

void MQTTPhotoLightSensor::reset() { lastVoltageValue = -1.0; }

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
