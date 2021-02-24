#ifndef MQTTSensors_h
#define MQTTSensors_h

#include "Adafruit_ADS1015.h"
#include "DHT.h"
#include "MQTTDevice.h"

class MQTTDevicePingDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTDevicePingDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTDevicePing : public MQTTSensor {
private:
  long pingTimeout;
  unsigned long startTime;
  bool pingWasSentInLastIteration = false;
  bool alivePingWasSent = false;

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void publishPing();

public:
  MQTTDevicePing(MQTTDeviceInfo deviceInfo, String uniqueId, long pingTimeout = 60000);
  void publishMeasurement();
  void reset();
};

class MQTTDoorSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTDoorSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTDoorSensor : public MQTTSensor {
private:
  bool doorIsOpen = false;
  int doorSensorPin;

public:
  MQTTDoorSensor(MQTTDeviceInfo deviceInfo, String uniqueId, int doorSensorPin);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

class MQTTPhotoLightSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTPhotoLightSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTPhotoLightSensor : public MQTTSensor {
private:
  int analogPin;
  float sensorVoltage;
  float lastVoltageValue = 0.0;

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);

public:
  MQTTPhotoLightSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin,
                       float sensorVoltage = 3.3);

  void publishMeasurement();
  void reset();
};

class MQTTMotionSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTMotionSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTMotionSensor : public MQTTSensor {
private:
  int motionSensorPin;
  bool lastMotionSensorState = false;
  int motionDetectionIterations;
  int motionDetectionTimeout;
  int motionDetectedThreshold;

public:
  MQTTMotionSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int motionSensorPin,
                   int motionDetectionIterations = 10, int motionDetectionTimeout = 15,
                   int motionDetectedThreshold = 5);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

class MQTTBatterySensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTBatterySensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTBatterySensor : public MQTTSensor {
private:
  String attributesTopic;
  float batteryMinimumVoltage = 0.0;
  float batteryMaximumVoltage = 0.0;
  float batteryNamedVoltage = 0.0;
  float lastMeasuredPercentage = 0.0;

  int analogPin = 0;

  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void publishAbsoluteVoltage(float currentVoltage);

public:
  MQTTBatterySensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, int analogPin,
                    float batteryNamedVoltage, float batteryMinimumVoltage, float batteryMaximumVoltage);

  void publishMeasurement();
  void reset();
};

class MQTTDHTSensor : public MQTTSensor {
private:
  int currentIteration = 0;
  int resetValuesIterations;

protected:
  DHT *dhtSensor;
  bool resetIfRequired();

public:
  MQTTDHTSensor(MQTTDeviceClassificationFactory *deviceClassFactory, MQTTDeviceInfo deviceInfo,
                DHT *dhtSensor, int resetValuesIterations = 5000);
  void setupSensor();
  virtual void publishMeasurement() = 0;
  virtual void reset() = 0;
};

class MQTTTemperatureSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTTemperatureSensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTHumiditySensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
public:
  MQTTHumiditySensorDeviceClassificationFactory(String deviceUniqueId);
  MQTTDeviceClassification create();
};

class MQTTHumiditySensor : public MQTTDHTSensor {
private:
  DHT *dhtSensor;
  float lastMeasuredHumidity = 0.0;

  void publishHumidity();

public:
  MQTTHumiditySensor(MQTTDeviceInfo deviceInfo, DHT *dhtSensor, String sensorUniqueId,
                     int resetValuesIterations = 5000);
  void publishMeasurement();
  void reset();
};

class MQTTTemperatureSensor : public MQTTDHTSensor {
private:
  DHT *dhtSensor;
  float lastMeasuredTemperature = 0.0;

  void publishTemperature();

public:
  MQTTTemperatureSensor(MQTTDeviceInfo deviceInfo, DHT *dhtSensor, String sensorUniqueId,
                        int resetValuesIterations = 5000);
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);
  void publishMeasurement();
  void reset();
};

class MQTTAnalogConverterSensorDeviceClassificationFactory : public MQTTDeviceClassificationFactory {
private:
  String sensorName;

public:
  MQTTAnalogConverterSensorDeviceClassificationFactory(String deviceUniqueId, String sensorName);
  MQTTDeviceClassification create();
};

class MQTTAnalogConverterSensor : public MQTTSensor {
private:
  String attributesTopic;
  Adafruit_ADS1015 *converter;
  int analogPort;
  int valueDelta;
  int minValue;
  int maxValue;
  int16_t lastValue = 0;

  float calculatePercentageValue(int currentSensorValue);
  void publishAbsoluteValues(int currentValue);
  DynamicJsonDocument extendAutoDiscoveryInfo(DynamicJsonDocument autoConfigureJsonDocument);

public:
  MQTTAnalogConverterSensor(MQTTDeviceInfo deviceInfo, String sensorUniqueId, Adafruit_ADS1015 *converter,
                            int analogPort, String sensorName, int minValue, int maxValue,
                            int valueDelta = 100);
  void setupSensor();
  void publishMeasurement();
  void reset();
};

#endif