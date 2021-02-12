#ifndef DeviceRuntime_h
#define DeviceRuntime_h

#include <AccessPointWebServer.h>
#include <DeviceRootConfig.h>
#include <WiFiUtils.h>

extern DeviceRootConfig *rootConfig;
extern MQTTDeviceInfo deviceInfo;
extern MQTTDeviceService *mqttDeviceService;

// externs
void setupDevice(WiFiClient &espClient, const String deviceId, int resetButtonPin, const String pingId,
                 const String resetSwitchId, void mqttSetupFunction());
void loopDevice(int delayTimeout);
MQTTDeviceInfo getMQTTDeviceInfo();
void registerMQTTDevice(MQTTPublisher *mqttPublisher);
void registerMQTTDevice(MQTTStateConsumer *stateConsumer);

// interns
// mqtt control
void messageReceived(String &topic, String &payload);
void configureStandardMQTTOperations(MQTTDeviceService *deviceService, MQTTDeviceInfo deviceInfo,
                                     const String pingId, const String resetSwitchId);
void setupMqttDeviceService(WiFiClient &espClient, const String deviceId, const String pingId,
                            const String resetSwitchId);
// resets
void checkForConfigurationReset(int resetButtonPin);
void onResetRequested();

#endif
