#ifndef DeviceRuntime_h
#define DeviceRuntime_h

#include <AccessPointWebServer.h>
#include <DeviceRootConfig.h>
#include <DeviceUpdateService.h>
#include <VERSION.h>

extern DeviceRootConfig *rootConfig;
extern MQTTDeviceInfo deviceInfo;
extern MQTTDeviceService *mqttDeviceService;

// externs
void setupDevice(WiFiClient &espClient, const String deviceId, const int buildNumber, int resetButtonPin,
                 const String pingId, const String resetStateSwitchId, void mqttSetupFunction());
void loopDevice(int delayTimeout);
MQTTDeviceInfo getMQTTDeviceInfo();
void registerMQTTDevice(MQTTPublisher *mqttPublisher);
void registerMQTTDevice(MQTTStateConsumer *stateConsumer);

// interns
// mqtt control
void messageReceived(String &topic, String &payload);
void configureStandardMQTTOperations(MQTTDeviceService *deviceService, MQTTDeviceInfo deviceInfo,
                                     const String pingId, const String resetStateSwitchId);
void setupMqttDeviceService(WiFiClient &espClient, const String deviceId, const int buildNumber,
                            const String pingId, const String resetStateSwitchId);

void setupUpdateService(WiFiClient &client, String server, int port, String deviceVersion);

// resets
void checkForConfigurationReset(int resetButtonPin);
void onResetRequested();

#endif
