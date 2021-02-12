#include <DeviceRuntime.h>

// extern
DeviceRootConfig *rootConfig = new DeviceRootConfig();
MQTTDeviceService *mqttDeviceService;
MQTTDeviceInfo deviceInfo;

// intern
DNSServer dnsServer;
boolean configureMode = false;

void setupDevice(WiFiClient &espClient, const String deviceId, int resetButtonPin, const String pingId,
                 const String resetSwitchId, void mqttSetupFunction()) {
    mountFileSystem();
    checkForConfigurationReset(resetButtonPin);
    if (rootConfig->read()) {
        setupWifiConnection(rootConfig->wifiSSID, rootConfig->wifiPasswd, rootConfig->getCleanedDeviceName());
        setupMqttDeviceService(espClient, deviceId, pingId, resetSwitchId);
        mqttSetupFunction();
        mqttDeviceService->setupMQTTDevices();
    } else {
        dnsServer = setupSoftAccessPointWithDnsServer("ROBOTronix_" + deviceId, "configure.me");
        configureWebServer();
        configureMode = true;
    }
}

void loopDevice(int delayTimeout) {
    if (configureMode) {
        dnsServer.processNextRequest();
        handleWebServerClient();
    } else {
        checkWifiStatus(rootConfig->wifiSSID, rootConfig->wifiPasswd, rootConfig->getCleanedDeviceName());
        mqttDeviceService->executeLoop();
    }

    delay(delayTimeout);
}

MQTTDeviceInfo getMQTTDeviceInfo() { return deviceInfo; }

void registerMQTTDevice(MQTTPublisher *mqttPublisher) { mqttDeviceService->addPublisher(mqttPublisher); }

void registerMQTTDevice(MQTTStateConsumer *mqttStateConsumer) {
    mqttDeviceService->addStateConsumer(mqttStateConsumer);
}

void checkForConfigurationReset(int resetButtonPin) {
    EasyButton flashButton(resetButtonPin);
    flashButton.begin();
    flashButton.onPressed(onResetRequested);

    for (int i = 0; i < 10; i++) {
        flashButton.read();
        delay(100);
    }
}

void onResetRequested() {
    rootConfig->deleteConfig();
    Serial.println("Configuration resetted!");

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i <= 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
}

void configureStandardMQTTOperations(MQTTDeviceService *deviceService, MQTTDeviceInfo deviceInfo,
                                     const String pingId, const String resetSwitchId) {

    MQTTDevicePing *devicePing = new MQTTDevicePing(deviceInfo, pingId, 30000);
    deviceService->addPublisher(devicePing);
    MQTTDeviceResetSwitch *resetSwitch = new MQTTDeviceResetSwitch(deviceInfo, resetSwitchId);
    deviceService->setResetStateConsumer(resetSwitch);
}

void setupMqttDeviceService(WiFiClient &espClient, const String deviceId, const String pingId,
                            const String resetSwitchId) {
    mqttDeviceService = rootConfig->createMQTTDeviceService(espClient, messageReceived);
    deviceInfo = rootConfig->createMQTTDeviceInfo(deviceId, "Node MCU");
    configureStandardMQTTOperations(mqttDeviceService, deviceInfo, pingId, resetSwitchId);
}

void messageReceived(String &topic, String &payload) {
    Serial.println("incoming: " + topic + " - " + payload);
    mqttDeviceService->handleMessage(topic, payload);
}
