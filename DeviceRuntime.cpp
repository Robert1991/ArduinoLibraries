#include <DeviceRuntime.h>

// extern
DeviceRootConfig *rootConfig = new DeviceRootConfig();
MQTTDeviceService *mqttDeviceService;
MQTTDeviceInfo deviceInfo;

// intern
DeviceUpdateService *deviceUpdateService;
DNSServer dnsServer;
boolean configureMode = false;
unsigned long updateTimer = millis();
const int UPDATE_TIMEOUT = 60000;

void setupDevice(WiFiClient &espClient, const String deviceId, const int buildNumber, int resetButtonPin,
                 const String pingId, String resetStateSwitchId, void mqttSetupFunction()) {
  mountFileSystem();
  checkForConfigurationReset(resetButtonPin);
  if (rootConfig->read()) {
    wifi_connection_status connection_status = setupWifiConnection(
        rootConfig->wifiSSID, rootConfig->wifiPasswd, rootConfig->getCleanedDeviceName(), WIFI_STA, 12);
    if (connection_status == CONNECTED) {
      setupMqttDeviceService(espClient, deviceId, buildNumber, pingId, resetStateSwitchId);
      mqttSetupFunction();
      mqttDeviceService->setupMQTTDevices();
      if (rootConfig->updateServerSet()) {
        String deviceVersion = deviceId + "-" + VERSION + "-" + buildNumber;
        setupUpdateService(espClient, rootConfig->updateServer, rootConfig->updateServerPort, deviceVersion);
      }
      return;
    } else if (connection_status == CONNECTION_NOT_POSSIBLE) {
      Serial.println("WiFi Connection failed, restarting device...");
      ESP.restart();
    }
    Serial.println("WiFi Connection failed. SSID or password wrong, starting access point...");
  }
  dnsServer = setupSoftAccessPointWithDnsServer(MANUFACTURER + "_" + deviceId, "configure.me");
  configureWebServer();
  configureMode = true;
}

void loopDevice(int delayTimeout) {

  if (configureMode) {
    dnsServer.processNextRequest();
    handleWebServerClient();
  } else {
    if (deviceUpdateService && (millis() - updateTimer) >= UPDATE_TIMEOUT) {
      deviceUpdateService->installUpdateIfPossible();
      updateTimer = millis();
    }
    checkWifiStatus(rootConfig->wifiSSID, rootConfig->wifiPasswd, rootConfig->getCleanedDeviceName(),
                    WIFI_STA, 8);
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

void setupMqttDeviceService(WiFiClient &espClient, const String deviceId, const int buildNumber,
                            const String pingId, const String resetStateSwitchId) {
  mqttDeviceService = rootConfig->createMQTTDeviceService(espClient, messageReceived);
  deviceInfo = rootConfig->createMQTTDeviceInfo(deviceId, buildNumber, "Node MCU");
  configureStandardMQTTOperations(mqttDeviceService, deviceInfo, pingId, resetStateSwitchId);
}

void configureStandardMQTTOperations(MQTTDeviceService *deviceService, MQTTDeviceInfo deviceInfo,
                                     const String pingId, const String resetStateSwitchId) {

  MQTTDevicePing *devicePing = new MQTTDevicePing(deviceInfo, pingId, 30000);
  deviceService->addPublisher(devicePing);
  MQTTDeviceResetSwitch *resetStateSwitch =
      new MQTTDeviceResetSwitch(deviceInfo, resetStateSwitchId, "reset_switch");
  deviceService->setResetStateConsumer(resetStateSwitch);
}

void setupUpdateService(WiFiClient &client, String server, int port, String deviceVersion) {
  deviceUpdateService = new DeviceUpdateService(client, server, port, deviceVersion);
  deviceUpdateService->setup();
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  mqttDeviceService->handleMessage(topic, payload);
}
