#include <AccessPointWebServer.h>

ESP8266WebServer webServer(80);

boolean lastWifiConnectionCheckSuccessful = true;
boolean lastMQTTConnectionCheckSuccessful = true;
boolean lastInputWasIncomplete = false;
boolean connectionWasTested = false;

ESP8266WebServer *configureWebServer() {
    webServer.onNotFound([]() { webServer.send(404, "text/html", SIDE_NOT_FOUND); });
    webServer.on("/", handleRoot);
    webServer.on("/config_submit", handleConfigSubmit);
    webServer.on("/reset", handleReset);
    webServer.begin();
    return &webServer;
}

void handleWebServerClient() { webServer.handleClient(); }

void handleReset() {
    Serial.println("reset called");
    DeviceRootConfig *deviceConfig = new DeviceRootConfig();
    deviceConfig->deleteConfig();
    resetSettingsCheck();
    sendToClient(buildSettingsPage());
}

void handleRoot() {
    Serial.println("root page called");
    sendToClient(buildSettingsPage());
}

void handleConfigSubmit() {
    Serial.println("Settings submitted");
    for (uint8_t i = 0; i < webServer.args(); i++) {
        Serial.println(webServer.argName(i));
        Serial.println(webServer.arg(i));
    }

    if (submittedSettingsAreComplete()) {
        sendToClient(DEVICE_INITIALIZATION_PAGE);
        delay(500);

        if (submittedSettingsAreValid()) {
            storeSubmittedSettings();
            connectionWasTested = true;
        }
    } else {
        connectionWasTested = true;
        lastInputWasIncomplete = true;
        sendToClient(buildSettingsPage());
    }
}

void resetSettingsCheck() {
    lastWifiConnectionCheckSuccessful = true;
    lastMQTTConnectionCheckSuccessful = true;
    lastInputWasIncomplete = false;
    connectionWasTested = false;
}

boolean submittedSettingsAreComplete() {
    return webServer.arg("wifissid").length() > 0 && webServer.arg("wifipasswd").length() > 0 &&
           webServer.arg("mqttbroker").length() > 0 && webServer.arg("mqttbroker_port").length() > 0 &&
           webServer.arg("device_name").length() > 0 &&
           webServer.arg("homeassistant_autoconfig_prefix").length() > 0;
}

boolean submittedSettingsAreValid() {
    int maxTries = 8;
    lastWifiConnectionCheckSuccessful =
        testWifiConnection(webServer.arg("wifissid"), webServer.arg("wifipasswd"), maxTries);
    if (lastWifiConnectionCheckSuccessful) {
        lastMQTTConnectionCheckSuccessful = testMqttClient();
        return lastMQTTConnectionCheckSuccessful;
    }
    return false;
}

boolean testMqttClient() {
    MessageQueueClient *mqttClient = nullptr;

    if (webServer.arg("mqttbroker_username").length() > 0 &&
        webServer.arg("mqttbroker_password").length() > 0) {
        mqttClient = new MessageQueueClient("TestClient", webServer.arg("mqttbroker_username"),
                                            webServer.arg("mqttbroker_password"));
    } else if (webServer.arg("mqttbroker_username").length() > 0) {
        mqttClient = new MessageQueueClient("TestClient", webServer.arg("mqttbroker_username"));
    } else {
        mqttClient = new MessageQueueClient("TestClient");
    }

    WiFiClient wifiClient;
    mqttClient->setVerbose(true);
    String brokerUrl = webServer.arg("mqttbroker");
    int brokerPort = webServer.arg("mqttbroker_port").toInt();
    if (mqttClient->setupClient(wifiClient, nullptr, brokerUrl, brokerPort)) {
        return true;
    }
    return false;
}

void storeSubmittedSettings() {
    DeviceRootConfig *deviceConfig = new DeviceRootConfig();
    deviceConfig->wifiSSID = webServer.arg("wifissid");
    deviceConfig->wifiPasswd = webServer.arg("wifipasswd");
    deviceConfig->mqttBroker = webServer.arg("mqttbroker");
    deviceConfig->mqttBrokerPort = webServer.arg("mqttbroker_port").toInt();

    if (webServer.arg("mqttbroker_username").length()) {
        deviceConfig->mqttUser = webServer.arg("mqttbroker_username");
    }
    if (webServer.arg("mqttbroker_password").length()) {
        deviceConfig->mqttPasswd = webServer.arg("mqttbroker_password");
    }
    deviceConfig->deviceName = webServer.arg("device_name");
    deviceConfig->homeassistantAutoConfigurePrefix = webServer.arg("homeassistant_autoconfig_prefix");
    deviceConfig->writeCurrent();
}

void sendToClient(String response) { webServer.send(200, "text/html", response); }

String buildSettingsPage() {
    String settingsPageHead = "<html lang=\"en\">" + SETTINGS_PAGE_HEAD;
    String settingsPageBody = SETTINGS_PAGE_BODY_HEADER;
    if (connectionWasTested && lastWifiConnectionCheckSuccessful && lastMQTTConnectionCheckSuccessful) {
        settingsPageBody += SETTINGS_SUCCESSFULLY_SETUP_MESSAGE;
    } else {
        if (!lastWifiConnectionCheckSuccessful) {
            settingsPageBody += WIFI_CONNECTION_FAILED_MESSAGE;
        } else if (!lastMQTTConnectionCheckSuccessful) {
            settingsPageBody += MQTT_CONNECTION_FAILED_MESSAGE;
        } else if (lastInputWasIncomplete) {
            settingsPageBody += SETTINGS_INCOMPLETE_MESSAGE;
        }
        settingsPageBody += SETTINGS_PAGE_SUBMIT_FORM + "</body>";
    }

    return settingsPageHead + settingsPageBody + "</html>";
}