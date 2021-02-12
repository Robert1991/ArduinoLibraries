#ifndef AccessPointWebServer_h
#define AccessPointWebServer_h

#include "SerialLogger.h"
#include <DeviceRootConfig.h>
#include <ESP8266WebServer.h>
#include <WiFiUtils.h>

const String SIDE_NOT_FOUND = "<!DOCTYPE html>"
                              "<html lang=\"en\">"
                              "<head>"
                              "<meta charset=\"utf-8\">"
                              "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                              "<title>Internet of Bottles</title>"
                              "</head>"
                              "<body>"
                              "<p>I'm just a stupid bottle with WiFi.</p>"
                              "</body>"
                              "</html>";

const String DEVICE_INITIALIZATION_PAGE =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<title>Configuring Device</title>"
    "</head>"
    "<body>"
    "<h1>Device Initialization</h1>"
    "<p>Checking configuration, this can take up to 10 seconds or longer. Check the following link to see if "
    "it works (refreshs maybe needed).</p>"
    "<a href=\"/\">Check Configuration Status</a>"
    "</body>"
    "</html>";

const String SETTINGS_PAGE_BODY_HEADER = R"=====(
                            <style>
                            .button {
                                border: none;
                                color: white;
                                padding: 12px 30px;
                                text-align: center;
                                text-decoration: none;
                                display: inline-block;
                                font-size: 16px;
                                margin: 4px 2px;
                                cursor: pointer;
                              }

                              .button1 {background-color: #ff3333;} /* Green */
                              .button2 {background-color: #008CBA;} /* Blue */
                            </style>
                            <body>
                            <h1>ROBOTronix Device Configuration</h1>
                            )=====";

const String SETTINGS_PAGE_SUBMIT_FORM = R"=====(
                              <form action="/config_submit" method="post">
                              <fieldset>
                              <legend>Wifi Configuration</legend>
                                <p>
                                  <label for="wifissid">Wifi SSID:</label><input id="wifissid" name="wifissid" type="text"/><br /><br />
                                  <label for="wifipasswd">Wifi Password:</label> <input id="wifipasswd" name="wifipasswd" type="password" /><br /><br />
                                </p>
                              </fieldset>
                              <fieldset>
                              <legend>MQTT Broker Settings</legend>
                                <p>
                                  <label for="mqttbroker">MQTT Broker:</label> <input id="mqttbroker" name="mqttbroker" type="text" /><br /><br />
                                  <label for="mqttbroker">MQTT Broker Port:</label> <input id="mqttbroker_port" name="mqttbroker_port" type="number" value="1883"/><br /><br />
                                  <label for="mqttbroker_username">MQTT Broker Username:</label> <input id="mqttbroker_username" name="mqttbroker_username" type="text" /><br /><br />
                                  <label for="mqttbroker_password">MQTT Broker Password:</label> <input id="mqttbroker_password" name="mqttbroker_password" type="password" /><br /><br />
                                </p>
                              </fieldset>
                              <fieldset>
                              <legend>Homeassistant Settings</legend>
                                <p>
                                  <label for="device_name">Device Name:</label> <input id="device_name" name="device_name" type="text" value="change me"/><br /><br />
                                  <label for="homeassistant_autoconfig_prefix">Homeassistant Autoconfigure Prefix:</label> <input id="homeassistant_autoconfig_prefix" name="homeassistant_autoconfig_prefix" type="text" value="homeassistant"/><br /><br />
                                </p>
                              </fieldset>
                              <input type="submit" value="Submit" class="button button2"/>
                              </form>
                              )=====";

const String SETTINGS_PAGE_HEAD = R"=====(
                              <head>
                              <title>Device Configuration</title>
                              </head>
                              )=====";

const String WIFI_CONNECTION_FAILED_MESSAGE = R"=====(
                              <div style="background-color:#f44336; padding: 18px">
                                <strong>Wifi Check Failed!</strong> Could not connect to Wifi with given SSID and password
                              </div>
                              )=====";

const String MQTT_CONNECTION_FAILED_MESSAGE = R"=====(
                              <div style="background-color:#f44336; padding: 18px">
                                <strong>MQTT Connection could not be established!</strong> Check the given MQTT broker in combination with the provided credentials
                              </div>
                              )=====";

const String SETTINGS_INCOMPLETE_MESSAGE = R"=====(
                              <div style="background-color:#f44336; padding: 18px">
                                <strong>Incomplete Settings!</strong>
                              </div>
                              )=====";

const String SETTINGS_SUCCESSFULLY_SETUP_MESSAGE = R"=====(
                              <div style="background-color:#00ff99; padding: 18px">
                                <strong>Success!</strong> Device was successfully setup. Restart/Reset to use device.
                              </div>
                              <a href="/reset" class="button button1">Reset Configuration</a>
                              )=====";

ESP8266WebServer *configureWebServer();
void handleRoot();
void handleWebServerClient();
void handleConfigSubmit();
void handleReset();
String buildSettingsPage();
boolean submittedSettingsAreComplete();
boolean submittedSettingsAreValid();
boolean testMqttClient();
void resetSettingsCheck();
void storeSubmittedSettings();
void sendToClient(String response);
#endif