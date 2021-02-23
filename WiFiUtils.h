#ifndef WiFiUtils_h
#define WiFiUtils_h

#include "user_interface.h"
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <Pinger.h>

typedef enum { CONNECTED, WRONG_PASSWORD, SSID_NOT_FOUND, CONNECTION_NOT_POSSIBLE } wifi_connection_status;

wifi_connection_status setupWifiConnection(const String ssid, const String password,
                                           const String hostname = "", WiFiMode wifiMode = WIFI_STA,
                                           int maxTries = 10);
DNSServer setupSoftAccessPointWithDnsServer(String ssid, String domainName);
void checkWifiStatus(const String ssid, const String password, const String hostname = "");

// dropping in 0 as maxTries lets the station try forever
boolean testWifiConnection(const String ssid, const String password, int maxTries = 5);
boolean pingServer(String server);
#endif