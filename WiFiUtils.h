#ifndef WiFiUtils_h
#define WiFiUtils_h

#include <DNSServer.h>
#include <ESP8266WiFi.h>

void setupWifiConnection(const String ssid, const String password, const String hostname = "",
                         WiFiMode wifiMode = WIFI_STA);
DNSServer setupSoftAccessPointWithDnsServer(String ssid, String domainName);
void checkWifiStatus(const String ssid, const String password, const String hostname = "");
boolean testWifiConnection(const String ssid, const String password, int maxTries = 5);

#endif