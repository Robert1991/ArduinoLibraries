#ifndef WiFiUtils_h
#define WiFiUtils_h

#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <Pinger.h>

boolean setupWifiConnection(const String ssid, const String password, const String hostname = "",
                            WiFiMode wifiMode = WIFI_STA, int maxTries = 10);
DNSServer setupSoftAccessPointWithDnsServer(String ssid, String domainName);
void checkWifiStatus(const String ssid, const String password, const String hostname = "");
boolean testWifiConnection(const String ssid, const String password, int maxTries = 5);
boolean pingServer(String server);
#endif