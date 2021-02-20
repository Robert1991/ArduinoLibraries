#include <WiFiUtils.h>

boolean setupWifiConnection(const String ssid, const String password, const String hostName,
                            WiFiMode wifiMode, int maxTries) {
  if (hostName.length() > 0) {
    Serial.print("setting hostname: ");
    Serial.println(hostName);
    WiFi.hostname(hostName);
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println(" ...");

  WiFi.mode(wifiMode);

  int currentTry = 0;
  while (WiFi.status() != WL_CONNECTED && currentTry < maxTries) {
    currentTry += 1;
    Serial.println("Waiting for connection...");
    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connection established!");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());
    return true;
  }
  return false;
}

boolean testWifiConnection(const String ssid, const String password, int maxTries) {
  WiFi.begin(ssid, password);
  Serial.print("Testing connection to: ");
  Serial.println(ssid);
  Serial.print("With password: ");
  Serial.println(password);
  int currentTry = 0;
  while (WiFi.status() != WL_CONNECTED && currentTry < maxTries) {
    Serial.println("Waiting for connection...");
    delay(1000);
    currentTry++;
  }
  delay(500);
  boolean wifiConnected = WiFi.status() == WL_CONNECTED;

  if (wifiConnected) {
    Serial.println("Connecting succeeded!");
  } else {
    Serial.println("Connecting failed!");
  }

  return wifiConnected;
}

DNSServer setupSoftAccessPointWithDnsServer(String ssid, String domainName) {
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid) ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  const byte DNS_PORT = 53;
  DNSServer dnsServer;
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  dnsServer.start(DNS_PORT, domainName, WiFi.softAPIP());

  return dnsServer;
}

unsigned long last_wifi_reconnect_attempt = 0;

void checkWifiStatus(const String ssid, const String password, const String hostname) {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - last_wifi_reconnect_attempt > 20000UL || last_wifi_reconnect_attempt == 0) {
      Serial.println("Attempting to connect to WiFi");
      last_wifi_reconnect_attempt = now;
      setupWifiConnection(ssid, password, hostname);
    }
    return;
  }
}

Pinger pinger;

boolean pingServer(String server) {
  IPAddress invalid(255, 255, 255, 255);
  IPAddress serverIP;

  WiFi.hostByName(server.c_str(), serverIP);
  Serial.print("Resolved server url: " + server + " to ip: ");
  Serial.println(serverIP);
  if (serverIP == invalid) {
    return false;
  }
  return pinger.Ping(serverIP);
}
