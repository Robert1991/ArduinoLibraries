#include <IOTClients.h>

MQTTClient::MQTTClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->userName = userName;
  this->password = password;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
  subscribeTopicArray = new char*[subscriptionTopicBufferSize];
}

void MQTTClient::setupClient(PubSubClient* pubSubClient, const char* mqttBroker, const int mqttPort) {
  this->pubSubClient = pubSubClient;
  pubSubClient->setServer(mqttBroker, mqttPort);
  this->reconnect();
}

int MQTTClient::publishMessage(char* topic, char* payload, bool retain) {
  if (!pubSubClient->connected()) {
    this->reconnect();
  }
  return pubSubClient->publish(topic, payload, retain);
}

void MQTTClient::subscribeTopic(char* topic) {
  if (subscribeTopicCount < subscriptionTopicBufferSize) {
    subscribeTopicArray[subscribeTopicCount] = topic;
    subscribeTopicCount++;
  }
}

void MQTTClient::subscribeTopics(char** subscribeTopics, int countOfTopics) {
  for (int i = 0; i < countOfTopics; i++) {
    subscribeTopicArray[subscribeTopicCount] = subscribeTopics[i];
    subscribeTopicCount++;
  }
}

bool MQTTClient::reconnect(int connectTimeout, int reconnectTries) {
  logToSerial("connect...");

  int currentTry = 0;
  bool connectionEstablished = false;
  while (!pubSubClient->connected() && (currentTry < reconnectTries)) {
    logLineToSerial("Reconnecting...");
    connectionEstablished = pubSubClient->connect(clientName, userName, password);
    if (!connectionEstablished) {
      logToSerial("failed, return state=");
      logToSerial(pubSubClient->state());
      logToSerial(" retrying in ");
      logToSerial(connectTimeout);
      logLineToSerial(" milli seconds");
      delay(connectTimeout);
      currentTry++;
    } else {
      break;
    }
  }
  if (connectionEstablished) {
    for (int i = 0; i < subscribeTopicCount; i++) {
      logToSerial("Subscribing: ");
      logLineToSerial(subscribeTopicArray[i]);
      boolean subscriptionSuccessful = pubSubClient->subscribe(subscribeTopicArray[i]);
      logToSerial("Subscription return status: ");
      logLineToSerial(subscriptionSuccessful);
    }
    loopClient();
  }
  return connectionEstablished;
}

bool MQTTClient::loopClient() {
  if (!pubSubClient->connected()) {
    if (this->reconnect()) {
      pubSubClient->loop();
      return true;
    } else {
      return false;
    }
  }
  pubSubClient->loop();
  return true;
}

void setupWifiConnection(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println(" ...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Waiting for connection...");
    delay(1000);
  }

  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
}

unsigned long last_wifi_reconnect_attempt = 0;

void checkWifiStatus(const char* ssid, const char* password) {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - last_wifi_reconnect_attempt > 20000UL || last_wifi_reconnect_attempt == 0) {
      Serial.println("Attempting to connect to WiFi");
      last_wifi_reconnect_attempt = now;
      setupWifiConnection(ssid, password);
    }
    return;
  }
}

void displayFreeRam() {
  uint32_t free = system_get_free_heap_size();
  Serial.print("Free ram: ");
  Serial.println(free);
}