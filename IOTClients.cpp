#include <IOTClients.h>

MessageQueueClient::MessageQueueClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->userName = userName;
  this->password = password;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
  subscribeTopicArray = new char*[subscriptionTopicBufferSize];
}

void MessageQueueClient::setupClient(MQTTClient* mqttClient) {
  this->mqttClient = mqttClient;
  this->reconnect();
}

int MessageQueueClient::publishMessage(String topic, String payload, bool retain) {
  if (!mqttClient->connected()) {
    this->reconnect();
    logLineToSerial("Message queue client is connected!");
  }

  return mqttClient->publish(topic, payload, retain, 1);
}

void MessageQueueClient::subscribeTopic(char* topic) {
  if (!mqttClient->connected()) {
    this->reconnect();
    logLineToSerial("Message queue client is connected!");
  }
  if (subscribeTopicCount < subscriptionTopicBufferSize) {
    subscribeTopicArray[subscribeTopicCount] = topic;
    subscribeTopicCount++;
  }
}

void MessageQueueClient::subscribeTopics(char** subscribeTopics, int countOfTopics) {
  for (int i = 0; i < countOfTopics; i++) {
    subscribeTopicArray[subscribeTopicCount] = subscribeTopics[i];
    subscribeTopicCount++;
  }
}

bool MessageQueueClient::reconnect(int connectTimeout, int reconnectTries) {
  logToSerial("connecting mqtt client...");

  int currentTry = 0;
  bool connectionEstablished = false;
  while (!mqttClient->connect(clientName, userName, password) && (currentTry < reconnectTries)) {
    logLineToSerial("Connection attemp failed. Reconnecting...");
    currentTry++;
  }
  if (currentTry >= reconnectTries) {
    return false;
  }
  if (connectionEstablished) {
    for (int i = 0; i < subscribeTopicCount; i++) {
      logToSerial("Subscribing: ");
      logLineToSerial(subscribeTopicArray[i]);
      boolean subscriptionSuccessful = mqttClient->subscribe(subscribeTopicArray[i]);
      logToSerial("Subscription return status: ");
      logLineToSerial(subscriptionSuccessful);
    }
    loopClient();
  }
  return connectionEstablished;
}

bool MessageQueueClient::loopClient() {
  if (!mqttClient->connected()) {
    if (this->reconnect()) {
      return mqttClient->loop();
    }
    return false;
  }
  return mqttClient->loop();
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