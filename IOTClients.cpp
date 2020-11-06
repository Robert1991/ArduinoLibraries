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

void MQTTClient::reconnect(int connectTimeout) {
  logToSerial("connect...");
  while (!pubSubClient->connected()) {
    logLineToSerial("Reconnecting...");
    if (!pubSubClient->connect(clientName, userName, password)) {
      logToSerial("failed, return state=");
      logToSerial(pubSubClient->state());
      logToSerial(" retrying in ");
      logToSerial(connectTimeout);
      logLineToSerial(" milli seconds");
      delay(connectTimeout);
    }
  }
  for (int i = 0; i < subscribeTopicCount; i++) {
    logToSerial("Subscribing: ");
    logLineToSerial(subscribeTopicArray[i]);
    boolean subscriptionSuccessful = pubSubClient->subscribe(subscribeTopicArray[i]);
    logToSerial("Subscription return status: ");
    logLineToSerial(subscriptionSuccessful);
  }
  loopClient();
}

void MQTTClient::loopClient() {
  if (!pubSubClient->connected()) {
    this->reconnect();
  }
  pubSubClient->loop();
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