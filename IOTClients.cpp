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

void MQTTClient::publishMessage(char* topic, char* payload) {
  if (!pubSubClient->connected()) {
    this->reconnect();
  }
  Serial.println("publishing");
  pubSubClient->publish(topic, payload);
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
  Serial.print("connect...");
  while (!pubSubClient->connected()) {
    Serial.println("Reconnecting...");
    if (!pubSubClient->connect(clientName, userName, password)) {
      Serial.print("failed, return state=");
      Serial.print(pubSubClient->state());
      Serial.print(" retrying in ");
      Serial.print(connectTimeout);
      Serial.println(" milli seconds");
      delay(connectTimeout);
    }
  }
  for (int i = 0; i < subscribeTopicCount; i++) {
    Serial.print("Subscribing: ");
    Serial.println(subscribeTopicArray[i]);
    boolean subscriptionSuccessful = pubSubClient->subscribe(subscribeTopicArray[i]);
    Serial.print("Subscription return status: ");
    Serial.println(subscriptionSuccessful);
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