#ifndef IOTClients_h
#define IOTClients_h

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SUBSCRIPTION_TOPIC_BUFFER_SIZE 20

class MQTTClient {
 private:
  PubSubClient* pubSubClient;
  const char* clientName;
  const char* userName;
  const char* password;
  char** subscribeTopicArray;
  int subscribeTopicCount = 0;
  int subscriptionTopicBufferSize = 0;

  void reconnect(int connectTimeout = 5000);

 public:
  MQTTClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize = 20);

  void setupClient(PubSubClient* pubSubClient, const char* mqttBroker, const int mqttPort = 1883);
  void publishMessage(char* topic, char* payload);
  void subscribeTopic(char* topic);
  void subscribeTopics(char** subscribeTopics, int subscribeTopicCount);
  void loopClient();
};

void setupWifiConnection(const char* ssid, const char* password);

#endif