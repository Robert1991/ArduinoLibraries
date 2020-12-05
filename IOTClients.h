#ifndef IOTClients_h
#define IOTClients_h

#include <ESP8266WiFi.h>
#include <MQTT.h>

#include "SerialLogger.h"

extern "C" {
#include "user_interface.h"
}

#define SUBSCRIPTION_TOPIC_BUFFER_SIZE 20

class MessageQueueClient : public SerialLogger {
 private:
  MQTTClient* mqttClient;
  const char* clientName;
  const char* userName;
  const char* password;
  int subscribeTopicCount = 0;
  int subscriptionTopicBufferSize = 0;
  String subscribeTopicArray[SUBSCRIPTION_TOPIC_BUFFER_SIZE];

  bool reconnectIfNecassary();

 public:
  MessageQueueClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize = 20);
  void setupClient(MQTTClient* mqttClient);
  bool connectToBroker(int connectTimeout = 2500, int reconnectTries = 5);
  int publishMessage(String topic, String payload, bool retain = false);
  void subscribeTopic(String topic);
  void subscribeTopics(String subscribeTopics[], int subscribeTopicCount);
  bool loopClient();
};

class MQTTAutoDiscoverDevice {
 public:
  virtual void configureInTargetPlatform() = 0;
};

class MQTTPublisher : public MQTTAutoDiscoverDevice {
 public:
  virtual void initializePublisher(MessageQueueClient* mqttClient) = 0;
  virtual void publishToTargetPlatform() = 0;
};

class MQTTStateConsumer : public MQTTPublisher {
 public:
  virtual bool consumeMessage(String topic, String payload) = 0;
  virtual void executeLoopMethod() = 0;
  virtual void setupSubscriptions() = 0;
};

class MQTTDeviceService : public SerialLogger {
 private:
  MessageQueueClient* messageQueueClient;
  MQTTPublisher** publishers;
  MQTTStateConsumer** stateConsumers;
  int mqttPublisherBufferSize = 0;
  int mqttPublisherCount = 0;
  int mqttStateConsumerBufferSize = 0;
  int mqttStateConsumerCount = 0;

 public:
  MQTTDeviceService(MessageQueueClient* messageQueueClient, int mqttPublisherBufferSize = 10, int mqttStateConsumerBufferSize = 1);
  void setupMQTTDevices();
  void addPublisher(MQTTPublisher* mqttPublisher);
  void addStateConsumer(MQTTStateConsumer* stateConsumer);
  void executeLoop();
  void handleMessage(String topic, String payload);
};

extern unsigned long last_wifi_reconnect_attempt;
extern unsigned long last_mqtt_reconnect_attempt;

void setupWifiConnection(const char* ssid, const char* password);
void checkWifiStatus(const char* ssid, const char* password);
void displayFreeRam();

#endif