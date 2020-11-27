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
  char** subscribeTopicArray;
  int subscribeTopicCount = 0;
  int subscriptionTopicBufferSize = 0;

  bool reconnect(int connectTimeout = 5000, int reconnectTries = 5);

 public:
  MessageQueueClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize = 20);

  void setupClient(MQTTClient* mqttClient);
  int publishMessage(String topic, String payload, bool retain = false);
  void subscribeTopic(char* topic);
  void subscribeTopics(char** subscribeTopics, int subscribeTopicCount);
  bool loopClient();
};

extern unsigned long last_wifi_reconnect_attempt;
extern unsigned long last_mqtt_reconnect_attempt;

void setupWifiConnection(const char* ssid, const char* password);
void checkWifiStatus(const char* ssid, const char* password);
void displayFreeRam();

#endif