#ifndef IOTClients_h
#define IOTClients_h

#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>

#include "SerialLogger.h"

extern "C" {
#include "user_interface.h"
}

#define SUBSCRIPTION_TOPIC_BUFFER_SIZE 20

class MessageQueueClient : public SerialLogger {
  private:
    MQTTClient *mqttClient;
    String clientName;
    String userName;
    String password;
    int subscribeTopicCount = 0;
    int subscriptionTopicBufferSize = 0;
    String subscribeTopicArray[SUBSCRIPTION_TOPIC_BUFFER_SIZE];

    bool reconnectIfNecassary();
    bool connectMqttToBroker();
    char *convertStringToCharArray(String toBeConverted, int bufferSize = 128);

  public:
    MessageQueueClient(String clientName, String userName, String password,
                       int subscriptionTopicBufferSize = 20);
    MessageQueueClient(String clientName, String userName, int subscriptionTopicBufferSize = 20);
    MessageQueueClient(String clientName, int subscriptionTopicBufferSize = 20);
    bool setupClient(WiFiClient &wifiClient, MQTTClientCallbackSimple messageReceivedCallback, String broker,
                     int port = 1883, int bufferSize = 128);
    bool connectToBroker(int connectTimeout = 500, int reconnectTries = 3);
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
    virtual void initializePublisher(MessageQueueClient *mqttClient) = 0;
    virtual void publishToTargetPlatform() = 0;
    virtual void reset() = 0;
};

class MQTTStateConsumer : public MQTTPublisher {
  public:
    virtual bool consumeMessage(String topic, String payload) = 0;
    virtual void executeLoopMethod() = 0;
    virtual void setupSubscriptions() = 0;
};

class MQTTDeviceService : public SerialLogger {
  private:
    MessageQueueClient *messageQueueClient;
    MQTTPublisher **publishers;
    MQTTStateConsumer **stateConsumers;
    MQTTStateConsumer *resetStateConsumer;
    int mqttPublisherBufferSize = 0;
    int mqttPublisherCount = 0;
    int mqttStateConsumerBufferSize = 0;
    int mqttStateConsumerCount = 0;

  public:
    MQTTDeviceService(MessageQueueClient *messageQueueClient, int mqttPublisherBufferSize = 10,
                      int mqttStateConsumerBufferSize = 1);
    void setResetStateConsumer(MQTTStateConsumer *resetStateConsumer);
    void setupMQTTDevices();
    void addPublisher(MQTTPublisher *mqttPublisher);
    void addStateConsumer(MQTTStateConsumer *stateConsumer);
    void executeLoop();
    void handleMessage(String topic, String payload);
};

extern unsigned long last_wifi_reconnect_attempt;
extern unsigned long last_mqtt_reconnect_attempt;

void setupWifiConnection(const String ssid, const String password, const String hostname = "",
                         WiFiMode wifiMode = WIFI_STA);
DNSServer setupSoftAccessPointWithDnsServer(String ssid, String domainName);
void checkWifiStatus(const String ssid, const String password, const String hostname = "");
void displayFreeRam();
boolean testWifiConnection(const String ssid, const String password, int maxTries = 5);

#endif