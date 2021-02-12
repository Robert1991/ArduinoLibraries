#include <IOTClients.h>

MessageQueueClient::MessageQueueClient(const char* clientName, const char* userName, const char* password, int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->userName = userName;
  this->password = password;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
}

void MessageQueueClient::setupClient(MQTTClient* mqttClient) {
  this->mqttClient = mqttClient;
  this->connectToBroker();
}

int MessageQueueClient::publishMessage(String topic, String payload, bool retain) {
  reconnectIfNecassary();
  logToSerial("Publishing: topic: ");
  logToSerial(topic);
  logToSerial(" payload: ");
  logLineToSerial(payload);
  return mqttClient->publish(topic, payload, retain, 1);
}

void MessageQueueClient::subscribeTopic(String topic) {
  reconnectIfNecassary();
  logToSerial("Subscribing ");
  logLineToSerial(topic);
  if (subscribeTopicCount < subscriptionTopicBufferSize) {
    subscribeTopicArray[subscribeTopicCount] = topic;
    subscribeTopicCount++;
  }
  bool subscriptionSuccessful = mqttClient->subscribe(topic);
  logToSerial("Subscription return status: ");
  logLineToSerial(subscriptionSuccessful);
}

void MessageQueueClient::subscribeTopics(String subscribeTopics[], int countOfTopics) {
  for (int i = 0; i < countOfTopics; i++) {
    subscribeTopicArray[subscribeTopicCount] = subscribeTopics[i];
    subscribeTopicCount++;
  }
}

bool MessageQueueClient::loopClient() {
  if (reconnectIfNecassary()) {
    return mqttClient->loop();
  }
  return false;
}

bool MessageQueueClient::reconnectIfNecassary() {
  if (!mqttClient->connected()) {
    return this->connectToBroker();
  }
  return true;
}

bool MessageQueueClient::connectToBroker(int connectTimeout, int reconnectTries) {
  logLineToSerial("connecting mqtt client...");
  int currentTry = 0;
  while (!mqttClient->connect(clientName, userName, password) && (currentTry < reconnectTries)) {
    logLineToSerial("Connection attemp failed. Reconnecting...");
    currentTry++;
    delay(connectTimeout);
  }
  if (currentTry >= reconnectTries) {
    return false;
  } else {
    logToSerial("Subscribing again to topic count: ");
    logLineToSerial(subscribeTopicCount);

    for (int i = 0; i < subscribeTopicCount; i++) {
      logToSerial("Subscribing: ");
      logLineToSerial(subscribeTopicArray[i]);
      boolean subscriptionSuccessful = mqttClient->subscribe(subscribeTopicArray[i]);
      logToSerial("Subscription return status: ");
      logLineToSerial(subscriptionSuccessful);
    }
    loopClient();
    return true;
  }
}

MQTTDeviceService::MQTTDeviceService(MessageQueueClient* messageQueueClient, int mqttPublisherBufferSize, int mqttStateConsumerBufferSize) {
  this->messageQueueClient = messageQueueClient;
  this->mqttPublisherBufferSize = mqttPublisherBufferSize;
  this->mqttStateConsumerBufferSize = mqttStateConsumerBufferSize;
  this->publishers = new MQTTPublisher*[mqttPublisherBufferSize];
  this->stateConsumers = new MQTTStateConsumer*[mqttStateConsumerBufferSize];
}

void MQTTDeviceService::setResetStateConsumer(MQTTStateConsumer* resetStateConsumer) { this->resetStateConsumer = resetStateConsumer; }

MQTTDeviceService::MQTTDeviceService(MessageQueueClient *messageQueueClient, int mqttPublisherBufferSize,
                                     int mqttStateConsumerBufferSize) {
    this->messageQueueClient = messageQueueClient;
    this->mqttPublisherBufferSize = mqttPublisherBufferSize;
    this->mqttStateConsumerBufferSize = mqttStateConsumerBufferSize;
    this->publishers = new MQTTPublisher *[mqttPublisherBufferSize];
    this->stateConsumers = new MQTTStateConsumer *[mqttStateConsumerBufferSize];
}

void MQTTDeviceService::setResetStateConsumer(MQTTStateConsumer *resetStateConsumer) {
    this->resetStateConsumer = resetStateConsumer;
}

void MQTTDeviceService::addPublisher(MQTTPublisher *mqttPublisher) {
    if (mqttPublisherCount < mqttPublisherBufferSize) {
        publishers[mqttPublisherCount] = mqttPublisher;
        mqttPublisherCount++;
    }
}

void MQTTDeviceService::addStateConsumer(MQTTStateConsumer *stateConsumer) {
    addPublisher(stateConsumer);
    if (mqttStateConsumerCount < mqttStateConsumerBufferSize) {
        stateConsumers[mqttStateConsumerCount] = stateConsumer;
        mqttStateConsumerCount++;
    }
}

void MQTTDeviceService::setupMQTTDevices() {
    for (int publisherIndex = 0; publisherIndex < mqttPublisherCount; publisherIndex++) {
        publishers[publisherIndex]->initializePublisher(messageQueueClient);
        publishers[publisherIndex]->configureInTargetPlatform();
        delay(500);
    }
    for (int consumerIndex = 0; consumerIndex < mqttStateConsumerCount; consumerIndex++) {
        stateConsumers[consumerIndex]->setupSubscriptions();
        delay(100);
    }
    if (resetStateConsumer) {
        resetStateConsumer->initializePublisher(messageQueueClient);
        resetStateConsumer->configureInTargetPlatform();
        resetStateConsumer->setupSubscriptions();
    }
}

void MQTTDeviceService::executeLoop() {
    if (messageQueueClient->loopClient()) {
        for (int publisherIndex = 0; publisherIndex < mqttPublisherCount; publisherIndex++) {
            publishers[publisherIndex]->publishToTargetPlatform();
        }
        for (int consumerIndex = 0; consumerIndex < mqttStateConsumerCount; consumerIndex++) {
            stateConsumers[consumerIndex]->executeLoopMethod();
        }
    }
}

void MQTTDeviceService::handleMessage(String topic, String payload) {
    for (int consumerIndex = 0; consumerIndex < mqttStateConsumerCount; consumerIndex++) {
        if (stateConsumers[consumerIndex]->consumeMessage(topic, payload)) {
            break;
        }
    }
    if (resetStateConsumer) {
        if (resetStateConsumer->consumeMessage(topic, payload)) {
            logLineToSerial("Resetting device sensor/actor states");
            for (int publisherIndex = 0; publisherIndex < mqttPublisherCount; publisherIndex++) {
                publishers[publisherIndex]->reset();
            }
            resetStateConsumer->reset();
        }
    }
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