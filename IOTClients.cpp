#include <IOTClients.h>

MessageQueueClient::MessageQueueClient(String clientName, String userName, int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->userName = userName;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
}

MessageQueueClient::MessageQueueClient(String clientName, int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
}

MessageQueueClient::MessageQueueClient(String clientName, String userName, String password,
                                       int subscriptionTopicBufferSize) {
  this->clientName = clientName;
  this->userName = userName;
  this->password = password;
  this->subscriptionTopicBufferSize = subscriptionTopicBufferSize;
}

boolean MessageQueueClient::setupClient(WiFiClient &wifiClient,
                                        MQTTClientCallbackSimple messageReceivedCallback, String broker,
                                        int port, int bufferSize) {
  this->mqttClient = new MQTTClient(bufferSize);
  const char *brokerChars = convertStringToCharArray(broker);
  mqttClient->begin(brokerChars, port, wifiClient);
  mqttClient->onMessage(messageReceivedCallback);
  return connectToBroker();
}

int MessageQueueClient::publishMessage(String topic, String payload, bool retain) {
  if (reconnectIfNecassary()) {
    logToSerial("Publishing: topic: ");
    logToSerial(topic);
    logToSerial(" payload: ");
    logLineToSerial(payload);
    return mqttClient->publish(topic, payload, retain, 1);
  }
  return -1;
}

void MessageQueueClient::subscribeTopic(String topic) {
  if (reconnectIfNecassary()) {
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
  while (!connectMqttToBroker() && (currentTry < reconnectTries)) {
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

bool MessageQueueClient::connectMqttToBroker() {
  const char *clientNameChars = convertStringToCharArray(clientName);

  if (password && userName) {
    const char *passwordChars = convertStringToCharArray(password);
    const char *userNameChars = convertStringToCharArray(userName);
    return mqttClient->connect(clientNameChars, userNameChars, passwordChars);
  } else if (userName) {
    const char *userNameChars = convertStringToCharArray(userName);
    return mqttClient->connect(clientNameChars, userNameChars);
  } else {
    return mqttClient->connect(clientNameChars);
  }
}

char *MessageQueueClient::convertStringToCharArray(String toBeConverted, int bufferSize) {
  char *charArrayBuffer = (char *)malloc(bufferSize * sizeof(char));
  toBeConverted.toCharArray(charArrayBuffer, bufferSize);
  return charArrayBuffer;
}

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