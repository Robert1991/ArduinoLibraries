#include "MQTTDevices.h"

MQTTSwitch::MQTTSwitch(MQTTSwitchConfiguration configuration) {
  this->mqttClient = mqttClient;
  this->configuration = configuration;
}

void MQTTSwitch::setupSubscription(MQTTClient* mqttClient) { mqttClient->subscribeTopic(configuration.switchSubscriptionTopic); }

void MQTTSwitch::executeDefaultAction(MQTTClient* mqttClient) {
  switchOn = false;
  mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_OFF);
  applySwitchStatus();
}

void MQTTSwitch::applySwitchStatus() {
  if (switchOn) {
    digitalWrite(configuration.switchPin, HIGH);
  } else {
    digitalWrite(configuration.switchPin, LOW);
  }
}

bool MQTTSwitch::consumeMessage(MQTTClient* mqttClient, String topic, String payload) {
  if (topic.equals(configuration.switchSubscriptionTopic)) {
    if (payload.equals(SWITCH_PAYLOAD_ON)) {
      switchOn = true;
      mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_ON);
    } else {
      switchOn = false;
      mqttClient->publishMessage(configuration.switchStateTopic, SWITCH_PAYLOAD_OFF);
    }
    applySwitchStatus();
    return true;
  }
  return false;
}

MQTTRgbLight::MQTTRgbLight(MQTTRgbLightConfiguration configuration) { this->configuration = configuration; }

void MQTTRgbLight::setupSubscription(MQTTClient* mqttClient) {
  mqttClient->subscribeTopic(configuration.lightSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(configuration.brightnessSwitchSubscriptionTopic);
  mqttClient->subscribeTopic(configuration.colorSetSubscriptionTopic);
}

void MQTTRgbLight::applyChoosenColorToLeds() {
  if (stripOn) {
    analogWrite(configuration.pins.red, (int)(redColorPart * currentBrightness));
    analogWrite(configuration.pins.green, (int)(greenColorPart * currentBrightness));
    analogWrite(configuration.pins.blue, (int)(blueColorPart * currentBrightness));
  } else {
    analogWrite(configuration.pins.red, LOW);
    analogWrite(configuration.pins.green, LOW);
    analogWrite(configuration.pins.blue, LOW);
  }
}

void MQTTRgbLight::executeDefaultAction(MQTTClient* mqttClient) {
  currentBrightness = 1.0;
  redColorPart = 255;
  greenColorPart = 0;
  blueColorPart = 0;
  stripOn = false;
  reportStatus(mqttClient);
}

void MQTTRgbLight::reportStatus(MQTTClient* mqttClient) {
  if (stripOn) {
    mqttClient->publishMessage(configuration.lightStateTopic, "{\"state\":\"ON\"}");
  } else {
    mqttClient->publishMessage(configuration.lightStateTopic, "{\"state\":\"OFF\"}");
  }
  int brightnessValue = (int)(currentBrightness * 255);
  char brightnessValueStringBuffer[16];
  itoa(brightnessValue, brightnessValueStringBuffer, 10);
  mqttClient->publishMessage(configuration.brightnessStateTopic, brightnessValueStringBuffer);

  char concatination[256];
  sprintf(concatination, "{ \"rgb\":[%i,%i,%i]}", redColorPart, greenColorPart, blueColorPart);
  mqttClient->publishMessage(configuration.colorSetStateTopic, concatination);
}

void MQTTRgbLight::processBrightnessPayload(String payload) {
  currentBrightness = (double)(payload.toFloat() / 255.0);
  Serial.print("brightness is now set to: ");
  Serial.println(currentBrightness);
}

void MQTTRgbLight::processColorCommandPayload(String payload) {
  char buffer[32];
  payload.toCharArray(buffer, sizeof(buffer));
  char* p = buffer;
  char* str;
  int iteration = 0;
  while ((str = strtok_r(p, ",", &p)) != NULL) {
    if (iteration == 0) {
      redColorPart = atoi(str);
      Serial.print("red color part was set to: ");
      Serial.println(redColorPart);
    } else if (iteration == 1) {
      greenColorPart = atoi(str);
      Serial.print("green color part was set to: ");
      Serial.println(greenColorPart);
    } else if (iteration == 2) {
      blueColorPart = atoi(str);
      Serial.print("blue color part was set to: ");
      Serial.println(blueColorPart);
      break;
    }
    iteration++;
  }
}

bool MQTTRgbLight::processIncomingMessage(String topic, String payload) {
  if (topic.equals(configuration.lightSwitchSubscriptionTopic)) {
    if (payload.equals("ON")) {
      stripOn = true;
    } else {
      stripOn = false;
    }
    return true;
  } else if (topic.equals(configuration.brightnessSwitchSubscriptionTopic)) {
    processBrightnessPayload(payload);
    return true;
  } else if (topic.equals(configuration.colorSetSubscriptionTopic)) {
    processColorCommandPayload(payload);
    return true;
  }
  return false;
}

bool MQTTRgbLight::consumeMessage(MQTTClient* mqttClient, String topic, String payload) {
  bool stateChanged = processIncomingMessage(topic, payload);
  if (stateChanged) {
    applyChoosenColorToLeds();
    reportStatus(mqttClient);
  }
  return stateChanged;
}