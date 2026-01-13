#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>
#include <ArduinoJson.h>

#ifndef NETWORK_H
#define NETWORK_H

extern int wifiConnectionCounter;
extern PubSubClient client;
extern PubSubClientTools mqtt;
extern StaticJsonDocument<256> doc;

#endif // NETWORK_H
