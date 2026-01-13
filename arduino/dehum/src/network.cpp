#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "hardware.h"

const String CMD_ENABLE = String("enable");
const String CMD_DISABLE = String("disable");
const String CMD_MODE_REMOTE = String("remote");
const String CMD_MODE_AUTO = String("auto");

const char* KEY_FAN = "fan";
const char* KEY_FAN_SETPOINT = "setpoint";
const char* KEY_FAN_HYSTERESIS = "hysteresis";
const char* KEY_FAN_MODE = "mode";

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, 1883, espClient);
PubSubClientTools mqtt(client);

StaticJsonDocument<256> doc;

int wifiConnectionCounter = 1;

void topic_subscriber(String topic, String message) {
  Serial.println(s+"Message arrived in handler ["+topic+"] "+message);

  auto error = deserializeJson(doc, message);
  if (error) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey(KEY_FAN)) {
    String fanStr = doc[KEY_FAN];
  
    if (CMD_ENABLE.equalsIgnoreCase(fanStr)) {
      relayState = true;
    } else if (CMD_DISABLE.equalsIgnoreCase(fanStr)) {
      relayState = false;
    }
  } else if (doc.containsKey(KEY_FAN_SETPOINT)) {
    String setpointStr = doc[KEY_FAN_SETPOINT];
  
    setpoint = setpointStr.toFloat();

    writeFloatToEeprom(EE_SETPOINT, setpoint);
  
    EEPROM.commit();  

    Serial.println("Write setpoint to EEPROM: " + String(setpoint));
  } else if (doc.containsKey(KEY_FAN_HYSTERESIS)) {
    String hysteresisStr = doc[KEY_FAN_HYSTERESIS];
  
    hysteresis = hysteresisStr.toFloat();

    writeFloatToEeprom(EE_HYSTERESIS, hysteresis);
  
    EEPROM.commit();  

    Serial.println("Write hysteresis to EEPROM: " + String(hysteresis));
  } else if (doc.containsKey(KEY_FAN_MODE)) {
    String modeStr = doc[KEY_FAN_MODE];
  
    mode = modeStr.toInt();

    EEPROM.write(EE_MODE, mode);
  
    EEPROM.commit();  

    Serial.println("Write hysteresis to EEPROM: " + String(hysteresis));
  }
}
void connectToMqttIfNecessary() {
  if (!client.connected()) {
    Serial.print(s+"Connecting to MQTT: "+MQTT_SERVER+" ... ");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
  
      mqtt.subscribe("dehum_in/control",  topic_subscriber);
    } else {
      Serial.println(s+"failed, rc="+client.state());
    }
  }
}

void wifiControl() {
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("Wifi connected!");

    connectToMqttIfNecessary();
  } else {
    Serial.println("Wifi disconnected. Try to reconnect!");
    WiFi.reconnect();

    wifiConnectionCounter++;
    if (wifiConnectionCounter > 999) {
      wifiConnectionCounter = 0;
    }
  }
}
