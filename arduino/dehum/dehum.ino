#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>

#include <Thread.h>             // https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>
#include <ArduinoJson.h>
#include "SevenSegmentTM1637.h"


#include "DHTesp.h"
#include "settings.h"

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, 1883, espClient);
PubSubClientTools mqtt(client);

ThreadController threadControl = ThreadController();
Thread thread = Thread();
Thread relayThread = Thread();
Thread buttonThread = Thread();
Thread dhtThread = Thread();

const byte PIN_CLK = 22;   // define CLK pin (any digital pin)
const byte PIN_DIO = 23;   // define DIO pin (any digital pin)
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);

boolean relayState = false;
byte buttonState = 0xff;

int value = 0;
const String s = "";

DHTesp dht;

String getTemperature();

/** Comfort profile */
ComfortState cf;
/** Pin number for DHT11 data pin */
int dhtPin = 15;

const byte PIN_RELAY = 13; //220V relay control pin
const byte PIN_BUTTON = 4; //Control button input

StaticJsonDocument<256> doc;

const String CMD_ENABLE = String("enable");
const String CMD_DISABLE = String("disable");

const char* KEY_FAN = "fan";

/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  // Enable Thread
  dhtThread.onRun(dhtPublisher);
  dhtThread.setInterval(20000);
  threadControl.add(&dhtThread);
  
  return true;
}

/**
 * getTemperature
 * Reads temperature from DHT11 sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
String getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    String dhtError = "DHT11 error status: " + String(dht.getStatusString());
    Serial.println(dhtError);
    return dhtError;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch(cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  doc.clear();

  doc["temperature"] = newValues.temperature;
  doc["humidity"] = newValues.humidity;
  doc["heatIndex"] = heatIndex;
  doc["dewPoint"] = dewPoint;
  doc["comfortStatus"] = comfortStatus;

  String output;
  serializeJson(doc, output);

  display.print(String(newValues.temperature, 1));

  Serial.println(output);
  return output;
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  
  Serial.begin(115200);
  Serial.println();

  // Connect to WiFi
  Serial.print(s+"Connecting to WiFi: "+WIFI_SSID+" ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected");

  // Connect to MQTT
  Serial.print(s+"Connecting to MQTT: "+MQTT_SERVER+" ... ");
  if (client.connect("ESP32Client")) {
    Serial.println("connected");

    mqtt.subscribe("dehum_in/control",  topic_subscriber);
  } else {
    Serial.println(s+"failed, rc="+client.state());
  }

  display.begin();            // initializes the display
  display.setBacklight(100);  // set the brightness to 100 %
  display.print("INIT");      // display INIT on the display
  delay(1000);                // wait 1000 ms
  
  // Enable Threads
  thread.onRun(publisher);
  thread.setInterval(2000);
  threadControl.add(&thread);

  relayThread.onRun(relayControl);
  relayThread.setInterval(1000);
  threadControl.add(&relayThread);

  buttonThread.onRun(buttonControl);
  buttonThread.setInterval(50);
  threadControl.add(&buttonThread);

  initTemp();
}

void loop() {
  client.loop();
  threadControl.run();
}

void buttonControl() {
  buttonState <<= 1;
  
  if (digitalRead(PIN_BUTTON)) {
    buttonState |= 1;
  }

  if ((buttonState & 0x0f) == 0x0c) {
    relayState ^= 1;
  }
}

void relayControl() {
  if (relayState == true) {
    digitalWrite(PIN_RELAY, HIGH);
  } else {
    digitalWrite(PIN_RELAY, LOW);    
  }
}

void publisher() {
  ++value;
  mqtt.publish("dehum_out/heartbit", s + "{\"counter\":\"" + value + "\"}");
}

void dhtPublisher() {
  String dhtReadings = getTemperature();
  mqtt.publish("dehum_out/measures", dhtReadings);
}

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
  }
}
