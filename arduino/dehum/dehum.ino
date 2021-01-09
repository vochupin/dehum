#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>

#include <Thread.h>             // https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>
#include "SevenSegmentTM1637.h"

#include "DHTesp.h"

#define WIFI_SSID "TP-Link_2E34"
#define WIFI_PASS "63004052"
#define MQTT_SERVER "192.168.0.105"

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, 1883, espClient);
PubSubClientTools mqtt(client);

ThreadController threadControl = ThreadController();
Thread thread = Thread();
Thread relayThread = Thread();
Thread dhtThread = Thread();

const byte PIN_CLK = 22;   // define CLK pin (any digital pin)
const byte PIN_DIO = 23;   // define DIO pin (any digital pin)
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);

boolean relayState = false;

int value = 0;
const String s = "";

DHTesp dht;

String getTemperature();

/** Comfort profile */
ComfortState cf;
/** Pin number for DHT11 data pin */
int dhtPin = 15;

const byte PIN_RELAY = 13; //220V relay control pin

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

  String retval = " T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus;

  display.print(String(newValues.temperature, 1));

  Serial.println(retval);
  return retval;
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  
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

    mqtt.subscribe("test_in/foo/bar",  topic1_subscriber);
//    mqtt.subscribe("test_in/+/bar",    topic2_subscriber);
//    mqtt.subscribe("test_in/#",        topic3_subscriber);
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

  initTemp();
}

void loop() {
  client.loop();
  threadControl.run();
}

void relayControl() {
  if (relayState == true) {
    relayState = false;

    digitalWrite(PIN_RELAY, HIGH);
  } else {
    relayState = true;

    digitalWrite(PIN_RELAY, LOW);    
  }
}

void publisher() {
  ++value;
  mqtt.publish("test_out/hello_world", s+"Hello World! - No. "+value);
}

void dhtPublisher() {
  String dhtReadings = getTemperature();
  mqtt.publish("test_out/dht", dhtReadings);
}

void topic1_subscriber(String topic, String message) {
  Serial.println(s+"Message arrived in function 1 ["+topic+"] "+message);
  display.print(message);
}
void topic2_subscriber(String topic, String message) {
  Serial.println(s+"Message arrived in function 2 ["+topic+"] "+message);
}
void topic3_subscriber(String topic, String message) {
  Serial.println(s+"Message arrived in function 3 ["+topic+"] "+message);
}
