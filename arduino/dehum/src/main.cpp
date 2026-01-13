#define MAIN_H

#include "main.h"
#include "settings.h"
#include "network.h"
#include "hardware.h"

ThreadController threadControl = ThreadController();
Thread wifiThread = Thread();
Thread publisherThread = Thread();
Thread indicatorThread = Thread();
Thread relayThread = Thread();
Thread buttonThread = Thread();
Thread dhtThread = Thread();

const byte PIN_CLK = 22;   // define CLK pin (any digital pin)
const byte PIN_DIO = 23;   // define DIO pin (any digital pin)
SevenSegmentTM1637    display(PIN_CLK, PIN_DIO);


String getTemperature();
void topic_subscriber(String topic, String message);
void wifiControl();
void publisher();
void indicatorControl();
void relayControl();
void buttonControl();

/** Comfort profile */
ComfortState cf;
/** Pin number for DHT11 data pin */
int dhtPin = 15;

const byte PIN_RELAY = 13; //220V relay control pin
const byte PIN_BUTTON = 4; //Control button input

boolean showMeasurements = false;

float temperature = 0;
float humidity = 0;

#define IND_TEMPERATURE 0
#define IND_HUMIDITY    1
#define IND_WIFI        2

byte indicatorState;

void initVariables() {
  setpoint = 50.0;
  hysteresis = 5.0;
  mode = MODE_AUTO;

  writeFloatToEeprom(EE_SETPOINT, setpoint);

  writeFloatToEeprom(EE_HYSTERESIS, hysteresis);

  EEPROM.write(EE_MODE, mode);

  EEPROM.commit();  

  Serial.println("Initialize setpoint: " + String(setpoint));
  Serial.println("Initialize hysteresis: " + String(hysteresis));
  Serial.println("Initialize mode: " + String(mode));
}

void readVariablesFromEeprom() {
  byte* b = (byte*) &setpoint;

  byte needInit = 0xff;
  
  b[0] = EEPROM.read(EE_SETPOINT);
  needInit &= b[0];
  b[1] = EEPROM.read(EE_SETPOINT + 1);
  needInit &= b[1];
  b[2] = EEPROM.read(EE_SETPOINT + 2);
  needInit &= b[2];
  b[3] = EEPROM.read(EE_SETPOINT + 3);
  needInit &= b[3];

  if (needInit == 0xff) {
    initVariables();
    return;
  }

  b = (byte*) &hysteresis;
  
  b[0] = EEPROM.read(EE_HYSTERESIS);
  b[1] = EEPROM.read(EE_HYSTERESIS + 1);
  b[2] = EEPROM.read(EE_HYSTERESIS + 2);
  b[3] = EEPROM.read(EE_HYSTERESIS + 3);

  mode = EEPROM.read(EE_MODE);

  Serial.println("Setpoint: " + String(setpoint));
  Serial.println("Hysteresis: " + String(hysteresis));
  Serial.println("Mode: " + String(mode));
}

void initThreads() {
  wifiThread.onRun(wifiControl);
  wifiThread.setInterval(10000);
  threadControl.add(&wifiThread);

  publisherThread.onRun(publisher);
  publisherThread.setInterval(2000);
  threadControl.add(&publisherThread);

  indicatorThread.onRun(indicatorControl);
  indicatorThread.setInterval(3000);
  threadControl.add(&indicatorThread);

  relayThread.onRun(relayControl);
  relayThread.setInterval(1000);
  threadControl.add(&relayThread);

  buttonThread.onRun(buttonControl);
  buttonThread.setInterval(50);
  threadControl.add(&buttonThread);
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  EEPROM.begin(EEPROM_SIZE);
  
  Serial.begin(115200);
  Serial.println();

  readVariablesFromEeprom();

  display.begin();            // initializes the display
  display.setBacklight(100);  // set the brightness to 100 %
  display.print("INIT");      // display INIT on the display

  // Connect to WiFi
  Serial.print(s+"Connecting to WiFi: "+WIFI_SSID+" ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected");

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  delay(500);
  
  display.print("conn");      // display INIT on the display
  
  initTemp();

  initThreads();
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

void indicatorControl() {
  switch(indicatorState) {
    case IND_TEMPERATURE:
      indicatorState = IND_HUMIDITY;
      if (showMeasurements) {
        display.print("t " + String(temperature, 0));
      } else {
        display.print("t --");
      }
      break;
    case IND_HUMIDITY:
      indicatorState = IND_WIFI;
      if (showMeasurements) {
        display.print("H " + String(humidity, 0));
      } else {
        display.print("H --");
      }
      break;
    case IND_WIFI:
      indicatorState = IND_TEMPERATURE;
      
      char cntStr[10] = "____";      
      if (WiFi.status() == WL_CONNECTED) {
        sprintf(cntStr, "C%03d", wifiConnectionCounter);
        display.print(cntStr);
      } else {
        sprintf(cntStr, "c%03d", wifiConnectionCounter);
        display.print(cntStr);
      }
      break;
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
