#include "main.h"
#include "hardware.h"
#include "network.h"

boolean relayState = false;
byte buttonState = 0xff;

float setpoint;
float hysteresis;

int value = 0;
const String s = "";

byte mode;

DHTesp dht;

void writeFloatToEeprom(word eepromAdr, float data) {
  byte* b = (byte*) &data;
  EEPROM.write(eepromAdr, b[0]);
  EEPROM.write(eepromAdr + 1, b[1]);
  EEPROM.write(eepromAdr + 2, b[2]);
  EEPROM.write(eepromAdr + 3, b[3]);
}

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

  if (mode == MODE_AUTO) {
    if (newValues.humidity > setpoint) {
      relayState = true;
    } else if (newValues.humidity < (setpoint - hysteresis)) {
      relayState = false;
    }
  }

  temperature = newValues.temperature;
  humidity = newValues.humidity;
  showMeasurements = true;

  doc.clear();

  doc["temperature"] = newValues.temperature;
  doc["humidity"] = newValues.humidity;
  doc["heatIndex"] = heatIndex;
  doc["dewPoint"] = dewPoint;
  doc["comfortStatus"] = comfortStatus;
  doc["relay"] = relayState;

  String output;
  serializeJson(doc, output);

  Serial.println(output);
  
  return output;
}


