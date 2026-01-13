#include <Arduino.h>
#include <EEPROM.h>
#include <DHTesp.h>

#define EEPROM_SIZE 9 

void writeFloatToEeprom(word eepromAdr, float data);
bool initTemp();

//EEPROM Map
#define EE_SETPOINT 0
#define EE_HYSTERESIS 4
#define EE_MODE 8

#define MODE_AUTO 0
#define MODE_REMOTE 1

#ifndef HARDWARE_H
#define HARDWARE_H

extern boolean relayState;
extern byte buttonState;
extern float setpoint;
extern float hysteresis;
extern int value;
extern const String s;
extern byte mode;

#endif // HARDWARE_H
