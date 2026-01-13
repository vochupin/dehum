#include <Arduino.h>
#include <DHTesp.h>

void dhtPublisher();

#include <Thread.h>             // https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>
#include "SevenSegmentTM1637.h"

#ifndef MAIN_H
#define MAIN_H

extern boolean showMeasurements;
extern float temperature;
extern float humidity;
extern ThreadController threadControl;
extern Thread dhtThread;
extern ComfortState cf;
extern int dhtPin;
extern boolean showMeasurements;

#endif // MAIN_H
