/**
 * INA3221 3-Channel Power Monitor
 * Reads bus voltage & current for battery and motor channels.
 */

#ifndef INA3221_POWER_H
#define INA3221_POWER_H

#include <Arduino.h>

bool ina3221_init();
void ina3221_read();  // Reads all 3 channels into busV/currentA arrays

extern float ina_busV[3];
extern float ina_currentA[3];

#endif
