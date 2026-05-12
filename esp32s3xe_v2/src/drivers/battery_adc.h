/**
 * Battery ADC — 3S LiPo via voltage divider
 */

#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#include <Arduino.h>

void  battery_init();
void  battery_update();     // Call periodically (~2Hz), updates state.power
float battery_read_raw();   // Single raw read

#endif
