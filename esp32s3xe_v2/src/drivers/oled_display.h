/**
 * OLED Display — SSD1306 128x64
 * Shows IP, battery, sensor status.
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>

bool oled_init();
void oled_update();  // Called from main loop (~2Hz)

#endif
