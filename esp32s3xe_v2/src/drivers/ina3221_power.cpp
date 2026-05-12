/**
 * INA3221 3-Channel Power Monitor — Implementation
 */

#include "ina3221_power.h"
#include "config.h"
#include "log.h"
#include <Wire.h>

extern SemaphoreHandle_t i2cMutex;

#define INA3221_ADDR 0x40

float ina_busV[3]     = {0, 0, 0};
float ina_currentA[3] = {0, 0, 0};

static int inaFailCount = 0;
static bool inaAvailableInternal = false;

static void i2c_recover_ina() {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
}

static int16_t readReg(uint8_t reg, bool* ok = nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        if (ok) *ok = false;
        return 0;
    }
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        i2c_recover_ina();
        xSemaphoreGive(i2cMutex);
        if (ok) *ok = false;
        return 0;
    }
    uint8_t rcv = Wire.requestFrom((uint8_t)INA3221_ADDR, (uint8_t)2);
    if (rcv < 2) {
        i2c_recover_ina();
        xSemaphoreGive(i2cMutex);
        if (ok) *ok = false;
        return 0;
    }
    int16_t res = (Wire.read() << 8) | Wire.read();
    xSemaphoreGive(i2cMutex);
    if (ok) *ok = true;
    return res;
}

bool ina3221_init() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    Wire.beginTransmission(INA3221_ADDR);
    bool found = (Wire.endTransmission() == 0);
    xSemaphoreGive(i2cMutex);

    if (found) {
        LOG_I("INA", "INA3221 OK (0x%02X)", INA3221_ADDR);
        inaAvailableInternal = true;
    } else {
        LOG_W("INA", "INA3221 not found!");
    }
    return found;
}

void ina3221_read() {
    if (!inaAvailableInternal) return;

    bool success = true;
    for (int ch = 1; ch <= 3; ch++) {
        bool ok1 = true, ok2 = true;
        int16_t rawBus   = readReg(2 + (ch - 1) * 2, &ok1);
        int16_t rawShunt = readReg(1 + (ch - 1) * 2, &ok2);

        if (!ok1 || !ok2) {
            success = false;
            continue;
        }

        // Bus voltage: shift right 3 bits, LSB = 8mV
        float voltage = (rawBus >> 3) * 0.008f;
        if (voltage > 1.0f) voltage -= 0.85f;  // Hardware offset correction
        ina_busV[ch - 1] = voltage;

        // Current: shift right 3, LSB=40uV, Rshunt=0.1Ω → I = V/R = raw*40e-6*10/R
        ina_currentA[ch - 1] = (rawShunt >> 3) * 0.0004f;
    }

    if (success) {
        inaFailCount = 0;
    } else {
        inaFailCount++;
        if (inaFailCount > 20) {
            LOG_E("INA", "Too many failures — disabling");
            inaAvailableInternal = false;
        }
    }
}
