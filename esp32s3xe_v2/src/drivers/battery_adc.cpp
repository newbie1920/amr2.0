/**
 * Battery ADC — 3S LiPo Implementation
 * Uses analogReadMilliVolts() for ESP32-S3 calibrated ADC readings.
 */

#include "battery_adc.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"

void battery_init() {
    analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
    analogSetPinAttenuation(BATT_PIN, ADC_11db);  // Pin-specific attenuation
    pinMode(BATT_PIN, INPUT);
    LOG_I("BATT", "ADC initialized (GPIO%d, 11dB attenuation)", BATT_PIN);
}

float battery_read_raw() {
    // Use analogReadMilliVolts() — ESP32-S3 has eFuse calibration for accuracy
    uint32_t mV = analogReadMilliVolts(BATT_PIN);
    float adcVolts = mV / 1000.0f;
    float battV = adcVolts * BATT_SCALE_FACTOR + BATT_OFFSET;

    // Debug logging (only every ~10s via static counter)
    static int debugCounter = 0;
    if (++debugCounter >= 20) {  // At 2Hz update rate → every 10s
        debugCounter = 0;
        int rawAdc = analogRead(BATT_PIN);
        LOG_D("BATT", "raw=%d mV=%lu adcV=%.3f battV=%.2f filtered=%.2f pct=%d%%",
              rawAdc, mV, adcVolts, battV, state.power.filteredVBatt, state.power.percent);
    }
    return battV;
}

void battery_update() {
    float raw;

    // Prefer INA3221 battery channel (accurate bus voltage measurement)
    if (state.power.inaAvailable && state.power.busV[INA_CH_BATT] > 1.0f) {
        raw = state.power.busV[INA_CH_BATT];
    } else {
        // Fallback to ADC voltage divider
        raw = battery_read_raw();
    }

    // Sanity check — ignore obviously bad readings
    if (raw < 1.0f || raw > 20.0f) return;

    // EMA filter (alpha = 0.05)
    state.power.filteredVBatt = state.power.filteredVBatt * 0.95f + raw * 0.05f;
    state.power.percent = (int)constrain(
        map((long)(state.power.filteredVBatt * 100),
            (long)(BATT_MIN_V * 100), (long)(BATT_MAX_V * 100),
            0, 100),
        0, 100);
}
