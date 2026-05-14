/**
 * Battery ADC — 3S Li-ion Implementation
 * Uses analogReadMilliVolts() for ESP32-S3 calibrated ADC readings.
 * SoC estimation via non-linear voltage lookup table.
 */

#include "battery_adc.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"

// ── 3S Li-ion Discharge Curve (no-load typical) ──────────────
// Voltage → State of Charge (%)
// Source: typical 18650 discharge profile × 3 cells
static const float SOC_TABLE[][2] = {
    {12.60f, 100.0f},
    {12.45f,  95.0f},
    {12.30f,  90.0f},
    {12.15f,  80.0f},
    {12.00f,  70.0f},
    {11.80f,  60.0f},
    {11.60f,  50.0f},
    {11.40f,  40.0f},
    {11.20f,  30.0f},
    {11.00f,  20.0f},
    {10.80f,  15.0f},
    {10.50f,  10.0f},
    {10.20f,   5.0f},
    { 9.90f,   0.0f},
};
static const int SOC_TABLE_LEN = sizeof(SOC_TABLE) / sizeof(SOC_TABLE[0]);

// Interpolate SoC from voltage using lookup table
static float voltage_to_soc(float voltage) {
    if (voltage >= SOC_TABLE[0][0]) return 100.0f;
    if (voltage <= SOC_TABLE[SOC_TABLE_LEN - 1][0]) return 0.0f;
    
    for (int i = 0; i < SOC_TABLE_LEN - 1; i++) {
        float vHigh = SOC_TABLE[i][0], sHigh = SOC_TABLE[i][1];
        float vLow  = SOC_TABLE[i + 1][0], sLow = SOC_TABLE[i + 1][1];
        if (voltage >= vLow && voltage <= vHigh) {
            // Linear interpolation between table entries
            float t = (voltage - vLow) / (vHigh - vLow);
            return sLow + t * (sHigh - sLow);
        }
    }
    return 0.0f;
}

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

    // Non-linear SoC from lookup table (proper Li-ion discharge curve)
    state.power.percent = (int)constrain(voltage_to_soc(state.power.filteredVBatt), 0.0f, 100.0f);
}

