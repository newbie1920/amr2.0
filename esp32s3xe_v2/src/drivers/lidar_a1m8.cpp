/**
 * LiDAR Driver — RPLidar A1M8 Implementation
 * Encapsulates init sequence, device detection, and recovery.
 */

#include "lidar_a1m8.h"
#include "config.h"
#include "log.h"

HardwareSerial lidarSerial(1);
RPLidar lidarDevice;

bool lidar_init() {
    // 1. Configure UART pins FIRST
    pinMode(LIDAR_PWM_PIN, OUTPUT);
    lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    delay(50);  // Wait for serial to stabilize

    // 2. Bind RPLidar library (patched: doesn't overwrite serial config)
    lidarDevice.begin(lidarSerial);

    // 3. Stop any ongoing scan from previous boot + flush buffer
    lidarDevice.stop();
    delay(100);
    while (lidarSerial.available()) lidarSerial.read();

    // 4. Turn motor ON — A1M8 needs stable rotation before scan commands
    analogWrite(LIDAR_PWM_PIN, 200);  // ~80% PWM
    LOG_I("LIDAR", "Motor ON (PWM=200). Waiting 2s for stabilization...");
    delay(2000);

    // 5. Check device communication
    rplidar_response_device_info_t info;
    if (IS_OK(lidarDevice.getDeviceInfo(info, 1000))) {
        LOG_I("LIDAR", "Device OK! Model:%d FW:%d.%d HW:%d",
              info.model, info.firmware_version >> 8,
              info.firmware_version & 0xFF, info.hardware_version);

        // 6. Start scanning
        if (IS_OK(lidarDevice.startScan())) {
            LOG_I("LIDAR", "startScan() SUCCESS");
            return true;
        } else {
            LOG_E("LIDAR", "startScan() FAILED");
            return false;
        }
    } else {
        LOG_E("LIDAR", "Device not found! Check wiring:");
        LOG_E("LIDAR", "  RX_PIN=%d (connect to Lidar TX)", LIDAR_RX_PIN);
        LOG_E("LIDAR", "  TX_PIN=%d (connect to Lidar RX)", LIDAR_TX_PIN);

        // Raw UART diagnostic
        LOG_I("LIDAR", "--- Raw UART diagnostic ---");
        uint8_t cmd[] = {0xA5, 0x50};
        lidarSerial.write(cmd, 2);
        delay(200);

        int bytes = lidarSerial.available();
        LOG_I("LIDAR", "Bytes received: %d", bytes);
        if (bytes > 0) {
            Serial.print("[LIDAR] Data (HEX): ");
            while (lidarSerial.available()) Serial.printf("%02X ", lidarSerial.read());
            Serial.println();
            LOG_I("LIDAR", "RX wire OK — check baudrate/noise");
        } else {
            LOG_E("LIDAR", "NO signal — check TX wire of Lidar or power");
        }
        return false;
    }
}

bool lidar_read_point(float& angle, float& distance, uint8_t& quality) {
    if (!IS_OK(lidarDevice.waitPoint())) return false;

    distance = lidarDevice.getCurrentPoint().distance;
    angle    = lidarDevice.getCurrentPoint().angle;
    quality  = lidarDevice.getCurrentPoint().quality;
    return true;
}

void lidar_reset() {
    LOG_W("LIDAR", "Resetting LiDAR...");
    analogWrite(LIDAR_PWM_PIN, 0);

    lidarDevice.stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    while (lidarSerial.available()) lidarSerial.read();
    vTaskDelay(pdMS_TO_TICKS(400));

    rplidar_response_device_info_t info;
    if (IS_OK(lidarDevice.getDeviceInfo(info, 500))) {
        lidarDevice.startScan();
        analogWrite(LIDAR_PWM_PIN, 200);
        vTaskDelay(pdMS_TO_TICKS(3000));  // Wait for motor to stabilize
        LOG_I("LIDAR", "Reset successful — scanning resumed");
    } else {
        // Full UART re-init as last resort
        lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
        analogWrite(LIDAR_PWM_PIN, 200);
        vTaskDelay(pdMS_TO_TICKS(2000));
        LOG_W("LIDAR", "Reset via UART re-init — retry next cycle");
    }
}

void lidar_motor_set(uint8_t pwm) {
    analogWrite(LIDAR_PWM_PIN, pwm);
}
