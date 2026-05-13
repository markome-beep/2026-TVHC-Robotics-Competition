#include "i2c_sensor.h"

#include <math.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>

namespace i2c_sensor {

namespace {

// Adafruit ESP32 Feather V2 default I2C pins (Stemma QT port).
constexpr int kSdaPin = 22;
constexpr int kSclPin = 20;

// 12-bit raw angle -> degrees scale. 4096 codes span [0, 360).
constexpr float kRawToDegrees = 360.0f / 4096.0f;

Adafruit_AS5600 g_sensor;
bool g_ready = false;

}  // namespace

void begin() {
    Wire.begin(kSdaPin, kSclPin);

    if (!g_sensor.begin(AS5600_DEFAULT_ADDR, &Wire)) {
        Serial.println("[as5600] not found at 0x36; angle will be NaN");
        g_ready = false;
        return;
    }

    if (!g_sensor.isMagnetDetected()) {
        Serial.println("[as5600] no magnet detected; check alignment");
        // Continue anyway — the chip will still answer, the readings
        // are just untrustworthy until the magnet is present.
    }

    // Aggressive-power / fastest-polling configuration.
    //
    //  * NOM: always-on internal sampling (~150 us period). The LPMx
    //    modes gate the front-end off between samples (5/20/100 ms).
    //  * Hysteresis OFF: smallest output dead-band; tiny angle changes
    //    propagate to the ANGLE register immediately.
    //  * Slow filter 2x: lightest steady-state filtering, so the
    //    register tracks motion as fast as the chip allows.
    //  * Fast filter threshold 6 LSBs: the smallest non-zero step
    //    bypasses the slow filter entirely and switches to the fast
    //    (no-filter) path.
    //  * Watchdog disabled: prevents the chip from auto-falling into
    //    LPM3 after one minute of stability.
    g_sensor.setPowerMode(AS5600_POWER_MODE_NOM);
    g_sensor.setHysteresis(AS5600_HYSTERESIS_OFF);
    g_sensor.setSlowFilter(AS5600_SLOW_FILTER_2X);
    g_sensor.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_6LSB);
    g_sensor.enableWatchdog(false);

    g_ready = true;
}

float read_sensor() {
    if (!g_ready) {
        return NAN;
    }
    const uint16_t raw = g_sensor.getRawAngle() & 0x0FFF;  // 12-bit mask
    return static_cast<float>(raw) * kRawToDegrees;
}

}  // namespace i2c_sensor
