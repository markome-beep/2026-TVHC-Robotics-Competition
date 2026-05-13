#pragma once

// Adafruit AS5600 12-bit magnetic rotary position encoder, I2C address
// 0x36. The wrapper hides the chip-specific configuration so the rest
// of the firmware just calls `begin()` once and `read_sensor()` per
// poll. Aggressive-power configuration (always-on internal sampling,
// minimal filtering) is applied in `begin()` so the angle register
// tracks motion as fast as the chip allows.

namespace i2c_sensor {

// Initialize the I2C bus on the Feather V2's default Stemma QT pins
// (SDA=22, SCL=20) and configure the AS5600 for fastest response:
// power mode NOM, hysteresis off, slow filter 2x, fast filter
// threshold 6 LSBs, watchdog disabled. Safe to call once from setup().
// On failure (sensor missing, magnet not detected) prints to Serial
// and returns; subsequent reads will return NaN.
void begin();

// Return the current shaft angle in degrees, range [0.0, 360.0).
// Returns NAN if the sensor was never successfully initialized.
float read_sensor();

}  // namespace i2c_sensor
