#pragma once

#include <stdint.h>

// Driver for the onboard NeoPixel on the Adafruit ESP32 Feather V2.
//
// State -> pattern mapping (see neopixel.cpp for the table):
//   0 : OFF
//   1 : solid green       ("ok")
//   2 : slow yellow blink (~2 Hz)  ("warn")
//   3 : fast red blink    (~5 Hz)  ("error")
//   4 : rainbow cycle              ("busy" / demo)
//   * : any other value clamps to OFF
namespace neopixel {

// Power up the NeoPixel rail and initialise the strip. Safe to call
// once from setup().
void begin();

// Cheap setter; just stores the new state. Safe to call from a ROS
// subscriber callback (no I/O, no delay).
void set_state(uint8_t state);

// Drive the LED. Call this every loop() iteration; it uses millis() to
// time blinks and the rainbow cycle, so its update rate sets the
// animation smoothness, not the message rate.
void tick();

}  // namespace neopixel
