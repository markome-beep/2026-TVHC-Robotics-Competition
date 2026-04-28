#include "neopixel.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

namespace neopixel {

namespace {

// PIN_NEOPIXEL and NEOPIXEL_I2C_POWER are defined in the
// adafruit_feather_esp32_v2 board variant
// (framework-arduinoespressif32/variants/adafruit_feather_esp32_v2/pins_arduino.h).
// On the Feather V2 the NeoPixel shares the I2C power rail, so we have
// to drive NEOPIXEL_I2C_POWER HIGH before the LED will light.
Adafruit_NeoPixel g_strip(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

volatile uint8_t g_state = 0;

constexpr uint32_t COLOR_OFF    = 0x000000;
constexpr uint32_t COLOR_GREEN  = 0x002000;  // dim — full brightness is blinding
constexpr uint32_t COLOR_YELLOW = 0x202000;
constexpr uint32_t COLOR_RED    = 0x200000;

constexpr unsigned long SLOW_BLINK_MS = 250;  // toggle every 250ms => 2 Hz
constexpr unsigned long FAST_BLINK_MS = 100;  // toggle every 100ms => 5 Hz
constexpr unsigned long RAINBOW_STEP_MS = 20;

void show(uint32_t color) {
    g_strip.setPixelColor(0, color);
    g_strip.show();
}

uint32_t wheel(uint8_t pos) {
    // Standard Adafruit color-wheel helper: 0..255 -> color cycle.
    pos = 255 - pos;
    if (pos < 85) {
        return Adafruit_NeoPixel::Color(255 - pos * 3, 0, pos * 3);
    } else if (pos < 170) {
        pos -= 85;
        return Adafruit_NeoPixel::Color(0, pos * 3, 255 - pos * 3);
    } else {
        pos -= 170;
        return Adafruit_NeoPixel::Color(pos * 3, 255 - pos * 3, 0);
    }
}

}  // namespace

void begin() {
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    g_strip.begin();
    g_strip.setBrightness(64);  // gentle default; cap before patterns scale.
    show(COLOR_OFF);
}

void set_state(uint8_t state) {
    g_state = state;
}

void tick() {
    static uint8_t  last_state = 255;       // force first-tick refresh
    static unsigned long last_toggle_ms = 0;
    static bool blink_on = false;
    static uint8_t rainbow_pos = 0;
    static unsigned long last_rainbow_ms = 0;

    const uint8_t state = g_state;
    const unsigned long now = millis();

    if (state != last_state) {
        last_state = state;
        last_toggle_ms = now;
        last_rainbow_ms = now;
        blink_on = true;
        rainbow_pos = 0;
    }

    switch (state) {
        case 0:
            show(COLOR_OFF);
            break;
        case 1:
            show(COLOR_GREEN);
            break;
        case 2:
            if (now - last_toggle_ms >= SLOW_BLINK_MS) {
                last_toggle_ms = now;
                blink_on = !blink_on;
            }
            show(blink_on ? COLOR_YELLOW : COLOR_OFF);
            break;
        case 3:
            if (now - last_toggle_ms >= FAST_BLINK_MS) {
                last_toggle_ms = now;
                blink_on = !blink_on;
            }
            show(blink_on ? COLOR_RED : COLOR_OFF);
            break;
        case 4:
            if (now - last_rainbow_ms >= RAINBOW_STEP_MS) {
                last_rainbow_ms = now;
                rainbow_pos++;
            }
            show(wheel(rainbow_pos));
            break;
        default:
            show(COLOR_OFF);
            break;
    }
}

}  // namespace neopixel
