#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_types.h"
#include <math.h>
#include <stdio.h>

#define NEOPIXEL_DATA_PIN 0
#define NEOPIXEL_PWR_PIN 2
#define NUM_PIXELS 1

[[nodiscard]]
led_strip_handle_t setup() {
  // 1. Enable power to the NeoPixel (Crucial for the Feather V2!)
  gpio_reset_pin(NEOPIXEL_PWR_PIN);
  gpio_set_direction(NEOPIXEL_PWR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(NEOPIXEL_PWR_PIN, 1);

  // 2. Configure the LED strip
  led_strip_handle_t led_strip;

  led_strip_config_t strip_config = {
      .strip_gpio_num = NEOPIXEL_DATA_PIN,
      .max_leds = NUM_PIXELS,
  };

  // Use the RMT peripheral to generate the precise WS2812 timing
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };

  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  led_strip_clear(led_strip);

  return led_strip;
}

void app_main(void) {
  led_strip_handle_t led_strip = setup();

  // 3. Cycle colors
  float phase = 0;
  float offset = 60 * (M_PI / 180);

  while (1) {
    phase = phase + 0.01;
    if (phase > 2 * M_PI) {
      phase -= 2 * M_PI;
    }

    uint32_t red = (uint32_t)((sinf(phase) + 1) / 2 * 255);
    uint32_t green = (uint32_t)((sinf(phase + offset * 1) + 1) / 2 * 255);
    uint32_t blue = (uint32_t)((sinf(phase + offset) + 2) / 2 * 255);

    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
