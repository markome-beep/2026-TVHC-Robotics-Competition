#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_strip.h"

// Hardware Definitions
#define NEOPIXEL_PIN 0
#define NEOPIXEL_PWR_PIN 2

// UART Definitions (Standard USB Serial on ESP32 is UART0)
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024

static led_strip_handle_t led_strip;

// --- UART Communication Task ---
static void uart_listen_task(void *arg)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Apply configuration and install driver
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Allocate memory for incoming data
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);

    while (1) {
        // Read data from the UART
        // This blocks for up to 100 ticks waiting for data
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            data[len] = '\0'; // Null-terminate to treat as a string

            // Parse the incoming command
            if (data[0] == '1') {
                led_strip_set_pixel(led_strip, 0, 255, 20, 147);
                led_strip_refresh(led_strip);
                
                // Send a response back to the computer
                const char* resp = "LED turned ON\n";
                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
                
            } else if (data[0] == '0') {
                led_strip_clear(led_strip);
                
                const char* resp = "LED turned OFF\n";
                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
            }
        }
    }
}

void app_main(void)
{
    // Power on the NeoPixel
    gpio_reset_pin(NEOPIXEL_PWR_PIN);
    gpio_set_direction(NEOPIXEL_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NEOPIXEL_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure the LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    led_strip_clear(led_strip);

    // Start the UART listener on a separate background task
    xTaskCreate(uart_listen_task, "uart_listen_task", 4096, NULL, 10, NULL);
}
