// micro-ROS firmware entry point for the Adafruit ESP32 Feather V2.
//
// Publishes std_msgs/Float32 on /esp32/angle at 100 Hz. The published
// value is the AS5600 raw shaft angle in degrees (0..360) read by
// i2c_sensor::read_sensor().
//
// Subscribes to std_msgs/UInt8 on /esp32/state; the callback hands the
// value to the onboard NeoPixel driver, which renders a per-state
// pattern (see neopixel.h for the table).
//
// Requires a running micro_ros_agent on the host:
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>

#include "i2c_sensor.h"
#include "neopixel.h"

namespace {

rcl_publisher_t g_publisher;
std_msgs__msg__Float32 g_msg;
rcl_subscription_t g_state_sub;
std_msgs__msg__UInt8 g_state_msg;
rclc_executor_t g_executor;
rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_node_t g_node;
rcl_timer_t g_timer;

#define RCCHECK(fn)                                          \
    do {                                                     \
        rcl_ret_t rc = (fn);                                 \
        if (rc != RCL_RET_OK) {                              \
            error_loop();                                    \
        }                                                    \
    } while (0)

#define RCSOFTCHECK(fn) (void)(fn)

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

void timer_callback(rcl_timer_t* timer, int64_t /*last_call_time*/) {
    if (timer == nullptr) {
        return;
    }
    g_msg.data = i2c_sensor::read_sensor();
    RCSOFTCHECK(rcl_publish(&g_publisher, &g_msg, nullptr));
}

void state_callback(const void* msgin) {
    const auto* msg = static_cast<const std_msgs__msg__UInt8*>(msgin);
    neopixel::set_state(msg->data);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    i2c_sensor::begin();
    neopixel::begin();

    g_allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&g_support, 0, nullptr, &g_allocator));
    RCCHECK(rclc_node_init_default(&g_node, "esp32_node", "", &g_support));

    RCCHECK(rclc_publisher_init_default(
        &g_publisher,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/esp32/angle"));

    RCCHECK(rclc_subscription_init_default(
        &g_state_sub,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/esp32/state"));

    const unsigned int timer_period_ms = 10;  // 100 Hz publish rate
    RCCHECK(rclc_timer_init_default(
        &g_timer,
        &g_support,
        RCL_MS_TO_NS(timer_period_ms),
        timer_callback));

    // Executor needs one slot per timer + one per subscription.
    RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 2, &g_allocator));
    RCCHECK(rclc_executor_add_timer(&g_executor, &g_timer));
    RCCHECK(rclc_executor_add_subscription(
        &g_executor, &g_state_sub, &g_state_msg, &state_callback, ON_NEW_DATA));

    g_msg.data = 0.0f;
    g_state_msg.data = 0;
}

void loop() {
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100));
    neopixel::tick();
}
