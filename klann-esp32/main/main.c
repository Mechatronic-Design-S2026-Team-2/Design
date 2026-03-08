/**
 * @file main.c
 * @brief ESP32 application entry and micro-ROS bridge logic.
 *
 * Orchestrates the local sensor tasks, LCD status task, and the
 * micro-ROS session running over the ESP32 USB-UART bridge.
 * Publishes IMU and force-sensor telemetry, subscribes to cmd_vel,
 * and keeps a compact runtime status cache for LCD output.
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <klann_msgs/msg/force_sensor_voltages.h>

#include "esp32_serial_transport.h"
#include "force_sensor_adc.h"
#include "mpu9250.h"
#include "character_lcd.h"

/* ---------- micro-ROS config ---------- */

#define MICRO_ROS_PUBLISH_PERIOD_MILLISECONDS                 50
#define MICRO_ROS_EXECUTOR_TIMEOUT_MILLISECONDS               10

#define MICRO_ROS_AGENT_PING_TIMEOUT_MILLISECONDS            100
#define MICRO_ROS_AGENT_PING_ATTEMPTS                         5
#define MICRO_ROS_RETRY_DELAY_MILLISECONDS                 1000
#define MICRO_ROS_BOOT_SETTLE_DELAY_MILLISECONDS           2000
#define MICRO_ROS_LINK_CHECK_PERIOD_MILLISECONDS           1000

#define MICRO_ROS_NODE_NAME                                   "esp32_sensor_bridge"
#define MICRO_ROS_IMU_TOPIC_NAME                              "imu/data_raw"
#define MICRO_ROS_FORCE_SENSOR_TOPIC_NAME                     "force_sensor_voltage_millivolts"
#define MICRO_ROS_VELOCITY_COMMAND_TOPIC_NAME                 "cmd_vel"

#define MICRO_ROS_IMU_FRAME_ID                                "imu_link"

#define SYSTEM_STATUS_LCD_UPDATE_PERIOD_MILLISECONDS          250
#define SYSTEM_STATUS_LCD_FORCE_SENSOR_ROTATION_PERIOD_MILLISECONDS 1000

/* ---------- micro-ROS state ---------- */

static const char *micro_ros_log_tag = "micro_ros_bridge";

static rcl_publisher_t imu_publisher;
static rcl_publisher_t force_sensor_publisher;
static rcl_subscription_t velocity_command_subscriber;

static sensor_msgs__msg__Imu imu_message;
static klann_msgs__msg__ForceSensorVoltages force_sensor_voltage_message;
static geometry_msgs__msg__Twist velocity_command_message;

/*
 * On classic ESP32 dev boards with a USB-to-UART bridge chip,
 * the USB connector is connected to UART0 (GPIO1 / GPIO3).
 * So use UART0 for onboard USB serial transport.
 */
static uart_port_t micro_ros_uart_port = UART_NUM_0;

/* ---------- Shared control / status cache ---------- */

typedef struct
{
    float desired_linear_x_meters_per_second;
    float desired_angular_z_radians_per_second;

    float left_side_placeholder_command;
    float right_side_placeholder_command;

    bool velocity_command_received;

    uint32_t publish_cycle_count;
    bool imu_publish_successful;
    bool force_sensor_publish_successful;

    bool agent_reachable;
    bool entities_initialized;
    bool session_running;

    int32_t last_rcl_error_code;
    int32_t last_error_line;
} micro_ros_runtime_state_t;

static micro_ros_runtime_state_t micro_ros_runtime_state = {0};

/**
 * @brief Local uptime -> ROS stamp helper.
 *
 * Convert ESP uptime microseconds into builtin stamp fields.
 *
 * @param seconds
 *     Output seconds pointer.
 *     Receives whole seconds.
 *     Used for ROS header stamp.
 * @param nanoseconds
 *     Output nanoseconds pointer.
 *     Receives fractional nanoseconds.
 *     Used for ROS header stamp.
 *
 * @return None.
 */
static void get_micro_ros_uptime_stamp(int32_t *seconds, uint32_t *nanoseconds)
{
    int64_t uptime_microseconds = esp_timer_get_time();

    if (uptime_microseconds < 0) {
        uptime_microseconds = 0;
    }

    if (seconds != NULL) {
        *seconds = (int32_t)(uptime_microseconds / 1000000LL);
    }

    if (nanoseconds != NULL) {
        *nanoseconds = (uint32_t)((uptime_microseconds % 1000000LL) * 1000LL);
    }
}

/**
 * @brief Local error-state cache update.
 *
 * Store latest micro-ROS init / runtime failure.
 *
 * @param line_number
 *     Source-code line number.
 *     Identifies failure site.
 *     Used for LCD/debug state.
 * @param rcl_error_code
 *     rcl return code.
 *     Latest failure status.
 *     Used for LCD/debug state.
 *
 * @return None.
 */
static void record_micro_ros_error(int32_t line_number, int32_t rcl_error_code)
{
    micro_ros_runtime_state.last_error_line = line_number;
    micro_ros_runtime_state.last_rcl_error_code = rcl_error_code;
}

/**
 * @brief Side-command placeholder update.
 *
 * Map body command into left / right placeholder commands.
 * Placeholder only. No CAN output yet.
 *
 * @param None.
 *
 * @return None.
 */
static void update_placeholder_side_commands_from_velocity_command(void)
{
    micro_ros_runtime_state.left_side_placeholder_command =
        micro_ros_runtime_state.desired_linear_x_meters_per_second -
        micro_ros_runtime_state.desired_angular_z_radians_per_second;

    micro_ros_runtime_state.right_side_placeholder_command =
        micro_ros_runtime_state.desired_linear_x_meters_per_second +
        micro_ros_runtime_state.desired_angular_z_radians_per_second;
}

/**
 * @brief IMU message static-field init.
 *
 * Initialize frame ID and covariance fields once.
 *
 * @param None.
 *
 * @return None.
 */
static void initialize_imu_message_defaults(void)
{
    sensor_msgs__msg__Imu__init(&imu_message);

    rosidl_runtime_c__String__assign(
        &imu_message.header.frame_id,
        MICRO_ROS_IMU_FRAME_ID);

    /* No orientation estimate yet. */
    imu_message.orientation.x = 0.0;
    imu_message.orientation.y = 0.0;
    imu_message.orientation.z = 0.0;
    imu_message.orientation.w = 1.0;

    for (int covariance_index = 0; covariance_index < 9; covariance_index++) {
        imu_message.orientation_covariance[covariance_index] = 0.0;
        imu_message.angular_velocity_covariance[covariance_index] = 0.0;
        imu_message.linear_acceleration_covariance[covariance_index] = 0.0;
    }

    /* Orientation absent. */
    imu_message.orientation_covariance[0] = -1.0;

    /* Placeholder diagonals. Tune later. */
    imu_message.angular_velocity_covariance[0] = 0.02;
    imu_message.angular_velocity_covariance[4] = 0.02;
    imu_message.angular_velocity_covariance[8] = 0.02;

    imu_message.linear_acceleration_covariance[0] = 0.10;
    imu_message.linear_acceleration_covariance[4] = 0.10;
    imu_message.linear_acceleration_covariance[8] = 0.10;
}

/**
 * @brief Force-sensor message static-field init.
 *
 * Initialize fixed-size voltage array once.
 *
 * @param None.
 *
 * @return None.
 */
static void initialize_force_sensor_voltage_message_defaults(void)
{
    klann_msgs__msg__ForceSensorVoltages__init(&force_sensor_voltage_message);

    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        force_sensor_voltage_message.voltage_millivolts[sensor_index] = 0;
    }
}

/**
 * @brief Velocity-command message init.
 *
 * Initialize subscriber-side Twist storage once.
 *
 * @param None.
 *
 * @return None.
 */
static void initialize_velocity_command_message_defaults(void)
{
    geometry_msgs__msg__Twist__init(&velocity_command_message);

    velocity_command_message.linear.x = 0.0;
    velocity_command_message.linear.y = 0.0;
    velocity_command_message.linear.z = 0.0;

    velocity_command_message.angular.x = 0.0;
    velocity_command_message.angular.y = 0.0;
    velocity_command_message.angular.z = 0.0;
}

/**
 * @brief IMU cache -> ROS message fill.
 *
 * Copy latest MPU / compatible IMU sample into sensor_msgs/Imu.
 *
 * @param None.
 *
 * @return None.
 */
static void update_imu_message_from_latest_measurement(void)
{
    mpu9250_measurement_t latest_mpu9250_measurement = get_latest_mpu9250_measurement();

    get_micro_ros_uptime_stamp(
        &imu_message.header.stamp.sec,
        &imu_message.header.stamp.nanosec);

    imu_message.angular_velocity.x = latest_mpu9250_measurement.gyroscope_x_radians_per_second;
    imu_message.angular_velocity.y = latest_mpu9250_measurement.gyroscope_y_radians_per_second;
    imu_message.angular_velocity.z = latest_mpu9250_measurement.gyroscope_z_radians_per_second;

    imu_message.linear_acceleration.x =
        latest_mpu9250_measurement.accelerometer_x_meters_per_second_squared;
    imu_message.linear_acceleration.y =
        latest_mpu9250_measurement.accelerometer_y_meters_per_second_squared;
    imu_message.linear_acceleration.z =
        latest_mpu9250_measurement.accelerometer_z_meters_per_second_squared;
}

/**
 * @brief FSR cache -> ROS message fill.
 *
 * Copy latest FSR voltages into fixed-size payload.
 *
 * @param None.
 *
 * @return None.
 */
static void update_force_sensor_voltage_message_from_latest_measurements(void)
{
    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        force_sensor_voltage_message.voltage_millivolts[sensor_index] =
            (int32_t)get_force_sensor_voltage_millivolts(sensor_index);
    }
}

/**
 * @brief Velocity-command subscriber callback.
 *
 * Cache desired forward speed and yaw rate locally.
 * CAN / motor actuation intentionally omitted for now.
 *
 * @param received_message
 *     Incoming ROS message pointer.
 *     Expected as geometry_msgs__msg__Twist.
 *     Used for local command cache update.
 *
 * @return None.
 */
static void velocity_command_callback(const void *received_message)
{
    const geometry_msgs__msg__Twist *received_velocity_command =
        (const geometry_msgs__msg__Twist *)received_message;

    if (received_velocity_command == NULL) {
        return;
    }

    micro_ros_runtime_state.desired_linear_x_meters_per_second =
        (float)received_velocity_command->linear.x;
    micro_ros_runtime_state.desired_angular_z_radians_per_second =
        (float)received_velocity_command->angular.z;
    micro_ros_runtime_state.velocity_command_received = true;

    update_placeholder_side_commands_from_velocity_command();
}

/**
 * @brief Periodic sensor-publish callback.
 *
 * Refresh message payloads from local caches and publish.
 *
 * @param timer
 *     rcl timer handle.
 *     Required by rclc timer API.
 *     Used as callback-validity check.
 * @param last_call_time
 *     Previous callback time.
 *     Required by rclc timer API.
 *     Unused here.
 *
 * @return None.
 */
static void sensor_publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    update_imu_message_from_latest_measurement();
    update_force_sensor_voltage_message_from_latest_measurements();

    rcl_ret_t imu_publish_result = rcl_publish(&imu_publisher, &imu_message, NULL);
    rcl_ret_t force_sensor_publish_result =
        rcl_publish(&force_sensor_publisher, &force_sensor_voltage_message, NULL);

    micro_ros_runtime_state.imu_publish_successful =
        (imu_publish_result == RCL_RET_OK);
    micro_ros_runtime_state.force_sensor_publish_successful =
        (force_sensor_publish_result == RCL_RET_OK);
    micro_ros_runtime_state.publish_cycle_count++;

    if (imu_publish_result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)imu_publish_result);
    }

    if (force_sensor_publish_result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)force_sensor_publish_result);
    }
}

/**
 * @brief Two-line LCD status formatter.
 *
 * Render compact system status into two 16-char lines.
 * Rotate displayed force sensor once per second.
 *
 * Output format:
 * - Line 1: "u<agent> e<entities> p<count>"
 *   - u<agent>
 *       - 1 -> micro-ROS agent reachable
 *       - 0 -> micro-ROS agent not reachable
 *   - e<entities>
 *       - 1 -> node / pubs / sub / timer / executor initialized
 *       - 0 -> entities not initialized
 *   - p<count>
 *       - publish cycle count modulo 10000
 *       - zero-padded to 4 digits
 *
 * - Line 2: "F<index> <voltage>mV rc<code>"
 *   - F<index>
 *       - currently displayed force sensor index
 *       - rotates through 0 .. NUM_FORCE_SENSORS - 1
 *       - advances once per second
 *   - <voltage>mV
 *       - latest cached force sensor voltage in millivolts
 *       - taken from get_force_sensor_voltage_millivolts(index)
 *   - rc<code>
 *       - last cached rcl / rmw error code modulo 100
 *       - debug indicator for latest micro-ROS failure state
 *
 * @param first_line_buffer
 *     First-line output buffer.
 *     Must hold at least 17 bytes.
 *     Receives null-terminated row text.
 * @param second_line_buffer
 *     Second-line output buffer.
 *     Must hold at least 17 bytes.
 *     Receives null-terminated row text.
 *
 * @return None.
 */
static void format_system_status_lcd_lines(char *first_line_buffer, char *second_line_buffer)
{
    static int displayed_force_sensor_index = 0;
    static uint32_t elapsed_rotation_time_milliseconds = 0;

    int displayed_force_sensor_voltage_millivolts =
        get_force_sensor_voltage_millivolts(displayed_force_sensor_index);

    snprintf(
        first_line_buffer,
        17,
        "u%d e%d p%04lu",
        micro_ros_runtime_state.agent_reachable ? 1 : 0,
        micro_ros_runtime_state.entities_initialized ? 1 : 0,
        (unsigned long)(micro_ros_runtime_state.publish_cycle_count % 10000UL));

    snprintf(
        second_line_buffer,
        17,
        "F%d %4dmV rc%2ld",
        displayed_force_sensor_index,
        displayed_force_sensor_voltage_millivolts,
        (long)(micro_ros_runtime_state.last_rcl_error_code % 100L));

    elapsed_rotation_time_milliseconds +=
        SYSTEM_STATUS_LCD_UPDATE_PERIOD_MILLISECONDS;

    if (elapsed_rotation_time_milliseconds >=
        SYSTEM_STATUS_LCD_FORCE_SENSOR_ROTATION_PERIOD_MILLISECONDS) {
        elapsed_rotation_time_milliseconds = 0;
        displayed_force_sensor_index++;

        if (displayed_force_sensor_index >= NUM_FORCE_SENSORS) {
            displayed_force_sensor_index = 0;
        }
    }
}

/**
 * @brief LCD status task.
 *
 * Initialize LCD and periodically show live status.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config pass-in.
 *
 * @return None.
 *     Task intended to run forever.
 */
void system_status_lcd_task(void *argument)
{
    (void)argument;

    char first_line_text[17];
    char second_line_text[17];

    ESP_ERROR_CHECK(initialize_character_lcd());

    write_character_lcd_two_line_status(
        "ESP32 booting...",
        "uROS sensors init");

    while (1) {
        format_system_status_lcd_lines(first_line_text, second_line_text);
        write_character_lcd_two_line_status(first_line_text, second_line_text);
        vTaskDelay(pdMS_TO_TICKS(SYSTEM_STATUS_LCD_UPDATE_PERIOD_MILLISECONDS));
    }
}

/**
 * @brief micro-ROS entity bring-up.
 *
 * Initialize support, node, publishers, subscriber, timer, executor.
 *
 * @param allocator
 *     rcl allocator.
 *     Used by all entity-init calls.
 * @param support
 *     Support object pointer.
 *     Receives initialized support context.
 * @param node
 *     Node object pointer.
 *     Receives initialized node.
 * @param sensor_publish_timer
 *     Timer object pointer.
 *     Receives initialized publish timer.
 * @param executor
 *     Executor object pointer.
 *     Receives initialized executor.
 *
 * @return true on success.
 *     All entities ready.
 *     Normal session start.
 * @return false on failure.
 *     One or more init calls failed.
 *     Caller should clean up and retry later.
 */
static bool initialize_micro_ros_entities(
    rcl_allocator_t *allocator,
    rclc_support_t *support,
    rcl_node_t *node,
    rcl_timer_t *sensor_publish_timer,
    rclc_executor_t *executor)
{
    rcl_ret_t result;

    result = rclc_support_init(support, 0, NULL, allocator);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_node_init_default(
        node,
        MICRO_ROS_NODE_NAME,
        "",
        support);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_publisher_init_default(
        &imu_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        MICRO_ROS_IMU_TOPIC_NAME);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_publisher_init_default(
        &force_sensor_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(klann_msgs, msg, ForceSensorVoltages),
        MICRO_ROS_FORCE_SENSOR_TOPIC_NAME);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_subscription_init_default(
        &velocity_command_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        MICRO_ROS_VELOCITY_COMMAND_TOPIC_NAME);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_timer_init_default2(
        sensor_publish_timer,
        support,
        RCL_MS_TO_NS(MICRO_ROS_PUBLISH_PERIOD_MILLISECONDS),
        sensor_publish_timer_callback,
        true);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_executor_init(executor, &support->context, 2, allocator);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_executor_add_timer(executor, sensor_publish_timer);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    result = rclc_executor_add_subscription(
        executor,
        &velocity_command_subscriber,
        &velocity_command_message,
        &velocity_command_callback,
        ON_NEW_DATA);
    if (result != RCL_RET_OK) {
        record_micro_ros_error(__LINE__, (int32_t)result);
        return false;
    }

    return true;
}

/**
 * @brief micro-ROS entity teardown.
 *
 * Finalize executor, timer, pubs, sub, node, support.
 *
 * @param support
 *     Support object pointer.
 *     Torn down if initialized.
 * @param node
 *     Node object pointer.
 *     Torn down if initialized.
 * @param sensor_publish_timer
 *     Timer object pointer.
 *     Torn down if initialized.
 * @param executor
 *     Executor object pointer.
 *     Torn down if initialized.
 *
 * @return None.
 */
static void deinitialize_micro_ros_entities(
    rclc_support_t *support,
    rcl_node_t *node,
    rcl_timer_t *sensor_publish_timer,
    rclc_executor_t *executor)
{
    rclc_executor_fini(executor);
    rcl_timer_fini(sensor_publish_timer);
    rcl_subscription_fini(&velocity_command_subscriber, node);
    rcl_publisher_fini(&force_sensor_publisher, node);
    rcl_publisher_fini(&imu_publisher, node);
    rcl_node_fini(node);
    rclc_support_fini(support);
}

/**
 * @brief micro-ROS bridge task.
 *
 * Retry until the agent is reachable.
 * Once connected, create node / pubs / sub / timer / executor.
 * If the session drops, tear down entities and retry.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config pass-in.
 *
 * @return None.
 *     Task intended to run forever.
 */
void micro_ros_task(void *argument)
{
    (void)argument;

    initialize_imu_message_defaults();
    initialize_force_sensor_voltage_message_defaults();
    initialize_velocity_command_message_defaults();

    /*
     * Let boot chatter and startup logs finish before using UART0
     * as binary XRCE transport.
     */
    vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_BOOT_SETTLE_DELAY_MILLISECONDS));

    while (1) {
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rcl_timer_t sensor_publish_timer;
        rclc_executor_t executor;

        memset(&support, 0, sizeof(support));
        node = rcl_get_zero_initialized_node();
        sensor_publish_timer = rcl_get_zero_initialized_timer();
        executor = rclc_executor_get_zero_initialized_executor();

        imu_publisher = rcl_get_zero_initialized_publisher();
        force_sensor_publisher = rcl_get_zero_initialized_publisher();
        velocity_command_subscriber = rcl_get_zero_initialized_subscription();

        micro_ros_runtime_state.agent_reachable = false;
        micro_ros_runtime_state.entities_initialized = false;
        micro_ros_runtime_state.session_running = false;

        rmw_ret_t ping_result = rmw_uros_ping_agent(
            MICRO_ROS_AGENT_PING_TIMEOUT_MILLISECONDS,
            MICRO_ROS_AGENT_PING_ATTEMPTS);

        if (ping_result != RMW_RET_OK) {
            record_micro_ros_error(__LINE__, (int32_t)ping_result);
            vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_RETRY_DELAY_MILLISECONDS));
            continue;
        }

        micro_ros_runtime_state.agent_reachable = true;

        if (!initialize_micro_ros_entities(
                &allocator,
                &support,
                &node,
                &sensor_publish_timer,
                &executor)) {
            micro_ros_runtime_state.entities_initialized = false;
            vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_RETRY_DELAY_MILLISECONDS));
            continue;
        }

        micro_ros_runtime_state.entities_initialized = true;
        micro_ros_runtime_state.session_running = true;
        micro_ros_runtime_state.publish_cycle_count = 0;
        micro_ros_runtime_state.imu_publish_successful = false;
        micro_ros_runtime_state.force_sensor_publish_successful = false;

        uint32_t elapsed_link_check_milliseconds = 0;

        while (1) {
            rcl_ret_t spin_result = rclc_executor_spin_some(
                &executor,
                RCL_MS_TO_NS(MICRO_ROS_EXECUTOR_TIMEOUT_MILLISECONDS));

            if (spin_result != RCL_RET_OK) {
                record_micro_ros_error(__LINE__, (int32_t)spin_result);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_EXECUTOR_TIMEOUT_MILLISECONDS));
            elapsed_link_check_milliseconds += MICRO_ROS_EXECUTOR_TIMEOUT_MILLISECONDS;

            if (elapsed_link_check_milliseconds >= MICRO_ROS_LINK_CHECK_PERIOD_MILLISECONDS) {
                elapsed_link_check_milliseconds = 0;

                ping_result = rmw_uros_ping_agent(
                    MICRO_ROS_AGENT_PING_TIMEOUT_MILLISECONDS,
                    1);

                if (ping_result != RMW_RET_OK) {
                    record_micro_ros_error(__LINE__, (int32_t)ping_result);
                    break;
                }
            }
        }

        micro_ros_runtime_state.session_running = false;
        micro_ros_runtime_state.entities_initialized = false;
        micro_ros_runtime_state.agent_reachable = false;

        deinitialize_micro_ros_entities(
            &support,
            &node,
            &sensor_publish_timer,
            &executor);

        vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_RETRY_DELAY_MILLISECONDS));
    }
}

/**
 * @brief ESP-IDF app entry point.
 *
 * Configure custom UART transport, start ADC task,
 * start IMU task, start LCD task, then silence logs and
 * start micro-ROS task on UART0.
 *
 * @param None.
 *
 * @return None.
 */
void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&micro_ros_uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif

    xTaskCreate(
        force_sensor_adc_task,
        "force_sensor_adc_task",
        8192,
        NULL,
        6,
        NULL);

    xTaskCreate(
        mpu9250_task,
        "mpu9250_task",
        8192,
        NULL,
        6,
        NULL);

    xTaskCreate(
        system_status_lcd_task,
        "system_status_lcd_task",
        4096,
        NULL,
        4,
        NULL);

    /*
     * Let startup logs finish, then silence app logging before
     * UART0 begins carrying binary XRCE traffic.
     */
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_log_level_set("*", ESP_LOG_NONE);

    xTaskCreate(
        micro_ros_task,
        "micro_ros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);
}
