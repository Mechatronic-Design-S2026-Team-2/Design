/**
 * @file motor_velocity_bridge.c
 * @brief micro-ROS bridge that directly accepts per-motor RPM commands and
 * publishes raw single-turn encoder counts plus one battery voltage.
 */

#include "motor_velocity_bridge.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dsy_motor_msgs/msg/motor_output_odometry_array.h"
#include "dsy_motor_msgs/msg/motor_rpm_array.h"
#include "dsy_rs485_servo.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_power_manager.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rmw/qos_profiles.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"


#ifndef CONFIG_SERVO_SAFE_MAX_MOTOR_RPM
#define CONFIG_SERVO_SAFE_MAX_MOTOR_RPM 3000
#endif

#ifndef CONFIG_SERVO_SAFE_TORQUE_LIMIT_TENTHS_PERCENT_RATED
#define CONFIG_SERVO_SAFE_TORQUE_LIMIT_TENTHS_PERCENT_RATED 393
#endif

#ifndef CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV
#define CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV 131072
#endif

#ifndef CONFIG_SERVO_OUTPUT_GEAR_RATIO
#define CONFIG_SERVO_OUTPUT_GEAR_RATIO 50
#endif

#ifndef CONFIG_SERVO_SPEED_COMMAND_ACCEL_MS
#define CONFIG_SERVO_SPEED_COMMAND_ACCEL_MS 250
#endif

#ifndef CONFIG_SERVO_SPEED_COMMAND_DECEL_MS
#define CONFIG_SERVO_SPEED_COMMAND_DECEL_MS 250
#endif

#ifndef CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US
#ifdef CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_MS
#define CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US ((uint32_t)CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_MS * 1000U)
#else
#define CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US 0
#endif
#endif

#ifndef CONFIG_SERVO_RUNTIME_ENABLE_PERIODIC_RPM_RESEND
#define CONFIG_SERVO_RUNTIME_ENABLE_PERIODIC_RPM_RESEND 0
#endif

#ifndef CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US
#ifdef CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_MS
#define CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US ((uint32_t)CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_MS * 1000U)
#else
#define CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US 50000
#endif
#endif

#ifndef CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_US
#ifdef CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_MS
#define CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_US ((uint32_t)CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_US 5000
#endif
#endif

#ifndef CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_US
#ifdef CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_MS
#define CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_US ((uint32_t)CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_US 1000
#endif
#endif

#ifndef CONFIG_MOTOR_RPM_SUBSCRIPTION_KEEP_LAST_DEPTH
#define CONFIG_MOTOR_RPM_SUBSCRIPTION_KEEP_LAST_DEPTH 1
#endif

#ifndef CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_US
#ifdef CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_MS
#define CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_US ((uint32_t)CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_US 3000000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_US
#ifdef CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_MS
#define CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_US ((uint32_t)CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_US 3000000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_US
#ifdef CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_MS
#define CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_US ((uint32_t)CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_US 1000000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_US
#ifdef CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_MS
#define CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_US ((uint32_t)CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_US 500000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_US
#ifdef CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_MS
#define CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_US ((uint32_t)CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_US 3000000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_COLD_READY_STABLE_READS
#define CONFIG_MOTOR_BRIDGE_COLD_READY_STABLE_READS 20
#endif

#ifndef CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_US
#ifdef CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_MS
#define CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_US ((uint32_t)CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_US 15000000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_US
#ifdef CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_MS
#define CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_US ((uint32_t)CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_US 200000
#endif
#endif

#ifndef CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US
#ifdef CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_MS
#define CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US ((uint32_t)CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US 10000
#endif
#endif

#ifndef CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_US
#ifdef CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_MS
#define CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_US ((uint32_t)CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_US 0
#endif
#endif

#ifndef CONFIG_RS485_WORKER_TASK_STACK
#define CONFIG_RS485_WORKER_TASK_STACK 6144
#endif

#ifndef CONFIG_RS485_WORKER_TASK_PRIO
#define CONFIG_RS485_WORKER_TASK_PRIO 6
#endif

#ifndef CONFIG_RS485_WORKER_TASK_CORE
#define CONFIG_RS485_WORKER_TASK_CORE 1
#endif

#ifndef CONFIG_RS485_WORKER_LOOP_DELAY_US
#ifdef CONFIG_RS485_WORKER_LOOP_DELAY_MS
#define CONFIG_RS485_WORKER_LOOP_DELAY_US ((uint32_t)CONFIG_RS485_WORKER_LOOP_DELAY_MS * 1000U)
#else
#define CONFIG_RS485_WORKER_LOOP_DELAY_US 1000
#endif
#endif

#ifndef CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US
#ifdef CONFIG_SERVO_POSITION_POLL_PERIOD_US
#define CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US CONFIG_SERVO_POSITION_POLL_PERIOD_US
#elif defined(CONFIG_SERVO_POSITION_POLL_PERIOD_MS)
#define CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US ((uint32_t)CONFIG_SERVO_POSITION_POLL_PERIOD_MS * 1000U)
#else
#define CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US 25000
#endif
#endif

#ifndef CONFIG_SERVO_ENCODER_SWEEP_STOP_PREEMPTION
#define CONFIG_SERVO_ENCODER_SWEEP_STOP_PREEMPTION 1
#endif

#ifndef CONFIG_SERVO_POSITION_POLL_PERIOD_US
#define CONFIG_SERVO_POSITION_POLL_PERIOD_US CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US
#endif

#ifndef CONFIG_SERVO_POSITION_HIGH_PRIORITY_MAX_GAP_US
#define CONFIG_SERVO_POSITION_HIGH_PRIORITY_MAX_GAP_US 0
#endif

#ifndef CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US
#ifdef CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_MS
#define CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US ((uint32_t)CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_MS * 1000U)
#else
#define CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US 50000
#endif
#endif

#ifndef CONFIG_SERVO_ROTATE_COMMAND_WRITE_START_INDEX
#define CONFIG_SERVO_ROTATE_COMMAND_WRITE_START_INDEX 1
#endif

#ifndef CONFIG_SERVO_ODOM_STOP_SPEED_RPM
#define CONFIG_SERVO_ODOM_STOP_SPEED_RPM 2
#endif

#ifndef CONFIG_SERVO_ODOM_STATIONARY_NOISE_COUNTS
#define CONFIG_SERVO_ODOM_STATIONARY_NOISE_COUNTS 2048
#endif

#ifndef CONFIG_SERVO_ODOM_SPEED_MARGIN_RPM
#define CONFIG_SERVO_ODOM_SPEED_MARGIN_RPM 30
#endif

#ifndef CONFIG_SERVO_ODOM_MIN_DELTA_MARGIN_COUNTS
#define CONFIG_SERVO_ODOM_MIN_DELTA_MARGIN_COUNTS 8192
#endif

#define MOTOR_COUNT                              NUM_DSY_RS485_SERVO_DRIVES
#define MOTOR_RPM_COMMAND_TOPIC_NAME             "motor_rpm_cmd"
#define MOTOR_OUTPUT_ODOMETRY_TOPIC_NAME         "motor_output_odom"
#define MOTOR_EXECUTOR_HANDLES                   1
#define MOTOR_EXECUTOR_SPIN_TIMEOUT_NS           ((uint64_t)CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_US * 1000ULL)
#define MOTOR_MAIN_LOOP_DELAY_US                 CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_US
#define MOTOR_PRE_CONFIG_OFF_DWELL_US            CONFIG_MOTOR_BRIDGE_PRE_CONFIG_OFF_DWELL_US
#define MOTOR_TORQUE_PRIME_ENABLE_SETTLE_US      CONFIG_MOTOR_BRIDGE_TORQUE_PRIME_ENABLE_SETTLE_US
#define MOTOR_POST_PRIME_OFF_DWELL_US            CONFIG_MOTOR_BRIDGE_POST_PRIME_OFF_DWELL_US
#define MOTOR_PRE_ENABLE_ZERO_DWELL_US           CONFIG_MOTOR_BRIDGE_PRE_ENABLE_ZERO_DWELL_US
#define MOTOR_SPEED_ENABLE_SETTLE_US             CONFIG_MOTOR_BRIDGE_SPEED_ENABLE_SETTLE_US
#define MOTOR_COLD_READY_STABLE_READS            CONFIG_MOTOR_BRIDGE_COLD_READY_STABLE_READS
#define MOTOR_COLD_READY_TIMEOUT_US              CONFIG_MOTOR_BRIDGE_COLD_READY_TIMEOUT_US
#define MOTOR_COLD_READY_POLL_US                 CONFIG_MOTOR_BRIDGE_COLD_READY_POLL_US

static const char *motor_velocity_bridge_log_tag = "motor_rpm_bridge";

static TickType_t ticks_from_microseconds(uint32_t timeout_us)
{
    if (timeout_us == 0U) {
        return 0;
    }

    const uint32_t timeout_ms_ceil = (timeout_us + 999U) / 1000U;
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms_ceil);
    return (timeout_ticks > 0) ? timeout_ticks : 1;
}

static void delay_microseconds(uint32_t delay_us)
{
    if (delay_us == 0U) {
        return;
    }

    if (delay_us < 1000U) {
        esp_rom_delay_us(delay_us);
    } else {
        vTaskDelay(ticks_from_microseconds(delay_us));
    }
}

static motor_velocity_bridge_status_t motor_velocity_bridge_status = {0};
static dsy_motor_msgs__msg__MotorRpmArray motor_velocity_bridge_rpm_command_message;
static dsy_motor_msgs__msg__MotorOutputOdometryArray motor_velocity_bridge_odometry_message;

static SemaphoreHandle_t motor_velocity_bridge_mutex = NULL;
static TaskHandle_t motor_velocity_bridge_rs485_task_handle = NULL;
static bool motor_velocity_bridge_drives_prepared = false;

static int16_t motor_velocity_bridge_latched_motor_rpm[MOTOR_COUNT] = {0};
static int32_t motor_velocity_bridge_absolute_position_counts[MOTOR_COUNT] = {0};
static float motor_velocity_bridge_output_phase_rad[MOTOR_COUNT] = {0.0f};
static bool motor_velocity_bridge_comm_ok[MOTOR_COUNT] = {false};
static int64_t motor_velocity_bridge_last_raw_feedback_counts[MOTOR_COUNT] = {0};
static bool motor_velocity_bridge_feedback_tracker_initialized[MOTOR_COUNT] = {false};
static uint64_t motor_velocity_bridge_last_feedback_time_us[MOTOR_COUNT] = {0ULL};
static float motor_velocity_bridge_battery_voltage_v = 0.0f;

static bool motor_velocity_bridge_has_received_command = false;
static uint32_t motor_velocity_bridge_command_write_count = 0U;
static uint32_t motor_velocity_bridge_odometry_publish_count = 0U;
static uint32_t motor_velocity_bridge_received_command_count = 0U;
static bool motor_velocity_bridge_pending_command_write = false;
static uint32_t motor_velocity_bridge_command_generation = 0U;
static uint32_t motor_velocity_bridge_last_applied_command_generation = 0U;
static bool motor_velocity_bridge_command_write_in_progress = false;
static int16_t motor_velocity_bridge_last_written_motor_rpm[MOTOR_COUNT] = {0};
static bool motor_velocity_bridge_last_written_motor_rpm_valid[MOTOR_COUNT] = {false};

/*
 * microsecond timing fields mirrored into MotorOutputOdometryArray.
 * last_cmd_seq/esp_cmd_rx_time_us/esp_apply_complete_time_us describe the
 * latest command whose full changed-RPM write set has been acknowledged.
 * If a command changes only command_seq while RPM targets are already applied,
 * it is marked applied without consuming the RS485 bus.
 */
static uint32_t motor_velocity_bridge_latched_command_seq = 0U;
static uint64_t motor_velocity_bridge_latched_command_rx_time_us = 0ULL;
static uint32_t motor_velocity_bridge_last_applied_command_seq = 0U;
static uint64_t motor_velocity_bridge_last_applied_command_rx_time_us = 0ULL;
static uint64_t motor_velocity_bridge_last_apply_complete_time_us = 0ULL;
static uint64_t motor_velocity_bridge_last_poll_complete_time_us = 0ULL;
static uint64_t motor_velocity_bridge_last_command_apply_tick_us = 0ULL;
static uint64_t motor_velocity_bridge_last_command_receive_tick_us = 0ULL;
static uint64_t motor_velocity_bridge_last_resend_tick_us = 0ULL;
static uint64_t motor_velocity_bridge_last_position_poll_tick_us = 0ULL;
static uint64_t motor_velocity_bridge_last_battery_update_tick_us = 0ULL;
static uint64_t motor_velocity_bridge_last_encoder_sweep_start_time_us = 0ULL;
static int motor_velocity_bridge_next_poll_index = 0;
static int motor_velocity_bridge_next_write_start_index = 0;
static bool motor_velocity_bridge_encoder_sweep_active = false;

static int16_t clamp_motor_rpm_to_effective_limit(int motor_rpm)
{
    const int safe_limit = CONFIG_SERVO_SAFE_MAX_MOTOR_RPM;

    if (motor_rpm > safe_limit) {
        return (int16_t)safe_limit;
    }
    if (motor_rpm < -safe_limit) {
        return (int16_t)(-safe_limit);
    }
    return (int16_t)motor_rpm;
}

static void refresh_status_summary_locked(void)
{
    uint8_t comm_ok_drive_count = 0U;

    motor_velocity_bridge_status.hardware_servo_enabled =
        dsy_rs485_is_servo_enable_asserted();
    motor_velocity_bridge_status.has_received_command =
        motor_velocity_bridge_has_received_command;
    motor_velocity_bridge_status.command_write_count =
        motor_velocity_bridge_command_write_count;
    motor_velocity_bridge_status.odometry_publish_count =
        motor_velocity_bridge_odometry_publish_count;
    motor_velocity_bridge_status.received_command_count =
        motor_velocity_bridge_received_command_count;
    motor_velocity_bridge_status.pending_command_write =
        motor_velocity_bridge_pending_command_write;
    motor_velocity_bridge_status.battery_voltage_v =
        motor_velocity_bridge_battery_voltage_v;

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        if (motor_velocity_bridge_comm_ok[motor_index]) {
            comm_ok_drive_count++;
        }

        motor_velocity_bridge_status.latched_motor_rpm[motor_index] =
            motor_velocity_bridge_latched_motor_rpm[motor_index];
        motor_velocity_bridge_status.absolute_position_counts[motor_index] =
            motor_velocity_bridge_absolute_position_counts[motor_index];
        motor_velocity_bridge_status.output_phase_rad[motor_index] =
            motor_velocity_bridge_output_phase_rad[motor_index];
    }

    motor_velocity_bridge_status.comm_ok_drive_count = comm_ok_drive_count;
}

static void refresh_status_summary_from_shared_state(void)
{
    if ((motor_velocity_bridge_mutex != NULL) &&
        (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE)) {
        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }
}

static void mark_latched_command_applied_locked(uint64_t apply_complete_time_us)
{
    motor_velocity_bridge_last_applied_command_seq =
        motor_velocity_bridge_latched_command_seq;
    motor_velocity_bridge_last_applied_command_rx_time_us =
        motor_velocity_bridge_latched_command_rx_time_us;
    motor_velocity_bridge_last_apply_complete_time_us = apply_complete_time_us;
}

static void zero_latched_speed_commands_locked(void)
{
    memset(motor_velocity_bridge_latched_motor_rpm, 0, sizeof(motor_velocity_bridge_latched_motor_rpm));
    motor_velocity_bridge_pending_command_write = true;
    motor_velocity_bridge_command_generation++;
}

static esp_err_t write_selected_motor_rpm_to_drives(
    const int16_t motor_rpm_command[MOTOR_COUNT],
    const bool write_drive_mask[MOTOR_COUNT],
    bool write_success_mask[MOTOR_COUNT],
    bool use_runtime_policy)
{
    esp_err_t last_error = ESP_OK;
    bool wrote_any_drive = false;
    const int start_index = motor_velocity_bridge_next_write_start_index;

    if (write_success_mask != NULL) {
        memset(write_success_mask, 0, sizeof(bool) * MOTOR_COUNT);
    }

    for (int offset = 0; offset < MOTOR_COUNT; offset++) {
        const int motor_index = (start_index + offset) % MOTOR_COUNT;

        if (!write_drive_mask[motor_index]) {
            continue;
        }

        wrote_any_drive = true;
        const int16_t clamped_motor_rpm = clamp_motor_rpm_to_effective_limit(
            (int)motor_rpm_command[motor_index]);
        const esp_err_t write_result = use_runtime_policy
            ? dsy_rs485_runtime_write_drive_speed_command_rpm(motor_index, clamped_motor_rpm)
            : dsy_rs485_write_drive_speed_command_rpm(motor_index, clamped_motor_rpm);

        if ((write_result == ESP_OK) && (write_success_mask != NULL)) {
            write_success_mask[motor_index] = true;
        }

        if ((write_result != ESP_OK) && (last_error == ESP_OK)) {
            last_error = write_result;
        }
    }

#if CONFIG_SERVO_ROTATE_COMMAND_WRITE_START_INDEX
    if (wrote_any_drive) {
        motor_velocity_bridge_next_write_start_index++;
        if (motor_velocity_bridge_next_write_start_index >= MOTOR_COUNT) {
            motor_velocity_bridge_next_write_start_index = 0;
        }
    }
#endif

    return last_error;
}

static esp_err_t write_motor_rpm_array_to_all_drives(const int16_t motor_rpm_command[MOTOR_COUNT])
{
    bool write_drive_mask[MOTOR_COUNT] = {false};
    bool write_success_mask[MOTOR_COUNT] = {false};

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        write_drive_mask[motor_index] = true;
    }

    return write_selected_motor_rpm_to_drives(
        motor_rpm_command,
        write_drive_mask,
        write_success_mask,
        false);
}

static bool is_latest_command_pending_locked(void)
{
    return motor_velocity_bridge_pending_command_write &&
           (motor_velocity_bridge_command_generation !=
            motor_velocity_bridge_last_applied_command_generation);
}

static bool is_latest_command_pending_from_shared_state(void)
{
    bool is_pending = false;

    if ((motor_velocity_bridge_mutex != NULL) &&
        (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(1)) == pdTRUE)) {
        is_pending = is_latest_command_pending_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }

    return is_pending;
}

static bool is_zero_rpm_vector_locked(void)
{
    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        if (motor_velocity_bridge_latched_motor_rpm[motor_index] != 0) {
            return false;
        }
    }

    return true;
}

static bool is_pending_zero_command_from_shared_state(void)
{
#if CONFIG_SERVO_ENCODER_SWEEP_STOP_PREEMPTION
    bool is_zero_pending = false;

    if ((motor_velocity_bridge_mutex != NULL) &&
        (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(1)) == pdTRUE)) {
        is_zero_pending = is_latest_command_pending_locked() &&
                          is_zero_rpm_vector_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }

    return is_zero_pending;
#else
    return false;
#endif
}

static esp_err_t copy_and_apply_latest_command(bool is_periodic_resend)
{
    int16_t local_motor_rpm[MOTOR_COUNT] = {0};
    bool write_drive_mask[MOTOR_COUNT] = {false};
    bool write_success_mask[MOTOR_COUNT] = {false};
    uint32_t command_generation_snapshot = 0U;
    uint32_t local_command_seq = 0U;
    uint64_t local_command_rx_time_us = 0ULL;
    bool should_apply = false;
    bool has_any_drive_to_write = false;

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    const bool minimum_interval_satisfied =
        (CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_US <= 0) ||
        ((now_us - motor_velocity_bridge_last_command_apply_tick_us) >=
         (uint64_t)CONFIG_MICRO_ROS_COMMAND_APPLY_MIN_INTERVAL_US);

    if (is_periodic_resend) {
        should_apply = motor_velocity_bridge_has_received_command && minimum_interval_satisfied;
    } else {
        should_apply = is_latest_command_pending_locked() && minimum_interval_satisfied;
    }

    if (should_apply) {
        memcpy(local_motor_rpm, motor_velocity_bridge_latched_motor_rpm, sizeof(local_motor_rpm));
        command_generation_snapshot = motor_velocity_bridge_command_generation;
        local_command_seq = motor_velocity_bridge_latched_command_seq;
        local_command_rx_time_us = motor_velocity_bridge_latched_command_rx_time_us;

        for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
            const bool drive_needs_write =
                is_periodic_resend ||
                !motor_velocity_bridge_last_written_motor_rpm_valid[motor_index] ||
                (local_motor_rpm[motor_index] !=
                 motor_velocity_bridge_last_written_motor_rpm[motor_index]);

            write_drive_mask[motor_index] = drive_needs_write;
            has_any_drive_to_write = has_any_drive_to_write || drive_needs_write;
        }

        if (!has_any_drive_to_write && !is_periodic_resend) {
            motor_velocity_bridge_pending_command_write = false;
            motor_velocity_bridge_last_applied_command_generation = command_generation_snapshot;
            mark_latched_command_applied_locked((uint64_t)esp_timer_get_time());
            refresh_status_summary_locked();
        }

        if (has_any_drive_to_write) {
            motor_velocity_bridge_command_write_in_progress = true;
        }
    }

    xSemaphoreGive(motor_velocity_bridge_mutex);

    if (!should_apply || !has_any_drive_to_write) {
        return ESP_OK;
    }

    const esp_err_t write_result = write_selected_motor_rpm_to_drives(
        local_motor_rpm,
        write_drive_mask,
        write_success_mask,
        true);

    (void)xSemaphoreTake(motor_velocity_bridge_mutex, portMAX_DELAY);
    {
        bool all_requested_writes_succeeded = true;

        motor_velocity_bridge_command_write_in_progress = false;
        motor_velocity_bridge_command_write_count++;
        motor_velocity_bridge_status.last_error_code = (int32_t)write_result;
        motor_velocity_bridge_last_command_apply_tick_us = (uint64_t)esp_timer_get_time();
        motor_velocity_bridge_last_resend_tick_us = motor_velocity_bridge_last_command_apply_tick_us;

        for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
            if (write_drive_mask[motor_index] && !write_success_mask[motor_index]) {
                all_requested_writes_succeeded = false;
            }

            if (write_success_mask[motor_index]) {
                motor_velocity_bridge_last_written_motor_rpm[motor_index] =
                    local_motor_rpm[motor_index];
                motor_velocity_bridge_last_written_motor_rpm_valid[motor_index] = true;
            }
        }

        if (!is_periodic_resend) {
            if ((command_generation_snapshot == motor_velocity_bridge_command_generation) &&
                all_requested_writes_succeeded) {
                motor_velocity_bridge_pending_command_write = false;
                motor_velocity_bridge_last_applied_command_generation = command_generation_snapshot;
                /*
                 * If newer packets with the same RPM vector arrived during the
                 * write, generation is unchanged and the current latched seq is
                 * also physically applied. Report that latest equivalent seq.
                 */
                mark_latched_command_applied_locked((uint64_t)esp_timer_get_time());
            } else {
                motor_velocity_bridge_pending_command_write = true;
                if (all_requested_writes_succeeded) {
                    motor_velocity_bridge_last_applied_command_seq = local_command_seq;
                    motor_velocity_bridge_last_applied_command_rx_time_us = local_command_rx_time_us;
                    motor_velocity_bridge_last_apply_complete_time_us =
                        (uint64_t)esp_timer_get_time();
                }
            }
        }

        refresh_status_summary_locked();
    }
    xSemaphoreGive(motor_velocity_bridge_mutex);

    return write_result;
}

static bool should_start_or_continue_encoder_sweep(uint64_t now_us)
{
#if CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US > 0
    if (motor_velocity_bridge_encoder_sweep_active) {
        return true;
    }

    if ((motor_velocity_bridge_last_encoder_sweep_start_time_us == 0ULL) ||
        ((now_us - motor_velocity_bridge_last_encoder_sweep_start_time_us) >=
         (uint64_t)CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US)) {
        motor_velocity_bridge_encoder_sweep_active = true;
        motor_velocity_bridge_next_poll_index = 0;
        motor_velocity_bridge_last_encoder_sweep_start_time_us = now_us;
        motor_velocity_bridge_last_position_poll_tick_us = now_us;
        return true;
    }
#else
    (void)now_us;
#endif

    return false;
}

static void advance_next_position_poll_index_after_poll(int polled_drive_index)
{
    motor_velocity_bridge_next_poll_index = polled_drive_index + 1;
    if (motor_velocity_bridge_next_poll_index >= MOTOR_COUNT) {
        motor_velocity_bridge_next_poll_index = 0;
        motor_velocity_bridge_encoder_sweep_active = false;
    }
}

static esp_err_t poll_one_drive_output_position_only(int drive_index)
{
    uint32_t encoder_single_turn_counts = 0U;
    const uint64_t poll_time_us = (uint64_t)esp_timer_get_time();
    const esp_err_t result =
        dsy_rs485_runtime_read_drive_absolute_encoder_single_turn_counts(
            drive_index,
            &encoder_single_turn_counts);

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (result == ESP_OK) {
            /*
             * Publish the raw P18.32/P18.33 single-turn encoder count.
             * No local phase calculation, modulo unwrap, or gear-ratio scaling
             * is applied here. The ROS-side controller/model should combine
             * this modulo raw count with the timing fields to estimate phase.
             */
            motor_velocity_bridge_feedback_tracker_initialized[drive_index] = true;
            motor_velocity_bridge_last_raw_feedback_counts[drive_index] =
                (int64_t)encoder_single_turn_counts;
            motor_velocity_bridge_last_feedback_time_us[drive_index] = poll_time_us;

            /*
             * Field names are retained for message/API compatibility.
             * output_phase_rad[] now carries raw encoder counts as float values.
             * M17S raw counts are 17-bit, so they are exactly representable in
             * float32. absolute_position_counts[] mirrors the same raw value
             * for internal status/debug callers.
             */
            motor_velocity_bridge_absolute_position_counts[drive_index] =
                (int32_t)encoder_single_turn_counts;
            motor_velocity_bridge_output_phase_rad[drive_index] =
                (float)encoder_single_turn_counts;
            motor_velocity_bridge_comm_ok[drive_index] = true;
            motor_velocity_bridge_last_poll_complete_time_us = poll_time_us;
        } else {
            motor_velocity_bridge_comm_ok[drive_index] = false;
            motor_velocity_bridge_status.last_error_code = (int32_t)result;
        }

        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }

    return result;
}

static void refresh_cached_battery_voltage_from_adc(void)
{
    const motor_power_manager_status_t power_status = get_motor_power_manager_status();

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motor_velocity_bridge_battery_voltage_v = power_status.latest_bus_voltage_v;
        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }
}

static bool populate_raw_encoder_message_from_cache(void)
{
    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    memset(&motor_velocity_bridge_odometry_message, 0, sizeof(motor_velocity_bridge_odometry_message));

    motor_velocity_bridge_odometry_message.last_cmd_seq =
        motor_velocity_bridge_last_applied_command_seq;
    motor_velocity_bridge_odometry_message.esp_cmd_rx_time_us =
        motor_velocity_bridge_last_applied_command_rx_time_us;
    motor_velocity_bridge_odometry_message.esp_apply_complete_time_us =
        motor_velocity_bridge_last_apply_complete_time_us;
    motor_velocity_bridge_odometry_message.esp_poll_complete_time_us =
        motor_velocity_bridge_last_poll_complete_time_us;

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        motor_velocity_bridge_odometry_message.output_phase_rad[motor_index] =
            motor_velocity_bridge_output_phase_rad[motor_index];
    }
    motor_velocity_bridge_odometry_message.battery_voltage_v =
        motor_velocity_bridge_battery_voltage_v;
    motor_velocity_bridge_odometry_message.esp_publish_time_us =
        (uint64_t)esp_timer_get_time();

    xSemaphoreGive(motor_velocity_bridge_mutex);
    return true;
}

static void motor_rpm_command_callback(const void *message_in)
{
    const dsy_motor_msgs__msg__MotorRpmArray *message =
        (const dsy_motor_msgs__msg__MotorRpmArray *)message_in;
    bool command_changed = false;
    bool command_sequence_changed = false;
    bool command_write_in_progress_snapshot = false;
    const uint64_t command_rx_time_us = (uint64_t)esp_timer_get_time();

    if ((message == NULL) || (motor_velocity_bridge_mutex == NULL)) {
        return;
    }

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        const bool had_previous_command = motor_velocity_bridge_has_received_command;
        command_write_in_progress_snapshot = motor_velocity_bridge_command_write_in_progress;
        command_sequence_changed =
            !had_previous_command ||
            (motor_velocity_bridge_latched_command_seq != message->command_seq);

        for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
            const int16_t clamped_motor_rpm =
                clamp_motor_rpm_to_effective_limit((int)message->motor_rpm[motor_index]);

            if (!had_previous_command ||
                (motor_velocity_bridge_latched_motor_rpm[motor_index] != clamped_motor_rpm)) {
                command_changed = true;
            }

            motor_velocity_bridge_latched_motor_rpm[motor_index] = clamped_motor_rpm;
        }

        motor_velocity_bridge_latched_command_seq = message->command_seq;
        motor_velocity_bridge_latched_command_rx_time_us = command_rx_time_us;
        motor_velocity_bridge_has_received_command = true;
        motor_velocity_bridge_received_command_count++;
        motor_velocity_bridge_last_command_receive_tick_us = command_rx_time_us;

        if (command_changed) {
            motor_velocity_bridge_pending_command_write = true;
            motor_velocity_bridge_command_generation++;
        } else if (command_sequence_changed && !motor_velocity_bridge_pending_command_write) {
            /*
             * New sequence number with an already-applied RPM vector: no RS485
             * write is needed, but echo the sequence/timing for controller
             * propagation logging.
             */
            motor_velocity_bridge_last_applied_command_generation =
                motor_velocity_bridge_command_generation;
            mark_latched_command_applied_locked(command_rx_time_us);
        }

        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }

    if (command_changed &&
        !command_write_in_progress_snapshot &&
        (motor_velocity_bridge_rs485_task_handle != NULL)) {
        xTaskNotifyGive(motor_velocity_bridge_rs485_task_handle);
    }
}

static void motor_velocity_bridge_rs485_worker_task(void *argument)
{
    (void)argument;

    refresh_cached_battery_voltage_from_adc();

    while (true) {
        (void)ulTaskNotifyTake(
            pdTRUE,
            ticks_from_microseconds(CONFIG_RS485_WORKER_LOOP_DELAY_US > 0 ? CONFIG_RS485_WORKER_LOOP_DELAY_US : 1000));

        uint64_t now_us = (uint64_t)esp_timer_get_time();

        if (is_pending_zero_command_from_shared_state()) {
            /*
             * Safety / operator stop takes precedence over any scheduled raw
             * encoder sweep. Preemption happens between RS485 transactions;
             * an in-flight Modbus read is still allowed to complete.
             */
            (void)copy_and_apply_latest_command(false);
            continue;
        }

        if (motor_velocity_bridge_encoder_sweep_active) {
            if (is_pending_zero_command_from_shared_state()) {
                (void)copy_and_apply_latest_command(false);
                continue;
            }

            const int position_poll_index = motor_velocity_bridge_next_poll_index;
            (void)poll_one_drive_output_position_only(position_poll_index);
            advance_next_position_poll_index_after_poll(position_poll_index);
            motor_velocity_bridge_last_position_poll_tick_us = (uint64_t)esp_timer_get_time();
            if (motor_velocity_bridge_encoder_sweep_active) {
                xTaskNotifyGive(xTaskGetCurrentTaskHandle());
            }
            continue;
        }

        if (is_latest_command_pending_from_shared_state()) {
            (void)copy_and_apply_latest_command(false);
            continue;
        }

        if (should_start_or_continue_encoder_sweep(now_us)) {
            const int position_poll_index = motor_velocity_bridge_next_poll_index;
            (void)poll_one_drive_output_position_only(position_poll_index);
            advance_next_position_poll_index_after_poll(position_poll_index);
            motor_velocity_bridge_last_position_poll_tick_us = (uint64_t)esp_timer_get_time();
            if (motor_velocity_bridge_encoder_sweep_active) {
                xTaskNotifyGive(xTaskGetCurrentTaskHandle());
            }
            continue;
        }

#if CONFIG_SERVO_RUNTIME_ENABLE_PERIODIC_RPM_RESEND
        if ((CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US > 0) &&
            ((now_us - motor_velocity_bridge_last_resend_tick_us) >=
             (uint64_t)CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US)) {
            (void)copy_and_apply_latest_command(true);
            continue;
        }
#endif

        if ((CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US > 0) &&
            ((now_us - motor_velocity_bridge_last_battery_update_tick_us) >=
             (uint64_t)CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US)) {
            refresh_cached_battery_voltage_from_adc();
            motor_velocity_bridge_last_battery_update_tick_us = (uint64_t)esp_timer_get_time();
        }

        if (CONFIG_RS485_WORKER_LOOP_DELAY_US > 0) {
            delay_microseconds(CONFIG_RS485_WORKER_LOOP_DELAY_US);
        }
    }
}

static void seed_cached_raw_encoder_from_drive_state(void)
{
    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        const dsy_rs485_servo_state_t state = get_dsy_rs485_servo_state(motor_index);

        /*
         * Do not seed runtime odometry from P18.07. The published value is the
         * raw P18.32/P18.33 single-turn encoder count once the first runtime
         * sample succeeds.
         */
        motor_velocity_bridge_absolute_position_counts[motor_index] = 0;
        motor_velocity_bridge_last_raw_feedback_counts[motor_index] = 0;
        motor_velocity_bridge_last_feedback_time_us[motor_index] = 0ULL;
        motor_velocity_bridge_output_phase_rad[motor_index] = 0.0f;
        motor_velocity_bridge_feedback_tracker_initialized[motor_index] = false;
        motor_velocity_bridge_comm_ok[motor_index] = state.communication_ok;
    }

    const motor_power_manager_status_t power_status = get_motor_power_manager_status();
    motor_velocity_bridge_battery_voltage_v = power_status.latest_bus_voltage_v;
    refresh_status_summary_locked();
    xSemaphoreGive(motor_velocity_bridge_mutex);
}

static const char *get_motor_bridge_stage_short_text_impl(motor_bridge_stage_t stage)
{
    switch (stage) {
        case motor_bridge_stage_boot: return "BOOT";
        case motor_bridge_stage_preparing_bus: return "BUS";
        case motor_bridge_stage_ready: return "READY";
        case motor_bridge_stage_fault: return "FAULT";
        default: return "UNKN";
    }
}

const char *get_motor_bridge_stage_short_text(motor_bridge_stage_t stage)
{
    return get_motor_bridge_stage_short_text_impl(stage);
}

motor_velocity_bridge_status_t get_motor_velocity_bridge_status(void)
{
    refresh_status_summary_from_shared_state();
    return motor_velocity_bridge_status;
}

esp_err_t motor_velocity_bridge_prepare_drives_before_ros(void)
{
    motor_velocity_bridge_status.stage = motor_bridge_stage_preparing_bus;
    ESP_RETURN_ON_ERROR(dsy_rs485_initialize_bus(), motor_velocity_bridge_log_tag, "bus init failed");

    ESP_LOGI(
        motor_velocity_bridge_log_tag,
        "prepare timing off_dwell_us=%d cold_ready_reads=%d cold_ready_timeout_us=%d cold_ready_poll_us=%d torque_prime_settle_us=%d torque_zero_gap_us=%d post_prime_off_us=%d pre_enable_zero_us=%d speed_enable_settle_us=%d",
        MOTOR_PRE_CONFIG_OFF_DWELL_US,
        MOTOR_COLD_READY_STABLE_READS,
        MOTOR_COLD_READY_TIMEOUT_US,
        MOTOR_COLD_READY_POLL_US,
        MOTOR_TORQUE_PRIME_ENABLE_SETTLE_US,
        CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US,
        MOTOR_POST_PRIME_OFF_DWELL_US,
        MOTOR_PRE_ENABLE_ZERO_DWELL_US,
        MOTOR_SPEED_ENABLE_SETTLE_US);

    ESP_RETURN_ON_ERROR(
        dsy_rs485_set_servo_enable_output(false),
        motor_velocity_bridge_log_tag,
        "servo disable failed");
    motor_velocity_bridge_status.hardware_servo_enabled = false;
    delay_microseconds(MOTOR_PRE_CONFIG_OFF_DWELL_US);

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        ESP_RETURN_ON_ERROR(
            dsy_rs485_wait_for_drive_ready(
                motor_index,
                MOTOR_COLD_READY_STABLE_READS,
                MOTOR_COLD_READY_TIMEOUT_US,
                MOTOR_COLD_READY_POLL_US),
            motor_velocity_bridge_log_tag,
            "drive ready wait failed");
    }

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        ESP_RETURN_ON_ERROR(
            dsy_rs485_configure_drive_for_internal_digit_torque_mode_debug(motor_index),
            motor_velocity_bridge_log_tag,
            "torque-prime config failed");
    }

    ESP_RETURN_ON_ERROR(
        dsy_rs485_set_servo_enable_output(true),
        motor_velocity_bridge_log_tag,
        "servo prime enable failed");
    motor_velocity_bridge_status.hardware_servo_enabled = true;
    delay_microseconds(MOTOR_TORQUE_PRIME_ENABLE_SETTLE_US);

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        const uint8_t slave_address = get_dsy_rs485_servo_slave_address(motor_index);
        (void)dsy_rs485_write_parameter_s16(slave_address, 6U, 5U, 0);
        if (CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US > 0) {
            delay_microseconds(CONFIG_MOTOR_BRIDGE_TORQUE_ZERO_INTER_DRIVE_DELAY_US);
        }
    }

    ESP_RETURN_ON_ERROR(
        dsy_rs485_set_servo_enable_output(false),
        motor_velocity_bridge_log_tag,
        "servo disable before speed config failed");
    motor_velocity_bridge_status.hardware_servo_enabled = false;
    delay_microseconds(MOTOR_POST_PRIME_OFF_DWELL_US);

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        ESP_RETURN_ON_ERROR(
            dsy_rs485_configure_drive_for_internal_digit_speed_mode(
                motor_index,
                (uint16_t)CONFIG_SERVO_SAFE_MAX_MOTOR_RPM,
                (uint16_t)CONFIG_SERVO_SAFE_MAX_MOTOR_RPM,
                (uint16_t)CONFIG_SERVO_SPEED_COMMAND_ACCEL_MS,
                (uint16_t)CONFIG_SERVO_SPEED_COMMAND_DECEL_MS,
                (uint16_t)CONFIG_SERVO_SAFE_TORQUE_LIMIT_TENTHS_PERCENT_RATED),
            motor_velocity_bridge_log_tag,
            "speed-mode config failed");
    }

    if (motor_velocity_bridge_mutex == NULL) {
        motor_velocity_bridge_mutex = xSemaphoreCreateMutex();
        if (motor_velocity_bridge_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        zero_latched_speed_commands_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }
    int16_t zero_command[MOTOR_COUNT] = {0};
    ESP_RETURN_ON_ERROR(
        write_motor_rpm_array_to_all_drives(zero_command),
        motor_velocity_bridge_log_tag,
        "initial zero-speed write failed");

    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
            motor_velocity_bridge_last_written_motor_rpm[motor_index] = 0;
            motor_velocity_bridge_last_written_motor_rpm_valid[motor_index] = true;
        }
        motor_velocity_bridge_pending_command_write = false;
        motor_velocity_bridge_last_applied_command_generation = motor_velocity_bridge_command_generation;
        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }

    delay_microseconds(MOTOR_PRE_ENABLE_ZERO_DWELL_US);

    ESP_RETURN_ON_ERROR(
        dsy_rs485_set_servo_enable_output(true),
        motor_velocity_bridge_log_tag,
        "servo speed enable failed");
    motor_velocity_bridge_status.hardware_servo_enabled = true;
    delay_microseconds(MOTOR_SPEED_ENABLE_SETTLE_US);

    for (int motor_index = 0; motor_index < MOTOR_COUNT; motor_index++) {
        (void)dsy_rs485_poll_one_drive(motor_index);
    }

    motor_velocity_bridge_drives_prepared = true;
    motor_velocity_bridge_status.drives_prepared = true;
    motor_velocity_bridge_status.stage = motor_bridge_stage_ready;
    seed_cached_raw_encoder_from_drive_state();
    return ESP_OK;
}

void motor_velocity_bridge_task(void *argument)
{
    (void)argument;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t odom_publisher;
    rcl_subscription_t rpm_subscription;
    rclc_executor_t executor;

    memset(&support, 0, sizeof(support));
    memset(&node, 0, sizeof(node));
    memset(&odom_publisher, 0, sizeof(odom_publisher));
    memset(&rpm_subscription, 0, sizeof(rpm_subscription));
    memset(&executor, 0, sizeof(executor));
    memset(&motor_velocity_bridge_rpm_command_message, 0, sizeof(motor_velocity_bridge_rpm_command_message));
    memset(&motor_velocity_bridge_odometry_message, 0, sizeof(motor_velocity_bridge_odometry_message));

    if (!motor_velocity_bridge_drives_prepared) {
        if (motor_velocity_bridge_prepare_drives_before_ros() != ESP_OK) {
            motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
            vTaskDelete(NULL);
            return;
        }
    }


    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    if (rclc_node_init_default(&node, "hexapod_motor_rpm_bridge", "", &support) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    if (rclc_publisher_init_best_effort(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dsy_motor_msgs, msg, MotorOutputOdometryArray),
            MOTOR_OUTPUT_ODOMETRY_TOPIC_NAME) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    rcl_subscription_options_t rpm_subscription_options =
        rcl_subscription_get_default_options();
    rpm_subscription_options.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    rpm_subscription_options.qos.depth = CONFIG_MOTOR_RPM_SUBSCRIPTION_KEEP_LAST_DEPTH;
    rpm_subscription_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rpm_subscription_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    if (rcl_subscription_init(
            &rpm_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dsy_motor_msgs, msg, MotorRpmArray),
            MOTOR_RPM_COMMAND_TOPIC_NAME,
            &rpm_subscription_options) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    if (rclc_executor_init(
            &executor,
            &support.context,
            MOTOR_EXECUTOR_HANDLES,
            &allocator) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &rpm_subscription,
            &motor_velocity_bridge_rpm_command_message,
            motor_rpm_command_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
        vTaskDelete(NULL);
        return;
    }

    uint64_t last_odometry_publish_time_us = (uint64_t)esp_timer_get_time();
    if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motor_velocity_bridge_last_command_receive_tick_us = last_odometry_publish_time_us;
        motor_velocity_bridge_last_command_apply_tick_us = last_odometry_publish_time_us;
        motor_velocity_bridge_last_resend_tick_us = last_odometry_publish_time_us;
        motor_velocity_bridge_last_position_poll_tick_us = last_odometry_publish_time_us;
        motor_velocity_bridge_last_encoder_sweep_start_time_us = 0ULL;
        motor_velocity_bridge_encoder_sweep_active = false;
        motor_velocity_bridge_next_poll_index = 0;
        motor_velocity_bridge_last_battery_update_tick_us = last_odometry_publish_time_us;
        refresh_status_summary_locked();
        xSemaphoreGive(motor_velocity_bridge_mutex);
    }


    if (motor_velocity_bridge_rs485_task_handle == NULL) {
        BaseType_t create_result = xTaskCreatePinnedToCore(
            motor_velocity_bridge_rs485_worker_task,
            "motor_rs485_worker",
            CONFIG_RS485_WORKER_TASK_STACK,
            NULL,
            CONFIG_RS485_WORKER_TASK_PRIO,
            &motor_velocity_bridge_rs485_task_handle,
            CONFIG_RS485_WORKER_TASK_CORE);
        if (create_result != pdPASS) {
            motor_velocity_bridge_status.stage = motor_bridge_stage_fault;
            vTaskDelete(NULL);
            return;
        }
    }

    ESP_LOGI(
        motor_velocity_bridge_log_tag,
        "runtime ready subscribe=%s publish=%s latest-only-depth=%d changed-only-writes=1 runtime-rs485=1 raw-encoder-counts-in-output_phase_rad=1 strict-encoder-sweep=1 stop-preempts-sweep=%d mppi-timing-fields=1 periodic-resend=%d resend_us=%d encoder_sweep_us=%d battery_cache_us=%d odom_us=%d executor_spin_us=%d loop_delay_us=%d rs485_worker_loop_delay_us=%d rs485_core=%d",
        MOTOR_RPM_COMMAND_TOPIC_NAME,
        MOTOR_OUTPUT_ODOMETRY_TOPIC_NAME,
        CONFIG_MOTOR_RPM_SUBSCRIPTION_KEEP_LAST_DEPTH,
        CONFIG_SERVO_ENCODER_SWEEP_STOP_PREEMPTION,
        CONFIG_SERVO_RUNTIME_ENABLE_PERIODIC_RPM_RESEND,
        CONFIG_SERVO_COMMAND_LATCH_RESEND_PERIOD_US,
        CONFIG_SERVO_ENCODER_SWEEP_PERIOD_US,
        CONFIG_SERVO_BATTERY_CACHE_UPDATE_PERIOD_US,
        CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US,
        CONFIG_MICRO_ROS_EXECUTOR_SPIN_TIMEOUT_US,
        CONFIG_MICRO_ROS_BRIDGE_MAIN_LOOP_DELAY_US,
        CONFIG_RS485_WORKER_LOOP_DELAY_US,
        CONFIG_RS485_WORKER_TASK_CORE);

    while (true) {
        (void)rclc_executor_spin_some(&executor, MOTOR_EXECUTOR_SPIN_TIMEOUT_NS);

        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        if ((CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US > 0) &&
            ((now_us - last_odometry_publish_time_us) >=
             (uint64_t)CONFIG_SERVO_ODOMETRY_PUBLISH_PERIOD_US)) {
            if (populate_raw_encoder_message_from_cache() &&
                (rcl_publish(&odom_publisher, &motor_velocity_bridge_odometry_message, NULL) == RCL_RET_OK)) {
                if (xSemaphoreTake(motor_velocity_bridge_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    motor_velocity_bridge_odometry_publish_count++;
                    refresh_status_summary_locked();
                    xSemaphoreGive(motor_velocity_bridge_mutex);
                }
            }
            last_odometry_publish_time_us = now_us;
        }

        if (MOTOR_MAIN_LOOP_DELAY_US > 0) {
            delay_microseconds(MOTOR_MAIN_LOOP_DELAY_US);
        }
    }
}
