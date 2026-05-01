/**
 * @file dsy_rs485_servo.h
 * @brief Minimal RS485 Modbus driver for DSY-RS servo smoke testing.
 *
 * This variant is trimmed for first bring-up:
 * - only RS485 TTL + micro-ROS are required
 * - no ADC, IMU, LCD, or custom-message dependencies
 * - one-servo bring-up first, six-servo expansion later
 */

#ifndef DSY_RS485_SERVO_H
#define DSY_RS485_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hexapod_hw_config.h"

#define NUM_DSY_RS485_SERVO_DRIVES HEXAPOD_MOTOR_COUNT

#define DSY_RS485_ERROR_BASE                0x7200
#define DSY_RS485_ERROR_INVALID_RESPONSE    (DSY_RS485_ERROR_BASE + 1)
#define DSY_RS485_ERROR_CRC_MISMATCH        (DSY_RS485_ERROR_BASE + 2)
#define DSY_RS485_ERROR_MODBUS_EXCEPTION    (DSY_RS485_ERROR_BASE + 3)
#define DSY_RS485_ERROR_BUS_NOT_INITIALIZED (DSY_RS485_ERROR_BASE + 4)
#define DSY_RS485_ERROR_INVALID_DRIVE_INDEX (DSY_RS485_ERROR_BASE + 5)

typedef enum
{
    dsy_rs485_control_mode_position = 0,
    dsy_rs485_control_mode_speed = 1,
    dsy_rs485_control_mode_torque = 2,
} dsy_rs485_control_mode_t;

/**
 * @brief Runtime motion sample retained for compatibility with older callers.
 *
 * New odometry code uses dsy_rs485_runtime_read_drive_absolute_encoder_motor_counts()
 * instead, so runtime phase tracking no longer depends on P18.01 speed feedback.
 */
typedef struct
{
    int16_t actual_speed_rpm;
    uint32_t encoder_single_turn_counts;
} dsy_rs485_runtime_motion_sample_t;

esp_err_t dsy_rs485_initialize_bus(void);

typedef struct
{
    uint8_t slave_address;
    bool communication_ok;
    esp_err_t last_communication_result;
    uint8_t last_modbus_exception_code;
    uint16_t servo_status_word;
    uint16_t bus_voltage_deci_volts;
    uint16_t phase_current_rms_centiamps;
    int16_t actual_speed_rpm;
    int32_t absolute_position_counts;
    int32_t position_deviation_counts;
} dsy_rs485_servo_state_t;

/**
 * @brief Background RS485 polling task.
 *
 * Initializes the RS485 bus, then polls each active drive in round-robin.
 * Use active-drive count = 1 for first bring-up, then raise to 6.
 *
 * @param argument Unused FreeRTOS task argument.
 *
 * @return None. Task runs forever.
 */
void dsy_rs485_servo_task(void *argument);

bool is_dsy_rs485_bus_initialized(void);



/**
 * @brief Wait for a drive to return stable readable telemetry after cold power-up.
 *
 * Requires consecutive successful reads of status, bus voltage, and absolute
 * position before returning ESP_OK. Intended for the pre-config startup path
 * before any torque-mode or speed-mode writes are attempted.
 */
esp_err_t dsy_rs485_wait_for_drive_ready(
    int drive_index,
    int stable_reads_required,
    int timeout_us,
    int poll_interval_us);

esp_err_t dsy_rs485_initialize_servo_enable_output(void);
esp_err_t dsy_rs485_set_servo_enable_output(bool enable_servo);
int dsy_rs485_get_servo_enable_gpio_level(void);
bool dsy_rs485_is_servo_enable_asserted(void);
int dsy_rs485_get_servo_enable_active_level_runtime(void);
void dsy_rs485_set_servo_enable_active_level_runtime(int active_level);
void dsy_rs485_restore_default_servo_enable_active_level(void);
void dsy_rs485_debug_log_servo_enable_line(const char *label);
esp_err_t dsy_rs485_debug_dump_drive_snapshot(int drive_index, const char *label);
esp_err_t dsy_rs485_debug_dump_drive_io_config(int drive_index, const char *label);
esp_err_t dsy_rs485_debug_dump_drive_mode_config(int drive_index, const char *label);
esp_err_t dsy_rs485_debug_dump_all_drives(const char *label, bool include_io_config, bool include_mode_config);
int get_dsy_rs485_active_drive_count(void);
dsy_rs485_servo_state_t get_dsy_rs485_servo_state(int drive_index);
uint8_t get_dsy_rs485_servo_slave_address(int drive_index);
uint8_t get_dsy_rs485_last_modbus_exception_code(void);

esp_err_t dsy_rs485_read_holding_registers(
    uint8_t slave_address,
    uint16_t start_register_address,
    uint16_t register_count,
    uint16_t *register_values_out);

esp_err_t dsy_rs485_write_single_register(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t register_value);

esp_err_t dsy_rs485_write_multiple_registers(
    uint8_t slave_address,
    uint16_t start_register_address,
    const uint16_t *register_values,
    uint16_t register_count);

esp_err_t dsy_rs485_read_parameter_u16(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint16_t *value_out);

esp_err_t dsy_rs485_read_parameter_s16(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    int16_t *value_out);

esp_err_t dsy_rs485_read_parameter_u32(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint32_t *value_out);

esp_err_t dsy_rs485_read_parameter_s32(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    int32_t *value_out);

esp_err_t dsy_rs485_write_parameter_u16(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint16_t value);

esp_err_t dsy_rs485_write_parameter_s16(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    int16_t value);

esp_err_t dsy_rs485_write_parameter_u32(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint32_t value);

esp_err_t dsy_rs485_write_parameter_s32(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    int32_t value);

esp_err_t dsy_rs485_set_drive_control_mode(int drive_index, dsy_rs485_control_mode_t control_mode);
esp_err_t dsy_rs485_configure_drive_for_internal_digit_torque_mode(int drive_index);

/**
 * @brief Debug configure helper for internal-digit torque mode.
 *
 * Apply the minimum torque-mode parameter sequence with conservative
 * inter-write delays and readback verification after each step.
 * This is intended for first bench bring-up over a weak bench supply
 * and uncertain drive state.
 *
 * @param drive_index
 *     Local drive array index.
 *     Expected range 0..active_drive_count-1.
 *     Used for slave-address lookup.
 *
 * @return ESP_OK on success.
 *     All writes acknowledged and read back as expected.
 * @return Other esp_err_t on failure.
 *     Timeout, Modbus exception, readback mismatch, or invalid drive index.
 */
esp_err_t dsy_rs485_configure_drive_for_internal_digit_torque_mode_debug(int drive_index);

esp_err_t dsy_rs485_write_drive_torque_command(int drive_index, int16_t torque_command_tenths_percent_rated);
esp_err_t dsy_rs485_set_drive_torque_mode_speed_limits(int drive_index, uint16_t positive_speed_limit_rpm, uint16_t negative_speed_limit_rpm);

esp_err_t dsy_rs485_set_drive_internal_torque_limits(
    int drive_index,
    uint16_t forward_limit_tenths_percent_rated,
    uint16_t backward_limit_tenths_percent_rated);

esp_err_t dsy_rs485_configure_drive_for_internal_digit_speed_mode(
    int drive_index,
    uint16_t forward_speed_limit_rpm,
    uint16_t backward_speed_limit_rpm,
    uint16_t accel_time_ms,
    uint16_t decel_time_ms,
    uint16_t torque_limit_tenths_percent_rated);

esp_err_t dsy_rs485_write_drive_speed_command_rpm(int drive_index, int16_t speed_command_rpm);

/**
 * @brief Runtime speed-command write using the short runtime RS485 policy.
 *
 * Intended only after startup configuration is complete. On ESP_OK the drive
 * acknowledged the P05.03 write. On failure, caller should leave that drive's
 * target marked pending and retry later.
 */
esp_err_t dsy_rs485_runtime_write_drive_speed_command_rpm(int drive_index, int16_t speed_command_rpm);

/**
 * @brief Runtime feedback-pulse read using the short runtime RS485 policy.
 *
 * Reads P18.17/P18.18, the drive feedback pulse counter. Use this for
 * motor-side odometry; P18.07 is the absolute-position counter and can move
 * with the drive's position-domain state rather than raw feedback pulses.
 */
esp_err_t dsy_rs485_runtime_read_drive_feedback_pulse_counts(
    int drive_index,
    int32_t *feedback_pulse_counts_out);

esp_err_t dsy_rs485_runtime_read_drive_motion_sample(
    int drive_index,
    dsy_rs485_runtime_motion_sample_t *sample_out);

/**
 * @brief Runtime single-turn absolute-encoder read using one short transaction.
 *
 * Reads P18.32/P18.33 absolute encoder single-round data. For the M17S
 * single-turn motor encoder, this is the only reliable raw encoder position
 * source. The caller unwraps this modulo count locally and subtracts the
 * first accepted sample to obtain relative motor motion from startup.
 */
esp_err_t dsy_rs485_runtime_read_drive_absolute_encoder_single_turn_counts(
    int drive_index,
    uint32_t *single_turn_counts_out);

/**
 * @brief Backward-compatible absolute-encoder read.
 *
 * For single-turn M17S encoders this returns only the P18.32/P18.33
 * single-turn value. Do not use P18.34 as a motion turn counter on these
 * motors; the firmware odometry path performs local modulo unwrapping.
 */
esp_err_t dsy_rs485_runtime_read_drive_absolute_encoder_motor_counts(
    int drive_index,
    int64_t *absolute_encoder_motor_counts_out);

/**
 * @brief Compatibility wrapper for older callers.
 *
 * This now returns the same P18.17 feedback pulse counter as
 * dsy_rs485_runtime_read_drive_feedback_pulse_counts().
 */
esp_err_t dsy_rs485_runtime_read_drive_absolute_position_counts(
    int drive_index,
    int32_t *absolute_position_counts_out);

esp_err_t dsy_rs485_poll_one_drive(int drive_index);

#ifdef __cplusplus
}
#endif

#endif
