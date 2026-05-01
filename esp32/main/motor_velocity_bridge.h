/**
 * @file motor_velocity_bridge.h
 * @brief micro-ROS direct motor-RPM bridge for DSY-RS drives.
 */

#ifndef MOTOR_VELOCITY_BRIDGE_H
#define MOTOR_VELOCITY_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hexapod_hw_config.h"

typedef enum
{
    motor_bridge_stage_boot = 0,
    motor_bridge_stage_preparing_bus,
    motor_bridge_stage_ready,
    motor_bridge_stage_fault,
} motor_bridge_stage_t;

typedef struct
{
    motor_bridge_stage_t stage;
    bool drives_prepared;
    bool hardware_servo_enabled;
    bool has_received_command;
    uint8_t comm_ok_drive_count;
    uint32_t command_write_count;
    uint32_t odometry_publish_count;
    uint32_t received_command_count;
    bool pending_command_write;
    int32_t last_error_code;
    int16_t latched_motor_rpm[HEXAPOD_MOTOR_COUNT];
    /* Runtime odometry now mirrors raw P18.32/P18.33 single-turn encoder counts. */
    int32_t absolute_position_counts[HEXAPOD_MOTOR_COUNT];
    /* API-compatible field name: carries raw encoder counts, not radians. */
    float output_phase_rad[HEXAPOD_MOTOR_COUNT];
    float battery_voltage_v;
} motor_velocity_bridge_status_t;

esp_err_t motor_velocity_bridge_prepare_drives_before_ros(void);
void motor_velocity_bridge_task(void *argument);
motor_velocity_bridge_status_t get_motor_velocity_bridge_status(void);
const char *get_motor_bridge_stage_short_text(motor_bridge_stage_t stage);

#ifdef __cplusplus
}
#endif

#endif
