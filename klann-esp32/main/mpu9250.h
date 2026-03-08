/**
 * @file mpu9250.h
 * @brief Public task API and cached-sample type for the IMU module.
 */

#ifndef MPU9250_H
#define MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------- Public measurement type ---------- */

typedef struct
{
    int16_t accelerometer_x_raw_counts;
    int16_t accelerometer_y_raw_counts;
    int16_t accelerometer_z_raw_counts;

    int16_t gyroscope_x_raw_counts;
    int16_t gyroscope_y_raw_counts;
    int16_t gyroscope_z_raw_counts;

    int16_t temperature_raw_counts;

    float accelerometer_x_meters_per_second_squared;
    float accelerometer_y_meters_per_second_squared;
    float accelerometer_z_meters_per_second_squared;

    float gyroscope_x_radians_per_second;
    float gyroscope_y_radians_per_second;
    float gyroscope_z_radians_per_second;

    float temperature_degrees_celsius;
} mpu9250_measurement_t;

/* ---------- Public task API ---------- */

/**
 * @brief MPU-9250 task entry.
 *
 * Initialize IMU subsystem, wait on INT notify,
 * read accel / gyro / temp frames, cache latest sample.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config pass-in.
 *
 * @return None.
 *     Task intended to run forever.
 */
void mpu9250_task(void *argument);

/* ---------- Public data getters ---------- */

/**
 * @brief Cached IMU sample getter.
 *
 * Return latest cached measurement struct by value.
 *
 * @param None.
 *
 * @return Latest cached measurement.
 *     Raw + scaled accel / gyro / temp.
 *     Used for control / telemetry / status.
 */
mpu9250_measurement_t get_latest_mpu9250_measurement(void);

#ifdef __cplusplus
}
#endif

#endif
