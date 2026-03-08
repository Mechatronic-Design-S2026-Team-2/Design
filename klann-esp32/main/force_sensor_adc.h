/**
 * @file force_sensor_adc.h
 * @brief Public task API and cached-value getters for the FSR ADC module.
 */

#ifndef FORCE_SENSOR_ADC_H
#define FORCE_SENSOR_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Public constants ---------- */

#define NUM_FORCE_SENSORS 6

/* ---------- Public task API ---------- */

/**
 * @brief Force-sensor ADC task entry.
 *
 * Initialize ADC subsystem, wait for frame-ready notify,
 * read and cache latest force-sensor measurements.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config pass-in.
 *
 * @return None.
 *     Task intended to run forever.
 */
void force_sensor_adc_task(void *argument);

/* ---------- Public data getters ---------- */

/**
 * @brief Cached raw-count getter.
 *
 * Return latest averaged raw ADC count for one sensor.
 *
 * @param sensor_index
 *     Local sensor array index.
 *     Expected range 0..NUM_FORCE_SENSORS-1.
 *     Used for bounds check and lookup.
 *
 * @return Cached raw ADC count on valid index.
 *     Unitless ADC code.
 *     Used for debug / thresholding / calibration.
 * @return 0 on invalid index.
 *     Bounds-fail fallback.
 *     Prevents bad array access.
 */
int get_force_sensor_raw_count(int sensor_index);

/**
 * @brief Cached voltage getter.
 *
 * Return latest averaged sensor voltage in millivolts.
 *
 * @param sensor_index
 *     Local sensor array index.
 *     Expected range 0..NUM_FORCE_SENSORS-1.
 *     Used for bounds check and lookup.
 *
 * @return Cached sensor voltage on valid index.
 *     Units: millivolts.
 *     Used for control / telemetry / status.
 * @return 0 on invalid index.
 *     Bounds-fail fallback.
 *     Prevents bad array access.
 */
int get_force_sensor_voltage_millivolts(int sensor_index);

#ifdef __cplusplus
}
#endif

#endif
