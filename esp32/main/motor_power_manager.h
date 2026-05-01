/**
 * @file motor_power_manager.h
 * @brief Precharge / contactor SSR control and main-bus ADC monitor.
 */

#ifndef MOTOR_POWER_MANAGER_H
#define MOTOR_POWER_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct
{
    bool initialized;
    bool precharge_enabled;
    bool contactor_enabled;
    bool adc_ready;
    bool precharge_complete;
    int latest_raw_adc_count;
    int latest_sensor_millivolts;
    float latest_bus_voltage_v;
    float adc_divider_ratio;
    float precharge_threshold_v;
    uint32_t precharge_elapsed_us;
} motor_power_manager_status_t;

esp_err_t motor_power_manager_initialize(void);
void motor_power_manager_task(void *argument);
esp_err_t motor_power_manager_run_startup_sequence(void);
esp_err_t motor_power_manager_set_precharge_enabled(bool enable);
esp_err_t motor_power_manager_set_contactor_enabled(bool enable);
motor_power_manager_status_t get_motor_power_manager_status(void);

#ifdef __cplusplus
}
#endif

#endif
