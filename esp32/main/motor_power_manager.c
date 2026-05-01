/**
 * @file motor_power_manager.c
 * @brief Precharge / contactor SSR control and main-bus ADC monitor.
 */

#include "motor_power_manager.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexapod_hw_config.h"

#ifndef CONFIG_POWER_PRECHARGE_USE_ADC_GATE
#define CONFIG_POWER_PRECHARGE_USE_ADC_GATE 1
#endif

#ifndef CONFIG_POWER_PRECHARGE_REQUIRE_ADC
#define CONFIG_POWER_PRECHARGE_REQUIRE_ADC 1
#endif

#ifndef CONFIG_POWER_PRECHARGE_MIN_DWELL_US
#ifdef CONFIG_POWER_PRECHARGE_MIN_DWELL_MS
#define CONFIG_POWER_PRECHARGE_MIN_DWELL_US ((uint32_t)CONFIG_POWER_PRECHARGE_MIN_DWELL_MS * 1000U)
#else
#define CONFIG_POWER_PRECHARGE_MIN_DWELL_US 500000
#endif
#endif

#ifndef CONFIG_POWER_PRECHARGE_POLL_PERIOD_US
#ifdef CONFIG_POWER_PRECHARGE_POLL_PERIOD_MS
#define CONFIG_POWER_PRECHARGE_POLL_PERIOD_US ((uint32_t)CONFIG_POWER_PRECHARGE_POLL_PERIOD_MS * 1000U)
#else
#define CONFIG_POWER_PRECHARGE_POLL_PERIOD_US 50000
#endif
#endif

#ifndef CONFIG_POWER_PRECHARGE_STABLE_SAMPLE_COUNT
#define CONFIG_POWER_PRECHARGE_STABLE_SAMPLE_COUNT 3
#endif

#ifndef CONFIG_POWER_PRECHARGE_TIME_US
#ifdef CONFIG_POWER_PRECHARGE_TIME_MS
#define CONFIG_POWER_PRECHARGE_TIME_US ((uint32_t)CONFIG_POWER_PRECHARGE_TIME_MS * 1000U)
#else
#define CONFIG_POWER_PRECHARGE_TIME_US 20000000
#endif
#endif

#ifndef CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_US
#ifdef CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_MS
#define CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_US ((uint32_t)CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_MS * 1000U)
#else
#define CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_US CONFIG_POWER_PRECHARGE_TIME_US
#endif
#endif

#ifndef CONFIG_POWER_CONTACTOR_SETTLE_US
#ifdef CONFIG_POWER_CONTACTOR_SETTLE_MS
#define CONFIG_POWER_CONTACTOR_SETTLE_US ((uint32_t)CONFIG_POWER_CONTACTOR_SETTLE_MS * 1000U)
#else
#define CONFIG_POWER_CONTACTOR_SETTLE_US 250000
#endif
#endif

#define MOTOR_POWER_MANAGER_TASK_PERIOD_US   100000
#define MOTOR_POWER_MANAGER_ADC_ATTEN        ADC_ATTEN_DB_12
#define MOTOR_POWER_MANAGER_ADC_BITWIDTH     ADC_BITWIDTH_12

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


static const char *motor_power_manager_log_tag = "motor_power";

static motor_power_manager_status_t motor_power_manager_status = {
    .adc_divider_ratio = 1.0f,
    .precharge_threshold_v = 0.001f * (float)HEXAPOD_PRECHARGE_COMPLETE_MILLIVOLTS,
};
static adc_oneshot_unit_handle_t motor_power_manager_adc_handle = NULL;
static adc_cali_handle_t motor_power_manager_adc_cali_handle = NULL;
static bool motor_power_manager_adc_cali_enabled = false;
static adc_unit_t motor_power_manager_adc_unit = ADC_UNIT_1;
static adc_channel_t motor_power_manager_adc_channel = ADC_CHANNEL_0;

static float get_precharge_adc_divider_ratio(void)
{
    const float divider_high_ohms = (float)HEXAPOD_PRECHARGE_DIVIDER_HIGH_OHMS;
    const float divider_low_ohms = (float)HEXAPOD_PRECHARGE_DIVIDER_LOW_OHMS;

    if (divider_low_ohms <= 0.0f) {
        return 1.0f;
    }

    return (divider_high_ohms + divider_low_ohms) / divider_low_ohms;
}

static void set_ssr_gpio_level(gpio_num_t gpio_num, bool enable)
{
    gpio_set_level(
        gpio_num,
        enable ? HEXAPOD_POWER_SSR_ACTIVE_LEVEL : !HEXAPOD_POWER_SSR_ACTIVE_LEVEL);
}

esp_err_t motor_power_manager_set_precharge_enabled(bool enable)
{
    if (!motor_power_manager_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    set_ssr_gpio_level(HEXAPOD_PRECHARGE_SSR_GPIO, enable);
    motor_power_manager_status.precharge_enabled = enable;
    return ESP_OK;
}

esp_err_t motor_power_manager_set_contactor_enabled(bool enable)
{
    if (!motor_power_manager_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    set_ssr_gpio_level(HEXAPOD_CONTACTOR_SSR_GPIO, enable);
    motor_power_manager_status.contactor_enabled = enable;
    return ESP_OK;
}

static void sample_main_bus_voltage_once(void)
{
    motor_power_manager_status.adc_divider_ratio = get_precharge_adc_divider_ratio();
    motor_power_manager_status.precharge_threshold_v =
        0.001f * (float)HEXAPOD_PRECHARGE_COMPLETE_MILLIVOLTS;

    if (motor_power_manager_adc_handle == NULL) {
        motor_power_manager_status.adc_ready = false;
        motor_power_manager_status.latest_raw_adc_count = 0;
        motor_power_manager_status.latest_sensor_millivolts = 0;
        motor_power_manager_status.latest_bus_voltage_v = 0.0f;
        return;
    }

    int raw_count = 0;
    esp_err_t read_result = adc_oneshot_read(
        motor_power_manager_adc_handle,
        motor_power_manager_adc_channel,
        &raw_count);
    if (read_result != ESP_OK) {
        motor_power_manager_status.adc_ready = false;
        return;
    }

    int sensed_millivolts = 0;
    if (motor_power_manager_adc_cali_enabled) {
        if (adc_cali_raw_to_voltage(
                motor_power_manager_adc_cali_handle,
                raw_count,
                &sensed_millivolts) != ESP_OK) {
            sensed_millivolts = (raw_count * 3300) / 4095;
        }
    } else {
        sensed_millivolts = (raw_count * 3300) / 4095;
    }

    motor_power_manager_status.latest_raw_adc_count = raw_count;
    motor_power_manager_status.latest_sensor_millivolts = sensed_millivolts;
    motor_power_manager_status.latest_bus_voltage_v =
        ((float)sensed_millivolts) * get_precharge_adc_divider_ratio() / 1000.0f;
    motor_power_manager_status.adc_ready = true;
}

static esp_err_t initialize_main_bus_adc(void)
{
    adc_unit_t detected_unit = ADC_UNIT_1;
    adc_channel_t detected_channel = ADC_CHANNEL_0;

    esp_err_t map_result = adc_oneshot_io_to_channel(
        (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
        &detected_unit,
        &detected_channel);
    if (map_result != ESP_OK) {
        ESP_LOGW(
            motor_power_manager_log_tag,
            "precharge ADC GPIO%d is not ADC-capable: %s",
            (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
            esp_err_to_name(map_result));
        motor_power_manager_status.adc_ready = false;
        return map_result;
    }

    motor_power_manager_adc_unit = detected_unit;
    motor_power_manager_adc_channel = detected_channel;

    adc_oneshot_unit_init_cfg_t adc_unit_configuration = {
        .unit_id = motor_power_manager_adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t adc_init_result = adc_oneshot_new_unit(
        &adc_unit_configuration,
        &motor_power_manager_adc_handle);

    if (adc_init_result != ESP_OK) {
        ESP_LOGW(
            motor_power_manager_log_tag,
            "main-bus ADC init failed on GPIO%d unit=%d channel=%d: %s",
            (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
            (int)motor_power_manager_adc_unit,
            (int)motor_power_manager_adc_channel,
            esp_err_to_name(adc_init_result));
        motor_power_manager_status.adc_ready = false;
        return adc_init_result;
    }

    adc_oneshot_chan_cfg_t adc_channel_configuration = {
        .atten = MOTOR_POWER_MANAGER_ADC_ATTEN,
        .bitwidth = MOTOR_POWER_MANAGER_ADC_BITWIDTH,
    };

    esp_err_t channel_result = adc_oneshot_config_channel(
        motor_power_manager_adc_handle,
        motor_power_manager_adc_channel,
        &adc_channel_configuration);
    if (channel_result != ESP_OK) {
        ESP_LOGW(
            motor_power_manager_log_tag,
            "main-bus ADC channel config failed on GPIO%d unit=%d channel=%d: %s",
            (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
            (int)motor_power_manager_adc_unit,
            (int)motor_power_manager_adc_channel,
            esp_err_to_name(channel_result));
        motor_power_manager_status.adc_ready = false;
        return channel_result;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_configuration = {
        .unit_id = motor_power_manager_adc_unit,
        .chan = motor_power_manager_adc_channel,
        .atten = MOTOR_POWER_MANAGER_ADC_ATTEN,
        .bitwidth = MOTOR_POWER_MANAGER_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(
            &cali_configuration,
            &motor_power_manager_adc_cali_handle) == ESP_OK) {
        motor_power_manager_adc_cali_enabled = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_configuration = {
        .unit_id = motor_power_manager_adc_unit,
        .atten = MOTOR_POWER_MANAGER_ADC_ATTEN,
        .bitwidth = MOTOR_POWER_MANAGER_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(
            &cali_configuration,
            &motor_power_manager_adc_cali_handle) == ESP_OK) {
        motor_power_manager_adc_cali_enabled = true;
    }
#endif

    return ESP_OK;
}

esp_err_t motor_power_manager_initialize(void)
{
    motor_power_manager_status.adc_divider_ratio = get_precharge_adc_divider_ratio();
    motor_power_manager_status.precharge_threshold_v =
        0.001f * (float)HEXAPOD_PRECHARGE_COMPLETE_MILLIVOLTS;

    gpio_config_t ssr_gpio_configuration = {
        .pin_bit_mask = (1ULL << HEXAPOD_PRECHARGE_SSR_GPIO) |
                        (1ULL << HEXAPOD_CONTACTOR_SSR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&ssr_gpio_configuration),
        motor_power_manager_log_tag,
        "SSR gpio_config failed");

    set_ssr_gpio_level(HEXAPOD_PRECHARGE_SSR_GPIO, false);
    set_ssr_gpio_level(HEXAPOD_CONTACTOR_SSR_GPIO, false);

    esp_err_t adc_init_result = initialize_main_bus_adc();
    if (adc_init_result != ESP_OK) {
        ESP_LOGW(
            motor_power_manager_log_tag,
            "continuing with ADC unavailable; ADC-required precharge will fail closed-safe");
    }

    motor_power_manager_status.initialized = true;
    sample_main_bus_voltage_once();

    ESP_LOGI(
        motor_power_manager_log_tag,
        "power manager ready precharge_gpio=%d contactor_gpio=%d adc_gpio=%d adc_unit=%d adc_channel=%d divider=%.4f threshold=%.2fV adc_ready=%d cal=%d",
        (int)HEXAPOD_PRECHARGE_SSR_GPIO,
        (int)HEXAPOD_CONTACTOR_SSR_GPIO,
        (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
        (int)motor_power_manager_adc_unit,
        (int)motor_power_manager_adc_channel,
        motor_power_manager_status.adc_divider_ratio,
        motor_power_manager_status.precharge_threshold_v,
        motor_power_manager_status.adc_ready ? 1 : 0,
        motor_power_manager_adc_cali_enabled ? 1 : 0);

    return ESP_OK;
}

static esp_err_t wait_for_adc_precharge_complete(void)
{
    if (!motor_power_manager_status.adc_ready) {
#if CONFIG_POWER_PRECHARGE_REQUIRE_ADC
        ESP_LOGE(
            motor_power_manager_log_tag,
            "precharge ADC required but unavailable; refusing to close contactor");
        return ESP_ERR_INVALID_STATE;
#else
        ESP_LOGW(
            motor_power_manager_log_tag,
            "precharge ADC unavailable; falling back to fixed precharge delay %d us",
            (int)CONFIG_POWER_PRECHARGE_TIME_US);
        delay_microseconds(CONFIG_POWER_PRECHARGE_TIME_US);
        motor_power_manager_status.precharge_complete = true;
        motor_power_manager_status.precharge_elapsed_us = CONFIG_POWER_PRECHARGE_TIME_US;
        return ESP_OK;
#endif
    }

    const uint32_t poll_period_us =
        (CONFIG_POWER_PRECHARGE_POLL_PERIOD_US > 0) ? CONFIG_POWER_PRECHARGE_POLL_PERIOD_US : 1000U;
    const uint32_t timeout_us = CONFIG_POWER_PRECHARGE_ADC_TIMEOUT_US;
    const uint32_t min_dwell_us = CONFIG_POWER_PRECHARGE_MIN_DWELL_US;
    const int required_stable_samples =
        (CONFIG_POWER_PRECHARGE_STABLE_SAMPLE_COUNT > 0) ? CONFIG_POWER_PRECHARGE_STABLE_SAMPLE_COUNT : 1;
    const float threshold_v = 0.001f * (float)HEXAPOD_PRECHARGE_COMPLETE_MILLIVOLTS;

    uint32_t elapsed_us = 0U;
    uint32_t next_log_us = 0U;
    int stable_samples = 0;

    ESP_LOGI(
        motor_power_manager_log_tag,
        "precharge ADC gate start threshold=%.2fV adc_timeout=%uus min_dwell=%uus poll=%uus stable_samples=%d divider=%.4f",
        threshold_v,
        (unsigned)timeout_us,
        (unsigned)min_dwell_us,
        (unsigned)poll_period_us,
        required_stable_samples,
        get_precharge_adc_divider_ratio());

    while (elapsed_us <= timeout_us) {
        sample_main_bus_voltage_once();

        if (!motor_power_manager_status.adc_ready) {
            ESP_LOGE(
                motor_power_manager_log_tag,
                "precharge ADC read failed during precharge; refusing to close contactor");
            return ESP_ERR_INVALID_STATE;
        }

        if (elapsed_us >= min_dwell_us) {
            if (motor_power_manager_status.latest_bus_voltage_v >= threshold_v) {
                stable_samples++;
                if (stable_samples >= required_stable_samples) {
                    motor_power_manager_status.precharge_complete = true;
                    motor_power_manager_status.precharge_elapsed_us = elapsed_us;
                    ESP_LOGI(
                        motor_power_manager_log_tag,
                        "precharge complete %.2fV sensor=%dmV raw=%d elapsed=%uus stable=%d/%d",
                        motor_power_manager_status.latest_bus_voltage_v,
                        motor_power_manager_status.latest_sensor_millivolts,
                        motor_power_manager_status.latest_raw_adc_count,
                        (unsigned)elapsed_us,
                        stable_samples,
                        required_stable_samples);
                    return ESP_OK;
                }
            } else {
                stable_samples = 0;
            }
        }

        if (elapsed_us >= next_log_us) {
            ESP_LOGI(
                motor_power_manager_log_tag,
                "precharge wait %.2fV sensor=%dmV raw=%d elapsed=%uus stable=%d/%d",
                motor_power_manager_status.latest_bus_voltage_v,
                motor_power_manager_status.latest_sensor_millivolts,
                motor_power_manager_status.latest_raw_adc_count,
                (unsigned)elapsed_us,
                stable_samples,
                required_stable_samples);
            next_log_us = elapsed_us + 500000U;
        }

        delay_microseconds(poll_period_us);
        elapsed_us += poll_period_us;
        motor_power_manager_status.precharge_elapsed_us = elapsed_us;
    }

    ESP_LOGE(
        motor_power_manager_log_tag,
        "precharge timeout %.2fV < %.2fV after %uus; refusing to close contactor",
        motor_power_manager_status.latest_bus_voltage_v,
        threshold_v,
        (unsigned)timeout_us);
    return ESP_ERR_TIMEOUT;
}

esp_err_t motor_power_manager_run_startup_sequence(void)
{
    if (!motor_power_manager_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    motor_power_manager_status.precharge_complete = false;
    motor_power_manager_status.precharge_elapsed_us = 0U;

    ESP_RETURN_ON_ERROR(
        motor_power_manager_set_contactor_enabled(false),
        motor_power_manager_log_tag,
        "disable contactor before precharge failed");

    ESP_RETURN_ON_ERROR(
        motor_power_manager_set_precharge_enabled(true),
        motor_power_manager_log_tag,
        "enable precharge failed");

#if CONFIG_POWER_PRECHARGE_USE_ADC_GATE
    esp_err_t precharge_result = wait_for_adc_precharge_complete();
#else
    ESP_LOGI(
        motor_power_manager_log_tag,
        "fixed precharge delay %d us because ADC gate is disabled",
        (int)CONFIG_POWER_PRECHARGE_TIME_US);
    delay_microseconds(CONFIG_POWER_PRECHARGE_TIME_US);
    motor_power_manager_status.precharge_complete = true;
    motor_power_manager_status.precharge_elapsed_us = CONFIG_POWER_PRECHARGE_TIME_US;
    esp_err_t precharge_result = ESP_OK;
#endif

    if (precharge_result != ESP_OK) {
        (void)motor_power_manager_set_precharge_enabled(false);
        (void)motor_power_manager_set_contactor_enabled(false);
        return precharge_result;
    }

    ESP_LOGI(
        motor_power_manager_log_tag,
        "closing contactor at measured bus %.2fV",
        motor_power_manager_status.latest_bus_voltage_v);

    ESP_RETURN_ON_ERROR(
        motor_power_manager_set_contactor_enabled(true),
        motor_power_manager_log_tag,
        "enable contactor failed");

    delay_microseconds(CONFIG_POWER_CONTACTOR_SETTLE_US);

    ESP_RETURN_ON_ERROR(
        motor_power_manager_set_precharge_enabled(false),
        motor_power_manager_log_tag,
        "disable precharge failed");

    sample_main_bus_voltage_once();
    ESP_LOGI(
        motor_power_manager_log_tag,
        "power path closed measured bus %.2fV sensor=%dmV raw=%d",
        motor_power_manager_status.latest_bus_voltage_v,
        motor_power_manager_status.latest_sensor_millivolts,
        motor_power_manager_status.latest_raw_adc_count);

    return ESP_OK;
}

motor_power_manager_status_t get_motor_power_manager_status(void)
{
    return motor_power_manager_status;
}

void motor_power_manager_task(void *argument)
{
    (void)argument;

    while (true) {
        sample_main_bus_voltage_once();
        delay_microseconds(MOTOR_POWER_MANAGER_TASK_PERIOD_US);
    }
}
