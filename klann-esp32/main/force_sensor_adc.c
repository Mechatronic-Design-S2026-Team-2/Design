/**
 * @file force_sensor_adc.c
 * @brief Continuous ADC reader for six force-sensitive resistor channels.
 *
 * Configures the ESP32 ADC continuous driver, averages frame samples for each
 * configured channel, converts them into millivolts, and caches the results
 * for LCD status output and micro-ROS publication.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

#include "force_sensor_adc.h"

/*
 * ESP-IDF refs:
 * - ADC Continuous Mode Driver
 * - ADC Calibration Driver
 * - ESP32 ADC channel / attenuation tables
 */

/* ---------- Force sensor ADC config ---------- */

static const int force_sensor_gpio_numbers[NUM_FORCE_SENSORS] = {
    36, /* sensor 0 -> VP / GPIO36 / ADC1_CH0 */
    39, /* sensor 1 -> VN / GPIO39 / ADC1_CH3 */
    34, /* sensor 2 -> GPIO34 / ADC1_CH6 */
    35, /* sensor 3 -> GPIO35 / ADC1_CH7 */
    32, /* sensor 4 -> GPIO32 / ADC1_CH4 */
    33  /* sensor 5 -> GPIO33 / ADC1_CH5 */
};

/* Highest attenuation. */
#define FORCE_SENSOR_ADC_ATTENUATION                   ADC_ATTEN_DB_12
#define FORCE_SENSOR_ADC_BIT_WIDTH                     ADC_BITWIDTH_12
#define FORCE_SENSOR_ADC_MAX_VOLTAGE_MILLIVOLTS        2450
#define FORCE_SENSOR_ADC_MAX_RAW_COUNT                 ((1U << 12) - 1U)

/* 100 Hz per sensor. */
#define FORCE_SENSOR_SAMPLE_RATE_HZ_PER_SENSOR         4000
#define FORCE_SENSOR_TOTAL_SAMPLE_RATE_HZ              (NUM_FORCE_SENSORS * FORCE_SENSOR_SAMPLE_RATE_HZ_PER_SENSOR)

/* Driver pool / frame sizing. */
#define FORCE_SENSOR_ADC_MAX_STORE_BUFFER_SIZE         1024
#define FORCE_SENSOR_ADC_CONVERSION_FRAME_SIZE         256
#define FORCE_SENSOR_ADC_READ_TIMEOUT_MS               20
#define FORCE_SENSOR_ADC_MAX_SAMPLE_COUNT              (FORCE_SENSOR_ADC_CONVERSION_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES)

/* ---------- Force sensor ADC state ---------- */

static const char *force_sensor_adc_log_tag = "force_sensor_adc";

/* Driver handle. */
static adc_continuous_handle_t force_sensor_adc_handle = NULL;

/* Calibration handle. */
static adc_cali_handle_t force_sensor_adc_calibration_handle = NULL;
static bool force_sensor_adc_calibration_enabled = false;

/* GPIO -> ADC lookup cache. */
static adc_unit_t force_sensor_adc_units[NUM_FORCE_SENSORS];
static adc_channel_t force_sensor_adc_channels[NUM_FORCE_SENSORS];

/* Pattern config cache. */
static adc_digi_pattern_config_t force_sensor_adc_pattern_configurations[NUM_FORCE_SENSORS];

/* Latest per-sensor values. */
static int force_sensor_raw_counts[NUM_FORCE_SENSORS] = {0};
static int force_sensor_voltage_millivolts[NUM_FORCE_SENSORS] = {0};

/* Working buffers. */
static uint8_t force_sensor_adc_raw_buffer[FORCE_SENSOR_ADC_CONVERSION_FRAME_SIZE];

/* ISR -> task handoff. */
static TaskHandle_t force_sensor_adc_task_to_notify = NULL;
static volatile bool force_sensor_adc_pool_overflowed = false;

/**
 * @brief ADC source -> sensor index map.
 *
 * Match ADC unit + channel back to local sensor index.
 *
 * @param adc_unit
 *     ADC peripheral ID.
 *     From parsed ADC sample.
 *     Used for source match.
 * @param adc_channel
 *     ADC channel ID.
 *     From parsed ADC sample.
 *     Used for source match.
 *
 * @return Sensor index on match.
 *     0..NUM_FORCE_SENSORS-1.
 *     Used for array access.
 * @return -1 on no match.
 *     Invalid source.
 *     Used to skip unknown sample.
 */
static int get_force_sensor_index_from_adc_source(adc_unit_t adc_unit, adc_channel_t adc_channel)
{
    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        if ((force_sensor_adc_units[sensor_index] == adc_unit) &&
            (force_sensor_adc_channels[sensor_index] == adc_channel)) {
            return sensor_index;
        }
    }

    return -1;
}

/**
 * @brief Raw sample decode helper.
 *
 * Decode one ADC DMA result into unit, channel, raw count.
 * ESP32 uses TYPE1 output only.
 *
 * @param conversion_output_data
 *     Raw conversion-result pointer.
 *     Cast from DMA byte stream.
 *     Used for field decode.
 * @param adc_unit
 *     Output ADC unit pointer.
 *     Receives decoded unit.
 *     Used by caller for source match.
 * @param adc_channel
 *     Output ADC channel pointer.
 *     Receives decoded channel.
 *     Used by caller for source match.
 * @param raw_count
 *     Output raw-count pointer.
 *     Receives decoded ADC code.
 *     Used by caller for averaging.
 *
 * @return true on valid decode.
 *     Sample fields accepted.
 *     Used for normal accumulation path.
 * @return false on invalid decode.
 *     Sample fields out of range.
 *     Used to skip malformed data.
 */
static bool decode_force_sensor_adc_output_data(
    const adc_digi_output_data_t *conversion_output_data,
    adc_unit_t *adc_unit,
    adc_channel_t *adc_channel,
    uint32_t *raw_count)
{
    if ((conversion_output_data == NULL) ||
        (adc_unit == NULL) ||
        (adc_channel == NULL) ||
        (raw_count == NULL)) {
        return false;
    }

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    uint32_t decoded_channel = conversion_output_data->type1.channel;
    uint32_t decoded_raw_count = conversion_output_data->type1.data;

    /*
     * ESP32 type1 output does not expose ADC unit here.
     * All configured FSR pins are ADC1, so fix unit to ADC_UNIT_1.
     */
    uint32_t decoded_unit = (uint32_t)ADC_UNIT_1;
#else
    uint32_t decoded_channel = conversion_output_data->type2.channel;
    uint32_t decoded_unit = conversion_output_data->type2.unit;
    uint32_t decoded_raw_count = conversion_output_data->type2.data;
#endif

    if (decoded_channel > (uint32_t)ADC_CHANNEL_9) {
        return false;
    }

    if (decoded_unit > (uint32_t)ADC_UNIT_2) {
        return false;
    }

    *adc_unit = (adc_unit_t)decoded_unit;
    *adc_channel = (adc_channel_t)decoded_channel;
    *raw_count = decoded_raw_count;

    return true;
}

/**
 * @brief ADC frame-ready ISR callback.
 *
 * Wake ADC task when one conversion frame is ready.
 *
 * @param handle
 *     ADC continuous driver handle.
 *     Passed by ESP-IDF callback API.
 *     Unused here beyond signature.
 * @param event_data
 *     ADC event payload.
 *     Contains frame-ready metadata.
 *     Unused here; task reads driver pool later.
 * @param user_data
 *     Caller-supplied callback context.
 *     Expected as TaskHandle_t.
 *     Used for ISR task notify.
 *
 * @return true if higher-priority task woke.
 *     Requests immediate context switch.
 * @return false otherwise.
 *     No switch needed.
 */
static bool IRAM_ATTR force_sensor_adc_conversion_done_callback(
    adc_continuous_handle_t handle,
    const adc_continuous_evt_data_t *event_data,
    void *user_data)
{
    (void)handle;
    (void)event_data;

    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;

    if (task_to_notify != NULL) {
        vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    }

    return (higher_priority_task_woken == pdTRUE);
}

/**
 * @brief ADC pool-overflow ISR callback.
 *
 * Latch overflow flag for later log / recovery.
 *
 * @param handle
 *     ADC continuous driver handle.
 *     Passed by ESP-IDF callback API.
 *     Unused here beyond signature.
 * @param event_data
 *     ADC event payload.
 *     Overflow event metadata.
 *     Unused here.
 * @param user_data
 *     Caller-supplied callback context.
 *     Not needed for overflow path.
 *     Unused here.
 *
 * @return false always.
 *     No task wake requested.
 */
static bool IRAM_ATTR force_sensor_adc_pool_overflow_callback(
    adc_continuous_handle_t handle,
    const adc_continuous_evt_data_t *event_data,
    void *user_data)
{
    (void)handle;
    (void)event_data;
    (void)user_data;

    force_sensor_adc_pool_overflowed = true;

    return false;
}

/**
 * @brief Force-sensor ADC init.
 *
 * Create driver, map GPIOs, load patterns, register callbacks,
 * create calibration handle, start continuous conversion.
 *
 * @param task_to_notify
 *     FreeRTOS task handle.
 *     ADC task waiting on frame-ready notify.
 *     Used by ISR callback for handoff.
 *
 * @return None.
 */
static void initialize_force_sensor_adc(TaskHandle_t task_to_notify)
{
    force_sensor_adc_task_to_notify = task_to_notify;

    adc_continuous_handle_cfg_t adc_handle_configuration = {
        .max_store_buf_size = FORCE_SENSOR_ADC_MAX_STORE_BUFFER_SIZE,
        .conv_frame_size = FORCE_SENSOR_ADC_CONVERSION_FRAME_SIZE,
        .flags = {
            .flush_pool = 1,
        },
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(
        &adc_handle_configuration,
        &force_sensor_adc_handle));

    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        ESP_ERROR_CHECK(adc_continuous_io_to_channel(
            force_sensor_gpio_numbers[sensor_index],
            &force_sensor_adc_units[sensor_index],
            &force_sensor_adc_channels[sensor_index]));
    }

    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        force_sensor_adc_pattern_configurations[sensor_index].atten =
            (uint8_t)FORCE_SENSOR_ADC_ATTENUATION;
        force_sensor_adc_pattern_configurations[sensor_index].channel =
            (uint8_t)force_sensor_adc_channels[sensor_index];
        force_sensor_adc_pattern_configurations[sensor_index].unit =
            (uint8_t)force_sensor_adc_units[sensor_index];
        force_sensor_adc_pattern_configurations[sensor_index].bit_width =
            (uint8_t)FORCE_SENSOR_ADC_BIT_WIDTH;
    }

    adc_continuous_config_t adc_continuous_configuration = {
        .pattern_num = NUM_FORCE_SENSORS,
        .adc_pattern = force_sensor_adc_pattern_configurations,
        .sample_freq_hz = FORCE_SENSOR_TOTAL_SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_LOGI(
        force_sensor_adc_log_tag,
        "ADC continuous sample freq request: %d Hz total, %d sensors, %d Hz per sensor",
        FORCE_SENSOR_TOTAL_SAMPLE_RATE_HZ,
        NUM_FORCE_SENSORS,
        FORCE_SENSOR_SAMPLE_RATE_HZ_PER_SENSOR);
    ESP_ERROR_CHECK(adc_continuous_config(
        force_sensor_adc_handle,
        &adc_continuous_configuration));

    adc_continuous_evt_cbs_t adc_event_callbacks = {
        .on_conv_done = force_sensor_adc_conversion_done_callback,
        .on_pool_ovf = force_sensor_adc_pool_overflow_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(
        force_sensor_adc_handle,
        &adc_event_callbacks,
        (void *)force_sensor_adc_task_to_notify));

    adc_cali_line_fitting_config_t calibration_configuration = {
        .unit_id = ADC_UNIT_1,
        .atten = FORCE_SENSOR_ADC_ATTENUATION,
        .bitwidth = FORCE_SENSOR_ADC_BIT_WIDTH,
    };

    esp_err_t calibration_result = adc_cali_create_scheme_line_fitting(
        &calibration_configuration,
        &force_sensor_adc_calibration_handle);

    if (calibration_result == ESP_OK) {
        force_sensor_adc_calibration_enabled = true;
        ESP_LOGI(force_sensor_adc_log_tag, "ADC calibration enabled");
    } else if (calibration_result == ESP_ERR_NOT_SUPPORTED) {
        force_sensor_adc_calibration_enabled = false;
        ESP_LOGW(force_sensor_adc_log_tag, "ADC calibration not supported; using raw scaling");
    } else {
        ESP_ERROR_CHECK(calibration_result);
    }

    ESP_ERROR_CHECK(adc_continuous_start(force_sensor_adc_handle));
}

/**
 * @brief Force-sensor ADC deinit.
 *
 * Stop continuous conversion, free driver handle,
 * delete calibration handle, clear local ownership.
 *
 * @param None.
 *
 * @return None.
 */
static void deinitialize_force_sensor_adc(void)
{
    if (force_sensor_adc_handle != NULL) {
        ESP_ERROR_CHECK(adc_continuous_stop(force_sensor_adc_handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(force_sensor_adc_handle));
        force_sensor_adc_handle = NULL;
    }

    if (force_sensor_adc_calibration_handle != NULL) {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(
            force_sensor_adc_calibration_handle));
        force_sensor_adc_calibration_handle = NULL;
        force_sensor_adc_calibration_enabled = false;
    }
}

/**
 * @brief Read and process one ADC frame.
 *
 * Pull one frame from driver pool, parse samples,
 * average per sensor, update cached raw counts and mV.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     Frame read and cache updated.
 *     Normal path.
 * @return ESP_ERR_TIMEOUT on no data.
 *     No frame available before timeout.
 *     Caller may ignore / retry.
 * @return Other esp_err_t via ESP_ERROR_CHECK.
 *     Driver / read / calibration failure.
 *     Fatal path in current design.
 */
static esp_err_t read_force_sensor_adc_samples(void)
{
    uint32_t bytes_read = 0;

    int raw_count_sums[NUM_FORCE_SENSORS] = {0};
    int raw_count_sample_counts[NUM_FORCE_SENSORS] = {0};

    memset(force_sensor_adc_raw_buffer, 0, sizeof(force_sensor_adc_raw_buffer));

    esp_err_t read_result = adc_continuous_read(
        force_sensor_adc_handle,
        force_sensor_adc_raw_buffer,
        sizeof(force_sensor_adc_raw_buffer),
        &bytes_read,
        FORCE_SENSOR_ADC_READ_TIMEOUT_MS);

    if (read_result == ESP_ERR_TIMEOUT) {
        return read_result;
    }

    ESP_ERROR_CHECK(read_result);

    if ((bytes_read % SOC_ADC_DIGI_RESULT_BYTES) != 0U) {
        ESP_LOGW(force_sensor_adc_log_tag, "ADC frame size misaligned: %lu", (unsigned long)bytes_read);
    }

    uint32_t conversion_result_count = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
    if (conversion_result_count > FORCE_SENSOR_ADC_MAX_SAMPLE_COUNT) {
        conversion_result_count = FORCE_SENSOR_ADC_MAX_SAMPLE_COUNT;
    }

    adc_digi_output_data_t *conversion_output_data_array =
        (adc_digi_output_data_t *)force_sensor_adc_raw_buffer;

    for (uint32_t sample_index = 0; sample_index < conversion_result_count; sample_index++) {
        adc_unit_t sample_adc_unit = ADC_UNIT_1;
        adc_channel_t sample_adc_channel = ADC_CHANNEL_0;
        uint32_t sample_raw_count = 0;

        bool sample_valid = decode_force_sensor_adc_output_data(
            &conversion_output_data_array[sample_index],
            &sample_adc_unit,
            &sample_adc_channel,
            &sample_raw_count);

        if (!sample_valid) {
            continue;
        }

        int sensor_index = get_force_sensor_index_from_adc_source(
            sample_adc_unit,
            sample_adc_channel);

        if (sensor_index >= 0) {
            raw_count_sums[sensor_index] += (int)sample_raw_count;
            raw_count_sample_counts[sensor_index] += 1;
        }
    }

    for (int sensor_index = 0; sensor_index < NUM_FORCE_SENSORS; sensor_index++) {
        if (raw_count_sample_counts[sensor_index] <= 0) {
            continue;
        }

        force_sensor_raw_counts[sensor_index] =
            raw_count_sums[sensor_index] / raw_count_sample_counts[sensor_index];

        if (force_sensor_adc_calibration_enabled) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
                force_sensor_adc_calibration_handle,
                force_sensor_raw_counts[sensor_index],
                &force_sensor_voltage_millivolts[sensor_index]));
        } else {
            force_sensor_voltage_millivolts[sensor_index] =
                (force_sensor_raw_counts[sensor_index] * FORCE_SENSOR_ADC_MAX_VOLTAGE_MILLIVOLTS) /
                FORCE_SENSOR_ADC_MAX_RAW_COUNT;
        }
    }

    if (force_sensor_adc_pool_overflowed) {
        force_sensor_adc_pool_overflowed = false;
        ESP_LOGW(force_sensor_adc_log_tag, "ADC pool overflow");
    }

    return ESP_OK;
}

/**
 * @brief Get cached raw count for one sensor.
 *
 * Return latest averaged raw ADC code for indexed sensor.
 *
 * @param sensor_index
 *     Local sensor array index.
 *     Expected range 0..NUM_FORCE_SENSORS-1.
 *     Used for bounds check and lookup.
 *
 * @return Cached raw ADC count on valid index.
 *     Unitless ADC code.
 *     Used for debug / calibration / thresholding.
 * @return 0 on invalid index.
 *     Bounds-fail fallback.
 *     Prevents bad array access.
 */
int get_force_sensor_raw_count(int sensor_index)
{
    if ((sensor_index < 0) || (sensor_index >= NUM_FORCE_SENSORS)) {
        return 0;
    }

    return force_sensor_raw_counts[sensor_index];
}

/**
 * @brief Get cached voltage for one sensor.
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
 *     Used for control / telemetry / thresholding.
 * @return 0 on invalid index.
 *     Bounds-fail fallback.
 *     Prevents bad array access.
 */
int get_force_sensor_voltage_millivolts(int sensor_index)
{
    if ((sensor_index < 0) || (sensor_index >= NUM_FORCE_SENSORS)) {
        return 0;
    }

    return force_sensor_voltage_millivolts[sensor_index];
}

/**
 * @brief ADC task main loop.
 *
 * Initialize ADC subsystem, wait for frame-ready notify,
 * drain frames, keep latest sensor cache fresh.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config pass-in.
 *
 * @return None.
 *     Task intended to run forever.
 *     Cleanup path only reached on loop exit.
 */
void force_sensor_adc_task(void *argument)
{
    (void)argument;

    initialize_force_sensor_adc(xTaskGetCurrentTaskHandle());

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (read_force_sensor_adc_samples() == ESP_OK) {
            /* Cache updated. */
        }
    }

    deinitialize_force_sensor_adc();
    vTaskDelete(NULL);
}
