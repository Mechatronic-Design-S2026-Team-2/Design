/**
 * @file mpu9250.c
 * @brief Legacy-I2C MPU-9250 / MPU-6500 accel-gyro driver.
 *
 * Brings up the IMU over the ESP-IDF legacy I2C driver, waits on the
 * data-ready interrupt, converts raw samples into SI units, and caches the
 * latest measurement for telemetry and status output.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mpu9250.h"

/*
 * ESP-IDF refs:
 * - Legacy I2C master driver: driver/i2c.h
 * - GPIO driver: driver/gpio.h
 *
 * Device note:
 * - MPU-9250 WHO_AM_I = 0x71
 * - MPU-6500 WHO_AM_I = 0x70
 * Current driver uses accel + gyro + temp only.
 * So both IDs are acceptable here.
 */

/* ---------- MPU / bus config ---------- */

#define MPU9250_I2C_PORT_NUMBER                         I2C_NUM_0
#define MPU9250_I2C_SCL_GPIO_NUM                        21
#define MPU9250_I2C_SDA_GPIO_NUM                        22
#define MPU9250_INTERRUPT_GPIO_NUM                      19

/* AD0 low -> 0x68. */
#define MPU9250_I2C_DEVICE_ADDRESS                      0x68

#define MPU9250_I2C_BUS_SPEED_HZ                        400000
#define MPU9250_I2C_TRANSFER_TIMEOUT_MS                 100
#define MPU9250_I2C_TRANSFER_TIMEOUT_TICKS              pdMS_TO_TICKS(MPU9250_I2C_TRANSFER_TIMEOUT_MS)

/* ---------- register map ---------- */

#define MPU9250_REGISTER_SMPLRT_DIV                     0x19
#define MPU9250_REGISTER_CONFIG                         0x1A
#define MPU9250_REGISTER_GYRO_CONFIG                    0x1B
#define MPU9250_REGISTER_ACCEL_CONFIG                   0x1C
#define MPU9250_REGISTER_ACCEL_CONFIG_2                 0x1D
#define MPU9250_REGISTER_INT_PIN_CFG                    0x37
#define MPU9250_REGISTER_INT_ENABLE                     0x38
#define MPU9250_REGISTER_INT_STATUS                     0x3A
#define MPU9250_REGISTER_ACCEL_XOUT_H                   0x3B
#define MPU9250_REGISTER_PWR_MGMT_1                     0x6B
#define MPU9250_REGISTER_PWR_MGMT_2                     0x6C
#define MPU9250_REGISTER_WHO_AM_I                       0x75

/* ---------- WHO_AM_I values ---------- */

#define MPU9250_WHO_AM_I_VALUE_MPU6500                  0x70
#define MPU9250_WHO_AM_I_VALUE_MPU9250                  0x71

/* ---------- startup values ---------- */

#define MPU9250_PWR_MGMT_1_STARTUP_VALUE                0x01
#define MPU9250_PWR_MGMT_2_STARTUP_VALUE                0x00
#define MPU9250_CONFIG_STARTUP_VALUE                    0x03
#define MPU9250_GYRO_CONFIG_STARTUP_VALUE               0x00
#define MPU9250_ACCEL_CONFIG_STARTUP_VALUE              0x00
#define MPU9250_ACCEL_CONFIG_2_STARTUP_VALUE            0x03
#define MPU9250_SMPLRT_DIV_STARTUP_VALUE                0x09
#define MPU9250_INT_PIN_CFG_STARTUP_VALUE               0x00
#define MPU9250_INT_ENABLE_STARTUP_VALUE                0x01

/* ---------- scale factors ---------- */

#define MPU9250_ACCELEROMETER_LSB_PER_G                 16384.0f
#define MPU9250_GYROSCOPE_LSB_PER_DEGREE_PER_SECOND     131.0f
#define MPU9250_TEMPERATURE_LSB_PER_DEGREE_C            333.87f
#define MPU9250_TEMPERATURE_OFFSET_DEGREE_C             21.0f
#define STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED      9.80665f
#define DEGREES_TO_RADIANS                              0.01745329251994329577f

/* ---------- local types ---------- */

typedef enum {
    detected_mpu_device_unknown = 0,
    detected_mpu_device_mpu6500,
    detected_mpu_device_mpu9250,
} detected_mpu_device_t;

/* ---------- local state ---------- */

static const char *mpu9250_log_tag = "mpu9250_imu";

static bool mpu9250_i2c_driver_installed = false;
static bool mpu9250_interrupt_gpio_ready = false;

static TaskHandle_t mpu9250_task_to_notify = NULL;
static volatile bool mpu9250_interrupt_received = false;

static detected_mpu_device_t detected_mpu_device = detected_mpu_device_unknown;
static uint8_t detected_mpu_who_am_i_value = 0x00;

static mpu9250_measurement_t mpu9250_latest_measurement = {0};

/**
 * @brief Two bytes -> signed 16-bit sample.
 *
 * Join big-endian high / low bytes into int16_t.
 *
 * @param high_byte
 *     Register high byte.
 *     Bits [15:8] of sample.
 *     Used for signed compose.
 * @param low_byte
 *     Register low byte.
 *     Bits [7:0] of sample.
 *     Used for signed compose.
 *
 * @return Signed 16-bit sample.
 *     Raw sensor counts.
 *     Used for scale convert.
 */
static int16_t combine_bytes_into_signed_int16(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8) | (uint16_t)low_byte);
}

/**
 * @brief One-register write.
 *
 * Write one byte to one MPU register.
 *
 * @param register_address
 *     Target register address.
 *     One-byte selector.
 *     Used as write target.
 * @param register_value
 *     Payload byte.
 *     One-byte register value.
 *     Used for config write.
 *
 * @return ESP_OK on success.
 *     I2C write complete.
 *     Normal path.
 * @return Other esp_err_t on failure.
 *     Transport / device error.
 *     Caller should abort op.
 */
static esp_err_t write_mpu9250_register(uint8_t register_address, uint8_t register_value)
{
    uint8_t transmit_buffer[2] = {
        register_address,
        register_value
    };

    return i2c_master_write_to_device(
        MPU9250_I2C_PORT_NUMBER,
        MPU9250_I2C_DEVICE_ADDRESS,
        transmit_buffer,
        sizeof(transmit_buffer),
        MPU9250_I2C_TRANSFER_TIMEOUT_TICKS);
}

/**
 * @brief One-register read.
 *
 * Read one byte from one MPU register.
 *
 * @param register_address
 *     Source register address.
 *     One-byte selector.
 *     Used as read start.
 * @param register_value
 *     Destination byte pointer.
 *     Receives register contents.
 *     Used by caller.
 *
 * @return ESP_OK on success.
 *     I2C read complete.
 *     Normal path.
 * @return Other esp_err_t on failure.
 *     Transport / device error.
 *     Caller should abort op.
 */
static esp_err_t read_mpu9250_register(uint8_t register_address, uint8_t *register_value)
{
    return i2c_master_write_read_device(
        MPU9250_I2C_PORT_NUMBER,
        MPU9250_I2C_DEVICE_ADDRESS,
        &register_address,
        sizeof(register_address),
        register_value,
        1,
        MPU9250_I2C_TRANSFER_TIMEOUT_TICKS);
}

/**
 * @brief Multi-register burst read.
 *
 * Read contiguous register block from MPU.
 *
 * @param start_register_address
 *     First register address.
 *     One-byte selector.
 *     Used as burst start.
 * @param read_buffer
 *     Destination byte buffer.
 *     Receives read payload.
 *     Used for decode.
 * @param read_buffer_length
 *     Number of bytes to read.
 *     Burst transfer length.
 *     Used by I2C call.
 *
 * @return ESP_OK on success.
 *     Burst read complete.
 *     Normal path.
 * @return Other esp_err_t on failure.
 *     Transport / device error.
 *     Caller should abort op.
 */
static esp_err_t read_mpu9250_registers(
    uint8_t start_register_address,
    uint8_t *read_buffer,
    size_t read_buffer_length)
{
    return i2c_master_write_read_device(
        MPU9250_I2C_PORT_NUMBER,
        MPU9250_I2C_DEVICE_ADDRESS,
        &start_register_address,
        sizeof(start_register_address),
        read_buffer,
        read_buffer_length,
        MPU9250_I2C_TRANSFER_TIMEOUT_TICKS);
}

/**
 * @brief INT pin ISR.
 *
 * Latch event and notify IMU task.
 *
 * @param user_data
 *     ISR callback context.
 *     Expected TaskHandle_t.
 *     Used for task notify.
 *
 * @return None.
 */
static void IRAM_ATTR mpu9250_interrupt_gpio_isr_callback(void *user_data)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;

    mpu9250_interrupt_received = true;

    if (task_to_notify != NULL) {
        vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Legacy I2C master init.
 *
 * Configure and install ESP-IDF legacy I2C master driver.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     Driver installed.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     Param config / driver install failed.
 *     Caller should abort IMU init.
 */
static esp_err_t initialize_mpu9250_i2c(void)
{
    i2c_config_t i2c_configuration = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU9250_I2C_SDA_GPIO_NUM,
        .scl_io_num = MPU9250_I2C_SCL_GPIO_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU9250_I2C_BUS_SPEED_HZ,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(
        i2c_param_config(MPU9250_I2C_PORT_NUMBER, &i2c_configuration),
        mpu9250_log_tag,
        "I2C param config failed");

    ESP_RETURN_ON_ERROR(
        i2c_driver_install(MPU9250_I2C_PORT_NUMBER, I2C_MODE_MASTER, 0, 0, 0),
        mpu9250_log_tag,
        "I2C driver install failed");

    mpu9250_i2c_driver_installed = true;

    return ESP_OK;
}

/**
 * @brief INT GPIO init.
 *
 * Configure INT pin as rising-edge input.
 * Install ISR service if needed.
 * Attach ISR callback to pin.
 *
 * @param task_to_notify
 *     IMU task handle.
 *     Target for ISR notify.
 *     Used for wakeup handoff.
 *
 * @return ESP_OK on success.
 *     GPIO interrupt ready.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     GPIO / ISR setup failed.
 *     Caller should abort IMU init.
 */
static esp_err_t initialize_mpu9250_interrupt_gpio(TaskHandle_t task_to_notify)
{
    mpu9250_task_to_notify = task_to_notify;

    gpio_config_t gpio_interrupt_configuration = {
        .pin_bit_mask = (1ULL << MPU9250_INTERRUPT_GPIO_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&gpio_interrupt_configuration),
        mpu9250_log_tag,
        "INT GPIO config failed");

    esp_err_t install_isr_service_result = gpio_install_isr_service(0);
    if ((install_isr_service_result != ESP_OK) &&
        (install_isr_service_result != ESP_ERR_INVALID_STATE)) {
        ESP_RETURN_ON_ERROR(
            install_isr_service_result,
            mpu9250_log_tag,
            "GPIO ISR service install failed");
    }

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(
            MPU9250_INTERRUPT_GPIO_NUM,
            mpu9250_interrupt_gpio_isr_callback,
            (void *)mpu9250_task_to_notify),
        mpu9250_log_tag,
        "INT ISR attach failed");

    mpu9250_interrupt_gpio_ready = true;

    return ESP_OK;
}

/**
 * @brief WHO_AM_I decode.
 *
 * Accept MPU-9250 and MPU-6500 IDs.
 * Cache detected device class for logs.
 *
 * @param who_am_i_register_value
 *     Raw WHO_AM_I register value.
 *     One-byte device identity.
 *     Used for variant accept / reject.
 *
 * @return ESP_OK on accepted device.
 *     Supported accel/gyro device found.
 *     Normal init path.
 * @return ESP_ERR_INVALID_RESPONSE on mismatch.
 *     Unexpected chip ID.
 *     Caller should disable IMU task.
 */
static esp_err_t decode_mpu_identity(uint8_t who_am_i_register_value)
{
    detected_mpu_who_am_i_value = who_am_i_register_value;

    if (who_am_i_register_value == MPU9250_WHO_AM_I_VALUE_MPU9250) {
        detected_mpu_device = detected_mpu_device_mpu9250;
        ESP_LOGI(mpu9250_log_tag, "Detected MPU-9250-compatible device, WHO_AM_I=0x%02X", who_am_i_register_value);
        return ESP_OK;
    }

    if (who_am_i_register_value == MPU9250_WHO_AM_I_VALUE_MPU6500) {
        detected_mpu_device = detected_mpu_device_mpu6500;
        ESP_LOGW(
            mpu9250_log_tag,
            "Detected MPU-6500-compatible device, WHO_AM_I=0x%02X; continuing with accel/gyro-only path",
            who_am_i_register_value);
        return ESP_OK;
    }

    detected_mpu_device = detected_mpu_device_unknown;

    ESP_LOGE(
        mpu9250_log_tag,
        "Unsupported WHO_AM_I=0x%02X; expected 0x%02X or 0x%02X",
        who_am_i_register_value,
        MPU9250_WHO_AM_I_VALUE_MPU6500,
        MPU9250_WHO_AM_I_VALUE_MPU9250);

    return ESP_ERR_INVALID_RESPONSE;
}

/**
 * @brief Identity check.
 *
 * Read WHO_AM_I and validate supported device.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     Device accepted.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     Read failed or device unsupported.
 *     Caller should disable IMU task.
 */
static esp_err_t verify_mpu9250_identity(void)
{
    uint8_t who_am_i_register_value = 0;

    ESP_RETURN_ON_ERROR(
        read_mpu9250_register(MPU9250_REGISTER_WHO_AM_I, &who_am_i_register_value),
        mpu9250_log_tag,
        "WHO_AM_I read failed");

    return decode_mpu_identity(who_am_i_register_value);
}

/**
 * @brief Startup register write set.
 *
 * Wake device, enable axes, set LPFs, set ranges,
 * set sample divider, configure INT pin, enable data-ready interrupt.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     Device configured.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     Register write failed.
 *     Caller should disable IMU task.
 */
static esp_err_t configure_mpu9250_startup_registers(void)
{
    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_PWR_MGMT_1,
            MPU9250_PWR_MGMT_1_STARTUP_VALUE),
        mpu9250_log_tag,
        "PWR_MGMT_1 write failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_PWR_MGMT_2,
            MPU9250_PWR_MGMT_2_STARTUP_VALUE),
        mpu9250_log_tag,
        "PWR_MGMT_2 write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_CONFIG,
            MPU9250_CONFIG_STARTUP_VALUE),
        mpu9250_log_tag,
        "CONFIG write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_GYRO_CONFIG,
            MPU9250_GYRO_CONFIG_STARTUP_VALUE),
        mpu9250_log_tag,
        "GYRO_CONFIG write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_ACCEL_CONFIG,
            MPU9250_ACCEL_CONFIG_STARTUP_VALUE),
        mpu9250_log_tag,
        "ACCEL_CONFIG write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_ACCEL_CONFIG_2,
            MPU9250_ACCEL_CONFIG_2_STARTUP_VALUE),
        mpu9250_log_tag,
        "ACCEL_CONFIG_2 write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_SMPLRT_DIV,
            MPU9250_SMPLRT_DIV_STARTUP_VALUE),
        mpu9250_log_tag,
        "SMPLRT_DIV write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_INT_PIN_CFG,
            MPU9250_INT_PIN_CFG_STARTUP_VALUE),
        mpu9250_log_tag,
        "INT_PIN_CFG write failed");

    ESP_RETURN_ON_ERROR(
        write_mpu9250_register(
            MPU9250_REGISTER_INT_ENABLE,
            MPU9250_INT_ENABLE_STARTUP_VALUE),
        mpu9250_log_tag,
        "INT_ENABLE write failed");

    return ESP_OK;
}

/**
 * @brief Full MPU init.
 *
 * Bring up I2C, INT GPIO, identity check, register config.
 *
 * @param task_to_notify
 *     IMU task handle.
 *     Used by ISR notify path.
 *     Required for INT-driven reads.
 *
 * @return ESP_OK on success.
 *     IMU ready.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     One or more init stages failed.
 *     Caller should disable IMU task.
 */
static esp_err_t initialize_mpu9250(TaskHandle_t task_to_notify)
{
    ESP_RETURN_ON_ERROR(
        initialize_mpu9250_i2c(),
        mpu9250_log_tag,
        "I2C init failed");

    ESP_RETURN_ON_ERROR(
        initialize_mpu9250_interrupt_gpio(task_to_notify),
        mpu9250_log_tag,
        "INT GPIO init failed");

    ESP_RETURN_ON_ERROR(
        verify_mpu9250_identity(),
        mpu9250_log_tag,
        "identity check failed");

    ESP_RETURN_ON_ERROR(
        configure_mpu9250_startup_registers(),
        mpu9250_log_tag,
        "startup config failed");

    return ESP_OK;
}

/**
 * @brief IMU deinit.
 *
 * Detach ISR handler and delete I2C driver if installed.
 *
 * @param None.
 *
 * @return None.
 */
static void deinitialize_mpu9250(void)
{
    if (mpu9250_interrupt_gpio_ready) {
        gpio_isr_handler_remove(MPU9250_INTERRUPT_GPIO_NUM);
        mpu9250_interrupt_gpio_ready = false;
    }

    if (mpu9250_i2c_driver_installed) {
        i2c_driver_delete(MPU9250_I2C_PORT_NUMBER);
        mpu9250_i2c_driver_installed = false;
    }
}

/**
 * @brief One measurement-frame read.
 *
 * Check INT_STATUS, confirm raw-data-ready,
 * burst-read accel + temp + gyro block, scale and cache.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     New sample cached.
 *     Normal read path.
 * @return ESP_ERR_INVALID_STATE if data-ready bit absent.
 *     Spurious edge / stale state.
 *     Caller may ignore.
 * @return Other esp_err_t on failure.
 *     Read failed.
 *     Caller should log / retry.
 */
static esp_err_t read_mpu9250_measurement_frame(void)
{
    uint8_t interrupt_status_register_value = 0;
    uint8_t measurement_register_block[14] = {0};

    ESP_RETURN_ON_ERROR(
        read_mpu9250_register(
            MPU9250_REGISTER_INT_STATUS,
            &interrupt_status_register_value),
        mpu9250_log_tag,
        "INT_STATUS read failed");

    if ((interrupt_status_register_value & 0x01U) == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        read_mpu9250_registers(
            MPU9250_REGISTER_ACCEL_XOUT_H,
            measurement_register_block,
            sizeof(measurement_register_block)),
        mpu9250_log_tag,
        "measurement burst read failed");

    mpu9250_latest_measurement.accelerometer_x_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[0], measurement_register_block[1]);
    mpu9250_latest_measurement.accelerometer_y_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[2], measurement_register_block[3]);
    mpu9250_latest_measurement.accelerometer_z_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[4], measurement_register_block[5]);

    mpu9250_latest_measurement.temperature_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[6], measurement_register_block[7]);

    mpu9250_latest_measurement.gyroscope_x_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[8], measurement_register_block[9]);
    mpu9250_latest_measurement.gyroscope_y_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[10], measurement_register_block[11]);
    mpu9250_latest_measurement.gyroscope_z_raw_counts =
        combine_bytes_into_signed_int16(measurement_register_block[12], measurement_register_block[13]);

    mpu9250_latest_measurement.accelerometer_x_meters_per_second_squared =
        ((float)mpu9250_latest_measurement.accelerometer_x_raw_counts / MPU9250_ACCELEROMETER_LSB_PER_G) *
        STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
    mpu9250_latest_measurement.accelerometer_y_meters_per_second_squared =
        ((float)mpu9250_latest_measurement.accelerometer_y_raw_counts / MPU9250_ACCELEROMETER_LSB_PER_G) *
        STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
    mpu9250_latest_measurement.accelerometer_z_meters_per_second_squared =
        ((float)mpu9250_latest_measurement.accelerometer_z_raw_counts / MPU9250_ACCELEROMETER_LSB_PER_G) *
        STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;

    mpu9250_latest_measurement.gyroscope_x_radians_per_second =
        ((float)mpu9250_latest_measurement.gyroscope_x_raw_counts / MPU9250_GYROSCOPE_LSB_PER_DEGREE_PER_SECOND) *
        DEGREES_TO_RADIANS;
    mpu9250_latest_measurement.gyroscope_y_radians_per_second =
        ((float)mpu9250_latest_measurement.gyroscope_y_raw_counts / MPU9250_GYROSCOPE_LSB_PER_DEGREE_PER_SECOND) *
        DEGREES_TO_RADIANS;
    mpu9250_latest_measurement.gyroscope_z_radians_per_second =
        ((float)mpu9250_latest_measurement.gyroscope_z_raw_counts / MPU9250_GYROSCOPE_LSB_PER_DEGREE_PER_SECOND) *
        DEGREES_TO_RADIANS;

    mpu9250_latest_measurement.temperature_degrees_celsius =
        ((float)mpu9250_latest_measurement.temperature_raw_counts /
         MPU9250_TEMPERATURE_LSB_PER_DEGREE_C) +
        MPU9250_TEMPERATURE_OFFSET_DEGREE_C;

    return ESP_OK;
}

/**
 * @brief Get latest cached sample.
 *
 * Return latest cached measurement by value.
 *
 * @param None.
 *
 * @return Latest cached measurement.
 *     Raw + scaled accel / gyro / temp.
 *     Used by publish / control paths.
 */
mpu9250_measurement_t get_latest_mpu9250_measurement(void)
{
    return mpu9250_latest_measurement;
}

/**
 * @brief IMU task main loop.
 *
 * Initialize IMU subsystem.
 * On init failure, log and stop task without rebooting MCU.
 * On success, wait on INT notify and refresh cached sample.
 *
 * @param argument
 *     FreeRTOS task argument pointer.
 *     Currently unused.
 *     Reserved for future config.
 *
 * @return None.
 */
void mpu9250_task(void *argument)
{
    (void)argument;

    esp_err_t initialize_result = initialize_mpu9250(xTaskGetCurrentTaskHandle());

    if (initialize_result != ESP_OK) {
        ESP_LOGE(
            mpu9250_log_tag,
            "IMU task disabled after init failure: %s",
            esp_err_to_name(initialize_result));
        deinitialize_mpu9250();
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        mpu9250_interrupt_received = false;

        esp_err_t read_result = read_mpu9250_measurement_frame();

        if (read_result == ESP_OK) {
            /* Cache updated. */
        } else if (read_result == ESP_ERR_INVALID_STATE) {
            /* Edge without RAW_DATA_RDY. */
        } else {
            ESP_LOGW(mpu9250_log_tag, "measurement read failed: %s", esp_err_to_name(read_result));
        }
    }
}
