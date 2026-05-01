/**
 * @file dsy_rs485_servo.c
 * @brief Minimal RS485 Modbus driver and poller for DSY-RS servo smoke tests.
 */

#include "dsy_rs485_servo.h"
#include "hexapod_hw_config.h"

#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#ifndef CONFIG_SERVO_RS485_UART_RX_BUFFER_SIZE_BYTES
#define CONFIG_SERVO_RS485_UART_RX_BUFFER_SIZE_BYTES 1024
#endif

#ifndef CONFIG_SERVO_RS485_UART_TX_BUFFER_SIZE_BYTES
#define CONFIG_SERVO_RS485_UART_TX_BUFFER_SIZE_BYTES 1024
#endif

#ifndef CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_US
#ifdef CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_MS
#define CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_US ((uint32_t)CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_US 300000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_TURNAROUND_DELAY_US
#define CONFIG_SERVO_RS485_TURNAROUND_DELAY_US 2000
#endif

#ifndef CONFIG_SERVO_RS485_POLL_PERIOD_US
#ifdef CONFIG_SERVO_RS485_POLL_PERIOD_MS
#define CONFIG_SERVO_RS485_POLL_PERIOD_US ((uint32_t)CONFIG_SERVO_RS485_POLL_PERIOD_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_POLL_PERIOD_US 100000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_US
#ifdef CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_MS
#define CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_US ((uint32_t)CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_US 5000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_TRANSACTION_RETRY_COUNT
#define CONFIG_SERVO_RS485_TRANSACTION_RETRY_COUNT 2
#endif

#ifndef CONFIG_SERVO_RS485_RETRY_BACKOFF_US
#ifdef CONFIG_SERVO_RS485_RETRY_BACKOFF_MS
#define CONFIG_SERVO_RS485_RETRY_BACKOFF_US ((uint32_t)CONFIG_SERVO_RS485_RETRY_BACKOFF_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_RETRY_BACKOFF_US 4000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_US
#ifdef CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_MS
#define CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_US ((uint32_t)CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_US 100000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_US
#ifdef CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_MS
#define CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_US ((uint32_t)CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_US 20000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_STARTUP_SETTLE_US
#ifdef CONFIG_SERVO_RS485_STARTUP_SETTLE_MS
#define CONFIG_SERVO_RS485_STARTUP_SETTLE_US ((uint32_t)CONFIG_SERVO_RS485_STARTUP_SETTLE_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_STARTUP_SETTLE_US 200000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_US
#ifdef CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_MS
#define CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_US ((uint32_t)CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_US 25000
#endif
#endif

#ifndef CONFIG_SERVO_RS485_RUNTIME_TURNAROUND_DELAY_US
#define CONFIG_SERVO_RS485_RUNTIME_TURNAROUND_DELAY_US 500
#endif

#ifndef CONFIG_SERVO_RS485_RUNTIME_TRANSACTION_ATTEMPT_COUNT
#define CONFIG_SERVO_RS485_RUNTIME_TRANSACTION_ATTEMPT_COUNT 1
#endif

#ifndef CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_US
#ifdef CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_MS
#define CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_US ((uint32_t)CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_MS * 1000U)
#else
#define CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_US 0
#endif
#endif

#ifndef CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV
#define CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV 131072
#endif

#define DSY_RS485_SERVO_UART_PORT_NUMBER              HEXAPOD_RS485_UART_PORT
#define DSY_RS485_SERVO_UART_TX_GPIO_NUM              HEXAPOD_RS485_UART_TX_GPIO
#define DSY_RS485_SERVO_UART_RX_GPIO_NUM              HEXAPOD_RS485_UART_RX_GPIO
#define DSY_RS485_SERVO_UART_DE_RE_GPIO_NUM           HEXAPOD_RS485_UART_DE_RE_GPIO
#define DSY_RS485_SERVO_BAUD_RATE                     CONFIG_SERVO_RS485_BAUD_RATE
#define DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT            CONFIG_SERVO_TEST_ACTIVE_DRIVE_COUNT
#define DSY_RS485_SERVO_UART_RX_BUFFER_SIZE_BYTES     CONFIG_SERVO_RS485_UART_RX_BUFFER_SIZE_BYTES
#define DSY_RS485_SERVO_UART_TX_BUFFER_SIZE_BYTES     CONFIG_SERVO_RS485_UART_TX_BUFFER_SIZE_BYTES
#define DSY_RS485_SERVO_RESPONSE_TIMEOUT_US           CONFIG_SERVO_RS485_RESPONSE_TIMEOUT_US
#define DSY_RS485_SERVO_TURNAROUND_DELAY_US           CONFIG_SERVO_RS485_TURNAROUND_DELAY_US
#define DSY_RS485_SERVO_POLL_PERIOD_US                CONFIG_SERVO_RS485_POLL_PERIOD_US
#define DSY_RS485_SERVO_INTER_DRIVE_POLL_DELAY_US     CONFIG_SERVO_RS485_INTER_DRIVE_POLL_DELAY_US
#define DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT       CONFIG_SERVO_RS485_TRANSACTION_RETRY_COUNT
#define DSY_RS485_SERVO_RETRY_BACKOFF_US              CONFIG_SERVO_RS485_RETRY_BACKOFF_US
#define DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US   CONFIG_SERVO_RS485_RUNTIME_RESPONSE_TIMEOUT_US
#define DSY_RS485_SERVO_RUNTIME_TURNAROUND_DELAY_US   CONFIG_SERVO_RS485_RUNTIME_TURNAROUND_DELAY_US
#define DSY_RS485_SERVO_RUNTIME_ATTEMPT_COUNT         CONFIG_SERVO_RS485_RUNTIME_TRANSACTION_ATTEMPT_COUNT
#define DSY_RS485_SERVO_RUNTIME_RETRY_BACKOFF_US      CONFIG_SERVO_RS485_RUNTIME_RETRY_BACKOFF_US

#define DSY_RS485_SERVO_MODE_SWITCH_SETTLE_US         CONFIG_SERVO_RS485_MODE_SWITCH_SETTLE_US
#define DSY_RS485_SERVO_PARAMETER_WRITE_SETTLE_US     CONFIG_SERVO_RS485_PARAMETER_WRITE_SETTLE_US
#define DSY_RS485_SERVO_STARTUP_SETTLE_US             CONFIG_SERVO_RS485_STARTUP_SETTLE_US
#define DSY_RS485_SERVO_ENABLE_GPIO_NUM               HEXAPOD_SERVO_ENABLE_GPIO
#define DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL           HEXAPOD_SERVO_ENABLE_ACTIVE_LEVEL /* Menuconfig-selectable relay GPIO / active level. Default wiring keeps servo-enable effectively active-low through the external relay path. */

#define DSY_RS485_MODBUS_FUNCTION_READ_HOLDING        0x03U
#define DSY_RS485_MODBUS_FUNCTION_WRITE_SINGLE        0x06U
#define DSY_RS485_MODBUS_FUNCTION_WRITE_MULTIPLE      0x10U
#define DSY_RS485_MODBUS_EXCEPTION_MASK               0x80U

#define DSY_PARAMETER_ADDRESS(group_number, parameter_index) \
    ((uint16_t)(((uint16_t)(group_number) * 256U) + (uint16_t)(parameter_index)))

#define DSY_P00_00_CONTROL_MODE_SELECTION             DSY_PARAMETER_ADDRESS(0U, 0U)
#define DSY_P06_00_MAIN_TORQUE_COMMAND_A_SOURCE       DSY_PARAMETER_ADDRESS(6U, 0U)
#define DSY_P06_02_TORQUE_COMMAND_SELECTION           DSY_PARAMETER_ADDRESS(6U, 2U)
#define DSY_P06_05_TORQUE_COMMAND_DIGIT_VALUE         DSY_PARAMETER_ADDRESS(6U, 5U)
#define DSY_P06_13_SPEED_LIMIT_SOURCE_SELECTION       DSY_PARAMETER_ADDRESS(6U, 13U)
#define DSY_P06_15_POSITIVE_SPEED_LIMIT               DSY_PARAMETER_ADDRESS(6U, 15U)
#define DSY_P06_16_NEGATIVE_SPEED_LIMIT               DSY_PARAMETER_ADDRESS(6U, 16U)
#define DSY_P05_00_MAIN_SPEED_COMMAND_A_SOURCE        DSY_PARAMETER_ADDRESS(5U, 0U)
#define DSY_P05_02_SPEED_COMMAND_SELECTION            DSY_PARAMETER_ADDRESS(5U, 2U)
#define DSY_P05_03_SPEED_COMMAND_DIGIT_VALUE          DSY_PARAMETER_ADDRESS(5U, 3U)
#define DSY_P05_05_SPEED_COMMAND_ACCEL_TIME_MS        DSY_PARAMETER_ADDRESS(5U, 5U)
#define DSY_P05_06_SPEED_COMMAND_DECEL_TIME_MS        DSY_PARAMETER_ADDRESS(5U, 6U)
#define DSY_P05_07_SPEED_COMMAND_LIMIT_SELECTION      DSY_PARAMETER_ADDRESS(5U, 7U)
#define DSY_P05_08_FORWARD_SPEED_LIMIT_RPM            DSY_PARAMETER_ADDRESS(5U, 8U)
#define DSY_P05_09_BACKWARD_SPEED_LIMIT_RPM           DSY_PARAMETER_ADDRESS(5U, 9U)
#define DSY_P06_06_TORQUE_LIMIT_SOURCE_SELECTION      DSY_PARAMETER_ADDRESS(6U, 6U)
#define DSY_P06_08_FORWARD_INTERNAL_TORQUE_LIMIT      DSY_PARAMETER_ADDRESS(6U, 8U)
#define DSY_P06_09_BACKWARD_INTERNAL_TORQUE_LIMIT     DSY_PARAMETER_ADDRESS(6U, 9U)
#define DSY_P02_00_FUNINL_UNASSIGNED_STATE            DSY_PARAMETER_ADDRESS(2U, 0U)
#define DSY_P02_01_DI1_FUNCTION_SELECTION             DSY_PARAMETER_ADDRESS(2U, 1U)
#define DSY_P02_02_DI2_FUNCTION_SELECTION             DSY_PARAMETER_ADDRESS(2U, 2U)
#define DSY_P02_03_DI3_FUNCTION_SELECTION             DSY_PARAMETER_ADDRESS(2U, 3U)
#define DSY_P02_10_FUNINH_UNASSIGNED_STATE            DSY_PARAMETER_ADDRESS(2U, 10U)
#define DSY_P02_11_DI1_LOGIC_SELECTION                DSY_PARAMETER_ADDRESS(2U, 11U)
#define DSY_P02_12_DI2_LOGIC_SELECTION                DSY_PARAMETER_ADDRESS(2U, 12U)
#define DSY_P02_13_DI3_LOGIC_SELECTION                DSY_PARAMETER_ADDRESS(2U, 13U)
#define DSY_P00_01_DIRECTION_SELECTION                DSY_PARAMETER_ADDRESS(0U, 1U)
#define DSY_P00_10_SERVO_OFF_STOP_MODE                DSY_PARAMETER_ADDRESS(0U, 10U)
#define DSY_P00_13_OVERTRAVEL_STOP_MODE               DSY_PARAMETER_ADDRESS(0U, 13U)
#define DSY_P18_00_SERVO_STATUS_WORD                  DSY_PARAMETER_ADDRESS(18U, 0U)
#define DSY_P18_01_ACTUAL_SPEED_RPM                   DSY_PARAMETER_ADDRESS(18U, 1U)
#define DSY_P18_05_PHASE_CURRENT_RMS                  DSY_PARAMETER_ADDRESS(18U, 5U)
#define DSY_P18_06_BUS_VOLTAGE                        DSY_PARAMETER_ADDRESS(18U, 6U)
#define DSY_P18_07_ABSOLUTE_POSITION_COUNTER          DSY_PARAMETER_ADDRESS(18U, 7U)
#define DSY_P18_13_POSITION_DEVIATION_COUNTER         DSY_PARAMETER_ADDRESS(18U, 13U)
#define DSY_P18_17_FEEDBACK_PULSE_COUNTER             DSY_PARAMETER_ADDRESS(18U, 17U)
#define DSY_P18_32_ABSOLUTE_ENCODER_SINGLE_ROUND_DATA DSY_PARAMETER_ADDRESS(18U, 32U)

static const char *dsy_rs485_log_tag = "dsy_rs485_servo";
static const uint8_t dsy_rs485_servo_slave_addresses[NUM_DSY_RS485_SERVO_DRIVES] = HEXAPOD_MOTOR_SLAVE_ADDRESS_ARRAY;
static bool dsy_rs485_bus_initialized = false;
static SemaphoreHandle_t dsy_rs485_bus_mutex = NULL;
static bool dsy_rs485_servo_enable_gpio_initialized = false;
static uint8_t dsy_rs485_last_modbus_exception_code = 0U;
static dsy_rs485_servo_state_t dsy_rs485_servo_states[NUM_DSY_RS485_SERVO_DRIVES] = {0};
static bool dsy_rs485_servo_enable_asserted = false;
static int dsy_rs485_servo_enable_output_level_latched = !DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL;
static int dsy_rs485_servo_enable_active_level_runtime = DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL;

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


static uint16_t compute_modbus_crc16(const uint8_t *buffer, size_t length_bytes)
{
    uint16_t crc_value = 0xFFFFU;

    for (size_t byte_index = 0; byte_index < length_bytes; byte_index++) {
        crc_value ^= (uint16_t)buffer[byte_index];

        for (int bit_index = 0; bit_index < 8; bit_index++) {
            if ((crc_value & 0x0001U) != 0U) {
                crc_value = (uint16_t)((crc_value >> 1) ^ 0xA001U);
            } else {
                crc_value >>= 1;
            }
        }
    }

    return crc_value;
}

static void append_modbus_crc16(uint8_t *frame_buffer, size_t frame_length_without_crc)
{
    uint16_t crc_value = compute_modbus_crc16(frame_buffer, frame_length_without_crc);
    frame_buffer[frame_length_without_crc + 0U] = (uint8_t)(crc_value & 0x00FFU);
    frame_buffer[frame_length_without_crc + 1U] = (uint8_t)((crc_value >> 8U) & 0x00FFU);
}

static esp_err_t write_single_register_once_locked(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t register_value);

static esp_err_t write_multiple_registers_once_locked(
    uint8_t slave_address,
    uint16_t start_register_address,
    const uint16_t *register_values,
    uint16_t register_count);

static esp_err_t validate_modbus_crc16(const uint8_t *frame_buffer, size_t frame_length_bytes)
{
    if (frame_length_bytes < 4U) {
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    uint16_t received_crc =
        (uint16_t)frame_buffer[frame_length_bytes - 2U] |
        ((uint16_t)frame_buffer[frame_length_bytes - 1U] << 8U);
    uint16_t computed_crc = compute_modbus_crc16(frame_buffer, frame_length_bytes - 2U);

    if (computed_crc != received_crc) {
        return DSY_RS485_ERROR_CRC_MISMATCH;
    }

    return ESP_OK;
}

static void set_rs485_direction_transmit(bool enable_transmit)
{
    if (DSY_RS485_SERVO_UART_DE_RE_GPIO_NUM >= 0) {
        gpio_set_level((gpio_num_t)DSY_RS485_SERVO_UART_DE_RE_GPIO_NUM, enable_transmit ? 1 : 0);
    }
}

static esp_err_t validate_drive_index(int drive_index, uint8_t *slave_address_out)
{
    if ((drive_index < 0) || (drive_index >= DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT)) {
        return DSY_RS485_ERROR_INVALID_DRIVE_INDEX;
    }

    if (slave_address_out != NULL) {
        *slave_address_out = dsy_rs485_servo_slave_addresses[drive_index];
    }

    return ESP_OK;
}

static esp_err_t read_exact_uart_bytes_with_timeout(
    uint8_t *buffer,
    size_t length_bytes,
    uint32_t response_timeout_us)
{
    size_t total_bytes_read = 0U;

    while (total_bytes_read < length_bytes) {
        int read_count = uart_read_bytes(
            DSY_RS485_SERVO_UART_PORT_NUMBER,
            buffer + total_bytes_read,
            length_bytes - total_bytes_read,
            ticks_from_microseconds(response_timeout_us));

        if (read_count <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        total_bytes_read += (size_t)read_count;
    }

    return ESP_OK;
}

static esp_err_t read_exact_uart_bytes(uint8_t *buffer, size_t length_bytes)
{
    return read_exact_uart_bytes_with_timeout(
        buffer,
        length_bytes,
        DSY_RS485_SERVO_RESPONSE_TIMEOUT_US);
}

static esp_err_t write_uart_frame_with_policy(
    const uint8_t *frame_buffer,
    size_t frame_length_bytes,
    uint32_t response_timeout_us,
    uint32_t turnaround_delay_us)
{
    uart_flush_input(DSY_RS485_SERVO_UART_PORT_NUMBER);
    set_rs485_direction_transmit(true);

    int written_count = uart_write_bytes(
        DSY_RS485_SERVO_UART_PORT_NUMBER,
        (const char *)frame_buffer,
        frame_length_bytes);

    if (written_count != (int)frame_length_bytes) {
        set_rs485_direction_transmit(false);
        return ESP_FAIL;
    }

    esp_err_t tx_done_result = uart_wait_tx_done(
        DSY_RS485_SERVO_UART_PORT_NUMBER,
        ticks_from_microseconds(response_timeout_us));

    if (turnaround_delay_us > 0U) {
        esp_rom_delay_us(turnaround_delay_us);
    }
    set_rs485_direction_transmit(false);

    return tx_done_result;
}

static esp_err_t write_uart_frame(const uint8_t *frame_buffer, size_t frame_length_bytes)
{
    return write_uart_frame_with_policy(
        frame_buffer,
        frame_length_bytes,
        DSY_RS485_SERVO_RESPONSE_TIMEOUT_US,
        DSY_RS485_SERVO_TURNAROUND_DELAY_US);
}

esp_err_t dsy_rs485_initialize_servo_enable_output(void)
{
    gpio_config_t servo_enable_gpio_configuration = {
        .pin_bit_mask = (1ULL << DSY_RS485_SERVO_ENABLE_GPIO_NUM),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&servo_enable_gpio_configuration),
        dsy_rs485_log_tag,
        "servo-enable gpio_config failed");

    ESP_RETURN_ON_ERROR(
        gpio_set_level(
            DSY_RS485_SERVO_ENABLE_GPIO_NUM,
            dsy_rs485_servo_enable_active_level_runtime ? 0 : 1),
        dsy_rs485_log_tag,
        "servo-enable default level set failed");

    dsy_rs485_servo_enable_gpio_initialized = true;
    dsy_rs485_servo_enable_asserted = false;
    dsy_rs485_servo_enable_output_level_latched =
        dsy_rs485_servo_enable_active_level_runtime ? 0 : 1;

    ESP_LOGI(
        dsy_rs485_log_tag,
        "servo-enable output ready on GPIO %d (default disabled, active_level_cfg=%d active_level_runtime=%d output_level=%d actual_gpio_level=%d)",
        (int)DSY_RS485_SERVO_ENABLE_GPIO_NUM,
        (int)DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL,
        dsy_rs485_servo_enable_active_level_runtime,
        dsy_rs485_servo_enable_output_level_latched,
        gpio_get_level(DSY_RS485_SERVO_ENABLE_GPIO_NUM));
    dsy_rs485_debug_log_servo_enable_line("initialize_servo_enable_output");

    return ESP_OK;
}

esp_err_t dsy_rs485_set_servo_enable_output(bool enable_servo)
{
    if (!dsy_rs485_servo_enable_gpio_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int gpio_level = enable_servo
        ? dsy_rs485_servo_enable_active_level_runtime
        : !dsy_rs485_servo_enable_active_level_runtime;

    ESP_RETURN_ON_ERROR(
        gpio_set_level(DSY_RS485_SERVO_ENABLE_GPIO_NUM, gpio_level),
        dsy_rs485_log_tag,
        "servo-enable gpio_set_level failed");

    dsy_rs485_servo_enable_asserted = enable_servo;
    dsy_rs485_servo_enable_output_level_latched = gpio_level;

    ESP_LOGI(
        dsy_rs485_log_tag,
        "servo-enable request=%d gpio_level=%d actual_gpio_level=%d active_level_cfg=%d active_level_runtime=%d",
        enable_servo ? 1 : 0,
        gpio_level,
        gpio_get_level(DSY_RS485_SERVO_ENABLE_GPIO_NUM),
        (int)DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL,
        dsy_rs485_servo_enable_active_level_runtime);
    dsy_rs485_debug_log_servo_enable_line("set_servo_enable_output");

    return ESP_OK;
}


int dsy_rs485_get_servo_enable_gpio_level(void)
{
    if (!dsy_rs485_servo_enable_gpio_initialized) {
        return -1;
    }

    return dsy_rs485_servo_enable_output_level_latched;
}

bool dsy_rs485_is_servo_enable_asserted(void)
{
    return dsy_rs485_servo_enable_asserted;
}

int dsy_rs485_get_servo_enable_active_level_runtime(void)
{
    return dsy_rs485_servo_enable_active_level_runtime;
}

void dsy_rs485_set_servo_enable_active_level_runtime(int active_level)
{
    dsy_rs485_servo_enable_active_level_runtime = active_level ? 1 : 0;
}

void dsy_rs485_restore_default_servo_enable_active_level(void)
{
    dsy_rs485_servo_enable_active_level_runtime = DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL;
}

void dsy_rs485_debug_log_servo_enable_line(const char *label)
{
    const char *resolved_label = (label != NULL) ? label : "servo_enable_line";
    int actual_gpio_level = -1;

    if (dsy_rs485_servo_enable_gpio_initialized) {
        actual_gpio_level = gpio_get_level(DSY_RS485_SERVO_ENABLE_GPIO_NUM);
    }

    ESP_LOGI(
        dsy_rs485_log_tag,
        "%s servo_enable gpio=%d request=%d latched=%d actual=%d active_level_cfg=%d active_level_runtime=%d semantics[gpio high=>relay on=>24V at DI, gpio low=>relay off=>DI low]",
        resolved_label,
        (int)DSY_RS485_SERVO_ENABLE_GPIO_NUM,
        dsy_rs485_servo_enable_asserted ? 1 : 0,
        dsy_rs485_servo_enable_output_level_latched,
        actual_gpio_level,
        (int)DSY_RS485_SERVO_ENABLE_ACTIVE_LEVEL,
        dsy_rs485_servo_enable_active_level_runtime);
}

static void log_u16_debug_value(uint8_t slave_address, const char *label, esp_err_t result, uint16_t value)
{
    if (result == ESP_OK) {
        ESP_LOGI(dsy_rs485_log_tag, "%s addr=%u value=%u (0x%04X)", label, (unsigned)slave_address, (unsigned)value, (unsigned)value);
    } else {
        ESP_LOGW(dsy_rs485_log_tag, "%s addr=%u read failed result=%s exception=%u", label, (unsigned)slave_address, esp_err_to_name(result), (unsigned)dsy_rs485_last_modbus_exception_code);
    }
}

static void log_s16_debug_value(uint8_t slave_address, const char *label, esp_err_t result, int16_t value)
{
    if (result == ESP_OK) {
        ESP_LOGI(dsy_rs485_log_tag, "%s addr=%u value=%d (0x%04X)", label, (unsigned)slave_address, (int)value, (unsigned)((uint16_t)value));
    } else {
        ESP_LOGW(dsy_rs485_log_tag, "%s addr=%u read failed result=%s exception=%u", label, (unsigned)slave_address, esp_err_to_name(result), (unsigned)dsy_rs485_last_modbus_exception_code);
    }
}

static void log_s32_debug_value(uint8_t slave_address, const char *label, esp_err_t result, int32_t value)
{
    if (result == ESP_OK) {
        ESP_LOGI(dsy_rs485_log_tag, "%s addr=%u value=%ld (0x%08lX)", label, (unsigned)slave_address, (long)value, (unsigned long)((uint32_t)value));
    } else {
        ESP_LOGW(dsy_rs485_log_tag, "%s addr=%u read failed result=%s exception=%u", label, (unsigned)slave_address, esp_err_to_name(result), (unsigned)dsy_rs485_last_modbus_exception_code);
    }
}

esp_err_t dsy_rs485_debug_dump_drive_snapshot(int drive_index, const char *label)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    uint16_t status_word = 0U;
    int16_t actual_speed_rpm = 0;
    uint16_t phase_current_centiamps = 0U;
    uint16_t bus_voltage_deci_volts = 0U;
    int32_t absolute_position_counts = 0;
    int32_t position_deviation_counts = 0;
    int32_t feedback_pulse_counts = 0;

    esp_err_t status_result = dsy_rs485_read_parameter_u16(slave_address, 18U, 0U, &status_word);
    esp_err_t speed_result = dsy_rs485_read_parameter_s16(slave_address, 18U, 1U, &actual_speed_rpm);
    esp_err_t current_result = dsy_rs485_read_parameter_u16(slave_address, 18U, 5U, &phase_current_centiamps);
    esp_err_t bus_result = dsy_rs485_read_parameter_u16(slave_address, 18U, 6U, &bus_voltage_deci_volts);
    esp_err_t pos_result = dsy_rs485_read_parameter_s32(slave_address, 18U, 7U, &absolute_position_counts);
    esp_err_t dev_result = dsy_rs485_read_parameter_s32(slave_address, 18U, 13U, &position_deviation_counts);
    esp_err_t feedback_result = dsy_rs485_read_parameter_s32(slave_address, 18U, 17U, &feedback_pulse_counts);

    dsy_rs485_debug_log_servo_enable_line(label);

    ESP_LOGI(
        dsy_rs485_log_tag,
        "%s drive=%d addr=%u snapshot status=%s speed=%s current=%s bus=%s pos=%s dev=%s feedback=%s status_word=0x%04X status_bit0=%u status_bit1=%u status_bit2=%u status_bit3=%u",
        (label != NULL) ? label : "snapshot",
        drive_index,
        (unsigned)slave_address,
        esp_err_to_name(status_result),
        esp_err_to_name(speed_result),
        esp_err_to_name(current_result),
        esp_err_to_name(bus_result),
        esp_err_to_name(pos_result),
        esp_err_to_name(dev_result),
        esp_err_to_name(feedback_result),
        (unsigned)status_word,
        (unsigned)((status_word >> 0U) & 0x1U),
        (unsigned)((status_word >> 1U) & 0x1U),
        (unsigned)((status_word >> 2U) & 0x1U),
        (unsigned)((status_word >> 3U) & 0x1U));

    log_s16_debug_value(slave_address, "P18.01 actual speed rpm", speed_result, actual_speed_rpm);
    log_u16_debug_value(slave_address, "P18.05 phase current centiamps", current_result, phase_current_centiamps);
    log_u16_debug_value(slave_address, "P18.06 bus voltage decivolts", bus_result, bus_voltage_deci_volts);
    log_s32_debug_value(slave_address, "P18.07 absolute position counter", pos_result, absolute_position_counts);
    log_s32_debug_value(slave_address, "P18.13 position deviation counts", dev_result, position_deviation_counts);
    log_s32_debug_value(slave_address, "P18.17 feedback pulse counter", feedback_result, feedback_pulse_counts);

    return (status_result == ESP_OK) ? ESP_OK : status_result;
}

esp_err_t dsy_rs485_debug_dump_drive_io_config(int drive_index, const char *label)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    uint16_t value = 0U;
    esp_err_t result = ESP_OK;

    ESP_LOGI(dsy_rs485_log_tag, "%s drive=%d addr=%u IO config dump", (label != NULL) ? label : "io_cfg", drive_index, (unsigned)slave_address);

    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 0U, &value);
    log_u16_debug_value(slave_address, "P02.00 FunINL unassigned state", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 1U, &value);
    log_u16_debug_value(slave_address, "P02.01 DI1 function selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 2U, &value);
    log_u16_debug_value(slave_address, "P02.02 DI2 function selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 3U, &value);
    log_u16_debug_value(slave_address, "P02.03 DI3 function selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 10U, &value);
    log_u16_debug_value(slave_address, "P02.10 FunINH unassigned state", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 11U, &value);
    log_u16_debug_value(slave_address, "P02.11 DI1 logic selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 12U, &value);
    log_u16_debug_value(slave_address, "P02.12 DI2 logic selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 2U, 13U, &value);
    log_u16_debug_value(slave_address, "P02.13 DI3 logic selection", result, value);

    ESP_LOGI(
        dsy_rs485_log_tag,
        "%s drive=%d addr=%u manual defaults of interest: P02.01=1 and P02.11=0 for DI1 low-level-effective FunIN.1",
        (label != NULL) ? label : "io_cfg",
        drive_index,
        (unsigned)slave_address);

    return ESP_OK;
}

esp_err_t dsy_rs485_debug_dump_drive_mode_config(int drive_index, const char *label)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    uint16_t value = 0U;
    int16_t s16_value = 0;

    ESP_LOGI(dsy_rs485_log_tag, "%s drive=%d addr=%u mode/config dump", (label != NULL) ? label : "mode_cfg", drive_index, (unsigned)slave_address);

    esp_err_t result = dsy_rs485_read_parameter_u16(slave_address, 0U, 0U, &value);
    log_u16_debug_value(slave_address, "P00.00 control mode selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 0U, 1U, &value);
    log_u16_debug_value(slave_address, "P00.01 direction selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 0U, 10U, &value);
    log_u16_debug_value(slave_address, "P00.10 servo-off stop mode", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 0U, 13U, &value);
    log_u16_debug_value(slave_address, "P00.13 overtravel stop mode", result, value);

    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 0U, &value);
    log_u16_debug_value(slave_address, "P05.00 speed source", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 2U, &value);
    log_u16_debug_value(slave_address, "P05.02 speed selection", result, value);
    result = dsy_rs485_read_parameter_s16(slave_address, 5U, 3U, &s16_value);
    log_s16_debug_value(slave_address, "P05.03 speed command", result, s16_value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 5U, &value);
    log_u16_debug_value(slave_address, "P05.05 accel time", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 6U, &value);
    log_u16_debug_value(slave_address, "P05.06 decel time", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 7U, &value);
    log_u16_debug_value(slave_address, "P05.07 speed limit selection", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 8U, &value);
    log_u16_debug_value(slave_address, "P05.08 forward speed limit", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 5U, 9U, &value);
    log_u16_debug_value(slave_address, "P05.09 backward speed limit", result, value);

    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 0U, &value);
    log_u16_debug_value(slave_address, "P06.00 torque source", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 2U, &value);
    log_u16_debug_value(slave_address, "P06.02 torque selection", result, value);
    result = dsy_rs485_read_parameter_s16(slave_address, 6U, 5U, &s16_value);
    log_s16_debug_value(slave_address, "P06.05 torque command", result, s16_value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 6U, &value);
    log_u16_debug_value(slave_address, "P06.06 torque limit source", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 8U, &value);
    log_u16_debug_value(slave_address, "P06.08 forward torque limit", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 9U, &value);
    log_u16_debug_value(slave_address, "P06.09 backward torque limit", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 13U, &value);
    log_u16_debug_value(slave_address, "P06.13 torque speed-limit source", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 15U, &value);
    log_u16_debug_value(slave_address, "P06.15 positive speed limit", result, value);
    result = dsy_rs485_read_parameter_u16(slave_address, 6U, 16U, &value);
    log_u16_debug_value(slave_address, "P06.16 negative speed limit", result, value);

    return ESP_OK;
}

esp_err_t dsy_rs485_debug_dump_all_drives(const char *label, bool include_io_config, bool include_mode_config)
{
    for (int drive_index = 0; drive_index < DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT; drive_index++) {
        (void)dsy_rs485_debug_dump_drive_snapshot(drive_index, label);
        if (include_io_config) {
            (void)dsy_rs485_debug_dump_drive_io_config(drive_index, label);
        }
        if (include_mode_config) {
            (void)dsy_rs485_debug_dump_drive_mode_config(drive_index, label);
        }
    }
    return ESP_OK;
}

static esp_err_t initialize_dsy_rs485_bus(void)
{
    if (dsy_rs485_bus_initialized) {
        return ESP_OK;
    }

    uart_config_t uart_configuration = {
        .baud_rate = DSY_RS485_SERVO_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(
            DSY_RS485_SERVO_UART_PORT_NUMBER,
            DSY_RS485_SERVO_UART_RX_BUFFER_SIZE_BYTES,
            DSY_RS485_SERVO_UART_TX_BUFFER_SIZE_BYTES,
            0,
            NULL,
            0),
        dsy_rs485_log_tag,
        "uart_driver_install failed");

    ESP_RETURN_ON_ERROR(
        uart_param_config(DSY_RS485_SERVO_UART_PORT_NUMBER, &uart_configuration),
        dsy_rs485_log_tag,
        "uart_param_config failed");

    ESP_RETURN_ON_ERROR(
        uart_set_pin(
            DSY_RS485_SERVO_UART_PORT_NUMBER,
            DSY_RS485_SERVO_UART_TX_GPIO_NUM,
            DSY_RS485_SERVO_UART_RX_GPIO_NUM,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE),
        dsy_rs485_log_tag,
        "uart_set_pin failed");

    if (DSY_RS485_SERVO_UART_DE_RE_GPIO_NUM >= 0) {
        gpio_config_t de_re_gpio_configuration = {
            .pin_bit_mask = 0,
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        de_re_gpio_configuration.pin_bit_mask =
            (1ULL << DSY_RS485_SERVO_UART_DE_RE_GPIO_NUM);

        ESP_RETURN_ON_ERROR(
            gpio_config(&de_re_gpio_configuration),
            dsy_rs485_log_tag,
            "gpio_config failed");

        set_rs485_direction_transmit(false);
    }

    dsy_rs485_bus_mutex = xSemaphoreCreateMutex();
    if (dsy_rs485_bus_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (int drive_index = 0; drive_index < NUM_DSY_RS485_SERVO_DRIVES; drive_index++) {
        dsy_rs485_servo_states[drive_index].slave_address = dsy_rs485_servo_slave_addresses[drive_index];
    }

    dsy_rs485_bus_initialized = true;
    ESP_LOGI(
        dsy_rs485_log_tag,
        "RS485 ready on UART2 TX=%d RX=%d active=%d baud=%d startup_settle_us=%d poll_pass_us=%d inter_drive_poll_us=%d",
        DSY_RS485_SERVO_UART_TX_GPIO_NUM,
        DSY_RS485_SERVO_UART_RX_GPIO_NUM,
        DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT,
        DSY_RS485_SERVO_BAUD_RATE,
        DSY_RS485_SERVO_STARTUP_SETTLE_US,
        DSY_RS485_SERVO_POLL_PERIOD_US,
        DSY_RS485_SERVO_INTER_DRIVE_POLL_DELAY_US);

    delay_microseconds(DSY_RS485_SERVO_STARTUP_SETTLE_US);
    return ESP_OK;
}

esp_err_t dsy_rs485_initialize_bus(void)
{
    return initialize_dsy_rs485_bus();
}

bool is_dsy_rs485_bus_initialized(void)
{
    return dsy_rs485_bus_initialized;
}

int get_dsy_rs485_active_drive_count(void)
{
    return DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT;
}

uint8_t get_dsy_rs485_last_modbus_exception_code(void)
{
    return dsy_rs485_last_modbus_exception_code;
}

dsy_rs485_servo_state_t get_dsy_rs485_servo_state(int drive_index)
{
    dsy_rs485_servo_state_t empty_state = {0};

    if ((drive_index < 0) || (drive_index >= NUM_DSY_RS485_SERVO_DRIVES)) {
        return empty_state;
    }

    return dsy_rs485_servo_states[drive_index];
}

uint8_t get_dsy_rs485_servo_slave_address(int drive_index)
{
    if ((drive_index < 0) || (drive_index >= NUM_DSY_RS485_SERVO_DRIVES)) {
        return 0U;
    }

    return dsy_rs485_servo_slave_addresses[drive_index];
}

esp_err_t dsy_rs485_read_holding_registers(
    uint8_t slave_address,
    uint16_t start_register_address,
    uint16_t register_count,
    uint16_t *register_values_out)
{
    if ((!dsy_rs485_bus_initialized) || (dsy_rs485_bus_mutex == NULL)) {
        return DSY_RS485_ERROR_BUS_NOT_INITIALIZED;
    }

    if ((register_count == 0U) || (register_values_out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t final_result = ESP_FAIL;

    for (int attempt_index = 0; attempt_index < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT; attempt_index++) {
        if (xSemaphoreTake(dsy_rs485_bus_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        dsy_rs485_last_modbus_exception_code = 0U;

        uint8_t request_frame[8] = {
            slave_address,
            DSY_RS485_MODBUS_FUNCTION_READ_HOLDING,
            (uint8_t)((start_register_address >> 8U) & 0xFFU),
            (uint8_t)(start_register_address & 0xFFU),
            (uint8_t)((register_count >> 8U) & 0xFFU),
            (uint8_t)(register_count & 0xFFU),
            0U,
            0U,
        };

        append_modbus_crc16(request_frame, 6U);

        esp_err_t result = write_uart_frame(request_frame, sizeof(request_frame));
        if (result == ESP_OK) {
            uint8_t response_header[3] = {0};
            result = read_exact_uart_bytes(response_header, sizeof(response_header));
            if (result == ESP_OK) {
                if ((response_header[1] & DSY_RS485_MODBUS_EXCEPTION_MASK) != 0U) {
                    uint8_t exception_tail[2] = {0};
                    result = read_exact_uart_bytes(exception_tail, sizeof(exception_tail));
                    if (result == ESP_OK) {
                        dsy_rs485_last_modbus_exception_code = response_header[2];
                        result = DSY_RS485_ERROR_MODBUS_EXCEPTION;
                    }
                } else if ((response_header[0] != slave_address) ||
                           (response_header[1] != DSY_RS485_MODBUS_FUNCTION_READ_HOLDING) ||
                           (response_header[2] != (uint8_t)(register_count * 2U))) {
                    result = DSY_RS485_ERROR_INVALID_RESPONSE;
                } else {
                    uint8_t response_data_and_crc[256] = {0};
                    size_t response_data_and_crc_length = ((size_t)register_count * 2U) + 2U;
                    result = read_exact_uart_bytes(response_data_and_crc, response_data_and_crc_length);
                    if (result == ESP_OK) {
                        uint8_t response_frame[259] = {0};
                        response_frame[0] = response_header[0];
                        response_frame[1] = response_header[1];
                        response_frame[2] = response_header[2];
                        memcpy(&response_frame[3], response_data_and_crc, response_data_and_crc_length);

                        result = validate_modbus_crc16(response_frame, 3U + response_data_and_crc_length);
                        if (result == ESP_OK) {
                            for (uint16_t register_index = 0U; register_index < register_count; register_index++) {
                                register_values_out[register_index] =
                                    ((uint16_t)response_data_and_crc[(size_t)register_index * 2U] << 8U) |
                                    (uint16_t)response_data_and_crc[(size_t)register_index * 2U + 1U];
                            }
                        }
                    }
                }
            }
        }

        xSemaphoreGive(dsy_rs485_bus_mutex);

        if (result == ESP_OK) {
            return ESP_OK;
        }

        final_result = result;
        if (attempt_index + 1 < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT) {
            delay_microseconds(DSY_RS485_SERVO_RETRY_BACKOFF_US);
        }
    }

    return final_result;
}

static esp_err_t write_single_register_once_locked(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t register_value)
{
    uint8_t request_frame[8] = {
        slave_address,
        DSY_RS485_MODBUS_FUNCTION_WRITE_SINGLE,
        (uint8_t)((register_address >> 8U) & 0xFFU),
        (uint8_t)(register_address & 0xFFU),
        (uint8_t)((register_value >> 8U) & 0xFFU),
        (uint8_t)(register_value & 0xFFU),
        0U,
        0U,
    };

    append_modbus_crc16(request_frame, 6U);

    esp_err_t result = write_uart_frame(request_frame, sizeof(request_frame));
    if (result != ESP_OK) {
        return result;
    }

    uint8_t response_frame[8] = {0};
    result = read_exact_uart_bytes(response_frame, sizeof(response_frame));
    if (result != ESP_OK) {
        return result;
    }

    result = validate_modbus_crc16(response_frame, sizeof(response_frame));
    if (result != ESP_OK) {
        return result;
    }

    if ((response_frame[1] & DSY_RS485_MODBUS_EXCEPTION_MASK) != 0U) {
        dsy_rs485_last_modbus_exception_code = response_frame[2];
        return DSY_RS485_ERROR_MODBUS_EXCEPTION;
    }

    if ((response_frame[0] != slave_address) ||
        (response_frame[1] != DSY_RS485_MODBUS_FUNCTION_WRITE_SINGLE)) {
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t dsy_rs485_write_single_register(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t register_value)
{
    if ((!dsy_rs485_bus_initialized) || (dsy_rs485_bus_mutex == NULL)) {
        return DSY_RS485_ERROR_BUS_NOT_INITIALIZED;
    }

    esp_err_t final_result = ESP_FAIL;

    for (int attempt_index = 0; attempt_index < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT; attempt_index++) {
        if (xSemaphoreTake(dsy_rs485_bus_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        dsy_rs485_last_modbus_exception_code = 0U;
        esp_err_t result = write_single_register_once_locked(
            slave_address,
            register_address,
            register_value);

        xSemaphoreGive(dsy_rs485_bus_mutex);

        if (result == ESP_OK) {
            return ESP_OK;
        }

        final_result = result;
        if (attempt_index + 1 < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT) {
            delay_microseconds(DSY_RS485_SERVO_RETRY_BACKOFF_US);
        }
    }

    return final_result;
}

static esp_err_t write_multiple_registers_once_locked(
    uint8_t slave_address,
    uint16_t start_register_address,
    const uint16_t *register_values,
    uint16_t register_count)
{
    uint8_t request_frame[80] = {0};
    size_t frame_length_without_crc = 7U + ((size_t)register_count * 2U);

    request_frame[0] = slave_address;
    request_frame[1] = DSY_RS485_MODBUS_FUNCTION_WRITE_MULTIPLE;
    request_frame[2] = (uint8_t)((start_register_address >> 8U) & 0xFFU);
    request_frame[3] = (uint8_t)(start_register_address & 0xFFU);
    request_frame[4] = (uint8_t)((register_count >> 8U) & 0xFFU);
    request_frame[5] = (uint8_t)(register_count & 0xFFU);
    request_frame[6] = (uint8_t)(register_count * 2U);

    for (uint16_t register_index = 0U; register_index < register_count; register_index++) {
        request_frame[7U + ((size_t)register_index * 2U)] =
            (uint8_t)((register_values[register_index] >> 8U) & 0xFFU);
        request_frame[7U + ((size_t)register_index * 2U) + 1U] =
            (uint8_t)(register_values[register_index] & 0xFFU);
    }

    append_modbus_crc16(request_frame, frame_length_without_crc);

    esp_err_t result = write_uart_frame(request_frame, frame_length_without_crc + 2U);
    if (result != ESP_OK) {
        return result;
    }

    uint8_t response_frame[8] = {0};
    result = read_exact_uart_bytes(response_frame, sizeof(response_frame));
    if (result != ESP_OK) {
        return result;
    }

    result = validate_modbus_crc16(response_frame, sizeof(response_frame));
    if (result != ESP_OK) {
        return result;
    }

    if ((response_frame[1] & DSY_RS485_MODBUS_EXCEPTION_MASK) != 0U) {
        dsy_rs485_last_modbus_exception_code = response_frame[2];
        return DSY_RS485_ERROR_MODBUS_EXCEPTION;
    }

    if ((response_frame[0] != slave_address) ||
        (response_frame[1] != DSY_RS485_MODBUS_FUNCTION_WRITE_MULTIPLE)) {
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t dsy_rs485_write_multiple_registers(
    uint8_t slave_address,
    uint16_t start_register_address,
    const uint16_t *register_values,
    uint16_t register_count)
{
    if ((!dsy_rs485_bus_initialized) || (dsy_rs485_bus_mutex == NULL)) {
        return DSY_RS485_ERROR_BUS_NOT_INITIALIZED;
    }

    if ((register_values == NULL) || (register_count == 0U) || (register_count > 32U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t final_result = ESP_FAIL;

    for (int attempt_index = 0; attempt_index < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT; attempt_index++) {
        if (xSemaphoreTake(dsy_rs485_bus_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        dsy_rs485_last_modbus_exception_code = 0U;
        esp_err_t result = write_multiple_registers_once_locked(
            slave_address,
            start_register_address,
            register_values,
            register_count);

        xSemaphoreGive(dsy_rs485_bus_mutex);

        if (result == ESP_OK) {
            return ESP_OK;
        }

        final_result = result;
        if (attempt_index + 1 < DSY_RS485_SERVO_TRANSACTION_RETRY_COUNT) {
            delay_microseconds(DSY_RS485_SERVO_RETRY_BACKOFF_US);
        }
    }

    return final_result;
}

esp_err_t dsy_rs485_read_parameter_u16(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, uint16_t *value_out)
{
    return dsy_rs485_read_holding_registers(
        slave_address,
        DSY_PARAMETER_ADDRESS(parameter_group, parameter_index),
        1U,
        value_out);
}

esp_err_t dsy_rs485_read_parameter_s16(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, int16_t *value_out)
{
    uint16_t value_u16 = 0U;
    esp_err_t result = dsy_rs485_read_parameter_u16(slave_address, parameter_group, parameter_index, &value_u16);
    if ((result == ESP_OK) && (value_out != NULL)) {
        *value_out = (int16_t)value_u16;
    }
    return result;
}

esp_err_t dsy_rs485_read_parameter_u32(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, uint32_t *value_out)
{
    uint16_t register_values[2] = {0U, 0U};
    esp_err_t result = dsy_rs485_read_holding_registers(
        slave_address,
        DSY_PARAMETER_ADDRESS(parameter_group, parameter_index),
        2U,
        register_values);
    if ((result == ESP_OK) && (value_out != NULL)) {
        *value_out = ((uint32_t)register_values[0] << 16U) | (uint32_t)register_values[1];
    }
    return result;
}

esp_err_t dsy_rs485_read_parameter_s32(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, int32_t *value_out)
{
    uint32_t value_u32 = 0U;
    esp_err_t result = dsy_rs485_read_parameter_u32(slave_address, parameter_group, parameter_index, &value_u32);
    if ((result == ESP_OK) && (value_out != NULL)) {
        *value_out = (int32_t)value_u32;
    }
    return result;
}

esp_err_t dsy_rs485_write_parameter_u16(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, uint16_t value)
{
    return dsy_rs485_write_single_register(
        slave_address,
        DSY_PARAMETER_ADDRESS(parameter_group, parameter_index),
        value);
}

esp_err_t dsy_rs485_write_parameter_s16(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, int16_t value)
{
    return dsy_rs485_write_parameter_u16(slave_address, parameter_group, parameter_index, (uint16_t)value);
}

esp_err_t dsy_rs485_write_parameter_u32(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, uint32_t value)
{
    uint16_t register_values[2] = {
        (uint16_t)((value >> 16U) & 0xFFFFU),
        (uint16_t)(value & 0xFFFFU),
    };

    return dsy_rs485_write_multiple_registers(
        slave_address,
        DSY_PARAMETER_ADDRESS(parameter_group, parameter_index),
        register_values,
        2U);
}

esp_err_t dsy_rs485_write_parameter_s32(uint8_t slave_address, uint8_t parameter_group, uint8_t parameter_index, int32_t value)
{
    return dsy_rs485_write_parameter_u32(slave_address, parameter_group, parameter_index, (uint32_t)value);
}

static int get_dsy_rs485_runtime_attempt_count(void)
{
    return (DSY_RS485_SERVO_RUNTIME_ATTEMPT_COUNT > 0) ?
        DSY_RS485_SERVO_RUNTIME_ATTEMPT_COUNT : 1;
}

static esp_err_t runtime_read_holding_registers(
    uint8_t slave_address,
    uint16_t start_register_address,
    uint16_t register_count,
    uint16_t *register_values_out)
{
    if ((!dsy_rs485_bus_initialized) || (dsy_rs485_bus_mutex == NULL)) {
        return DSY_RS485_ERROR_BUS_NOT_INITIALIZED;
    }

    if ((register_count == 0U) || (register_values_out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t final_result = ESP_FAIL;
    const int attempt_count = get_dsy_rs485_runtime_attempt_count();

    for (int attempt_index = 0; attempt_index < attempt_count; attempt_index++) {
        if (xSemaphoreTake(dsy_rs485_bus_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        dsy_rs485_last_modbus_exception_code = 0U;

        uint8_t request_frame[8] = {
            slave_address,
            DSY_RS485_MODBUS_FUNCTION_READ_HOLDING,
            (uint8_t)((start_register_address >> 8U) & 0xFFU),
            (uint8_t)(start_register_address & 0xFFU),
            (uint8_t)((register_count >> 8U) & 0xFFU),
            (uint8_t)(register_count & 0xFFU),
            0U,
            0U,
        };

        append_modbus_crc16(request_frame, 6U);

        esp_err_t result = write_uart_frame_with_policy(
            request_frame,
            sizeof(request_frame),
            DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US,
            DSY_RS485_SERVO_RUNTIME_TURNAROUND_DELAY_US);
        if (result == ESP_OK) {
            uint8_t response_header[3] = {0};
            result = read_exact_uart_bytes_with_timeout(
                response_header,
                sizeof(response_header),
                DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US);
            if (result == ESP_OK) {
                if ((response_header[1] & DSY_RS485_MODBUS_EXCEPTION_MASK) != 0U) {
                    uint8_t exception_tail[2] = {0};
                    result = read_exact_uart_bytes_with_timeout(
                        exception_tail,
                        sizeof(exception_tail),
                        DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US);
                    if (result == ESP_OK) {
                        dsy_rs485_last_modbus_exception_code = response_header[2];
                        result = DSY_RS485_ERROR_MODBUS_EXCEPTION;
                    }
                } else if ((response_header[0] != slave_address) ||
                           (response_header[1] != DSY_RS485_MODBUS_FUNCTION_READ_HOLDING) ||
                           (response_header[2] != (uint8_t)(register_count * 2U))) {
                    result = DSY_RS485_ERROR_INVALID_RESPONSE;
                } else {
                    uint8_t response_data_and_crc[256] = {0};
                    size_t response_data_and_crc_length = ((size_t)register_count * 2U) + 2U;
                    result = read_exact_uart_bytes_with_timeout(
                        response_data_and_crc,
                        response_data_and_crc_length,
                        DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US);
                    if (result == ESP_OK) {
                        uint8_t response_frame[259] = {0};
                        response_frame[0] = response_header[0];
                        response_frame[1] = response_header[1];
                        response_frame[2] = response_header[2];
                        memcpy(&response_frame[3], response_data_and_crc, response_data_and_crc_length);

                        result = validate_modbus_crc16(response_frame, 3U + response_data_and_crc_length);
                        if (result == ESP_OK) {
                            for (uint16_t register_index = 0U; register_index < register_count; register_index++) {
                                register_values_out[register_index] =
                                    ((uint16_t)response_data_and_crc[(size_t)register_index * 2U] << 8U) |
                                    (uint16_t)response_data_and_crc[(size_t)register_index * 2U + 1U];
                            }
                        }
                    }
                }
            }
        }

        xSemaphoreGive(dsy_rs485_bus_mutex);

        if (result == ESP_OK) {
            return ESP_OK;
        }

        final_result = result;
        if (attempt_index + 1 < attempt_count) {
            delay_microseconds(DSY_RS485_SERVO_RUNTIME_RETRY_BACKOFF_US);
        }
    }

    return final_result;
}

static esp_err_t runtime_write_single_register(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t register_value)
{
    if ((!dsy_rs485_bus_initialized) || (dsy_rs485_bus_mutex == NULL)) {
        return DSY_RS485_ERROR_BUS_NOT_INITIALIZED;
    }

    esp_err_t final_result = ESP_FAIL;
    const int attempt_count = get_dsy_rs485_runtime_attempt_count();

    for (int attempt_index = 0; attempt_index < attempt_count; attempt_index++) {
        if (xSemaphoreTake(dsy_rs485_bus_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        dsy_rs485_last_modbus_exception_code = 0U;

        uint8_t request_frame[8] = {
            slave_address,
            DSY_RS485_MODBUS_FUNCTION_WRITE_SINGLE,
            (uint8_t)((register_address >> 8U) & 0xFFU),
            (uint8_t)(register_address & 0xFFU),
            (uint8_t)((register_value >> 8U) & 0xFFU),
            (uint8_t)(register_value & 0xFFU),
            0U,
            0U,
        };

        append_modbus_crc16(request_frame, 6U);

        esp_err_t result = write_uart_frame_with_policy(
            request_frame,
            sizeof(request_frame),
            DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US,
            DSY_RS485_SERVO_RUNTIME_TURNAROUND_DELAY_US);

        if (result == ESP_OK) {
            uint8_t response_frame[8] = {0};
            result = read_exact_uart_bytes_with_timeout(
                response_frame,
                sizeof(response_frame),
                DSY_RS485_SERVO_RUNTIME_RESPONSE_TIMEOUT_US);
            if (result == ESP_OK) {
                result = validate_modbus_crc16(response_frame, sizeof(response_frame));
            }
            if (result == ESP_OK) {
                if ((response_frame[1] & DSY_RS485_MODBUS_EXCEPTION_MASK) != 0U) {
                    dsy_rs485_last_modbus_exception_code = response_frame[2];
                    result = DSY_RS485_ERROR_MODBUS_EXCEPTION;
                } else if ((response_frame[0] != slave_address) ||
                           (response_frame[1] != DSY_RS485_MODBUS_FUNCTION_WRITE_SINGLE)) {
                    result = DSY_RS485_ERROR_INVALID_RESPONSE;
                }
            }
        }

        xSemaphoreGive(dsy_rs485_bus_mutex);

        if (result == ESP_OK) {
            return ESP_OK;
        }

        final_result = result;
        if (attempt_index + 1 < attempt_count) {
            delay_microseconds(DSY_RS485_SERVO_RUNTIME_RETRY_BACKOFF_US);
        }
    }

    return final_result;
}

esp_err_t dsy_rs485_runtime_write_drive_speed_command_rpm(int drive_index, int16_t speed_command_rpm)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    return runtime_write_single_register(
        slave_address,
        DSY_P05_03_SPEED_COMMAND_DIGIT_VALUE,
        (uint16_t)speed_command_rpm);
}

esp_err_t dsy_rs485_runtime_read_drive_feedback_pulse_counts(
    int drive_index,
    int32_t *feedback_pulse_counts_out)
{
    uint8_t slave_address = 0U;
    uint16_t register_values[2] = {0U, 0U};

    if (feedback_pulse_counts_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    esp_err_t result = runtime_read_holding_registers(
        slave_address,
        DSY_P18_17_FEEDBACK_PULSE_COUNTER,
        2U,
        register_values);

    if (result == ESP_OK) {
        *feedback_pulse_counts_out =
            (int32_t)(((uint32_t)register_values[0] << 16U) | (uint32_t)register_values[1]);
    }

    return result;
}

static uint32_t select_absolute_encoder_single_turn_count(const uint16_t *register_values)
{
    const uint32_t high_word_first_count =
        ((uint32_t)register_values[0] << 16U) |
        (uint32_t)register_values[1];
    const uint32_t low_word_first_count =
        ((uint32_t)register_values[1] << 16U) |
        (uint32_t)register_values[0];

    /*
     * The manual names P18.32 as the absolute encoder single-round data and
     * P18.34 as the multi-round data. Because P18.33 is skipped, P18.32 is
     * treated as a 32-bit value occupying P18.32/P18.33. Keep the word-order
     * guard because some DSY firmware variants expose 32-bit monitor values
     * with opposite Modbus word order.
     */
    if (high_word_first_count < (uint32_t)CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV) {
        return high_word_first_count;
    }
    if (low_word_first_count < (uint32_t)CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV) {
        return low_word_first_count;
    }
    return high_word_first_count;
}

esp_err_t dsy_rs485_runtime_read_drive_absolute_encoder_single_turn_counts(
    int drive_index,
    uint32_t *single_turn_counts_out)
{
    uint8_t slave_address = 0U;
    uint16_t encoder_registers[2] = {0U, 0U};

    if (single_turn_counts_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    /*
     * P18.32 absolute encoder single-round data is the motor encoder position
     * within one motor revolution. It occupies two 16-bit Modbus registers for
     * 17-bit M17 encoders. P18.34 is not used here because the installed M17S
     * encoder is single-turn; the bridge unwraps P18.32 locally.
     */
    esp_err_t result = runtime_read_holding_registers(
        slave_address,
        DSY_P18_32_ABSOLUTE_ENCODER_SINGLE_ROUND_DATA,
        2U,
        encoder_registers);
    if (result != ESP_OK) {
        return result;
    }

    *single_turn_counts_out = select_absolute_encoder_single_turn_count(encoder_registers);
    return ESP_OK;
}

esp_err_t dsy_rs485_runtime_read_drive_absolute_encoder_motor_counts(
    int drive_index,
    int64_t *absolute_encoder_motor_counts_out)
{
    uint32_t single_turn_counts = 0U;

    if (absolute_encoder_motor_counts_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = dsy_rs485_runtime_read_drive_absolute_encoder_single_turn_counts(
        drive_index,
        &single_turn_counts);
    if (result != ESP_OK) {
        return result;
    }

    *absolute_encoder_motor_counts_out = (int64_t)single_turn_counts;
    return ESP_OK;
}

esp_err_t dsy_rs485_runtime_read_drive_motion_sample(
    int drive_index,
    dsy_rs485_runtime_motion_sample_t *sample_out)
{
    uint8_t slave_address = 0U;
    uint16_t speed_register = 0U;
    uint16_t encoder_single_turn_registers[2] = {0U, 0U};

    if (sample_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    esp_err_t result = runtime_read_holding_registers(
        slave_address,
        DSY_P18_01_ACTUAL_SPEED_RPM,
        1U,
        &speed_register);
    if (result != ESP_OK) {
        return result;
    }

    result = runtime_read_holding_registers(
        slave_address,
        DSY_P18_32_ABSOLUTE_ENCODER_SINGLE_ROUND_DATA,
        2U,
        encoder_single_turn_registers);
    if (result != ESP_OK) {
        return result;
    }

    const uint32_t high_word_first_count =
        ((uint32_t)encoder_single_turn_registers[0] << 16U) |
        (uint32_t)encoder_single_turn_registers[1];
    const uint32_t low_word_first_count =
        ((uint32_t)encoder_single_turn_registers[1] << 16U) |
        (uint32_t)encoder_single_turn_registers[0];

    sample_out->actual_speed_rpm = (int16_t)speed_register;
    if (high_word_first_count < (uint32_t)CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV) {
        sample_out->encoder_single_turn_counts = high_word_first_count;
    } else if (low_word_first_count < (uint32_t)CONFIG_SERVO_ENCODER_COUNTS_PER_MOTOR_REV) {
        sample_out->encoder_single_turn_counts = low_word_first_count;
    } else {
        sample_out->encoder_single_turn_counts = high_word_first_count;
    }

    return ESP_OK;
}

esp_err_t dsy_rs485_runtime_read_drive_absolute_position_counts(
    int drive_index,
    int32_t *absolute_position_counts_out)
{
    /*
     * Compatibility wrapper retained for older callers. It returns P18.17
     * feedback pulses, while current odometry uses P18.32/P18.33 single-turn data
     * with local modulo unwrapping.
     */
    return dsy_rs485_runtime_read_drive_feedback_pulse_counts(
        drive_index,
        absolute_position_counts_out);
}


static esp_err_t write_parameter_u16_with_settle(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint16_t parameter_value,
    uint32_t settle_delay_us)
{
    esp_err_t write_result = dsy_rs485_write_parameter_u16(
        slave_address,
        parameter_group,
        parameter_index,
        parameter_value);

    if ((write_result == ESP_OK) && (settle_delay_us > 0U)) {
        delay_microseconds(settle_delay_us);
    }

    return write_result;
}

static esp_err_t read_back_and_verify_parameter_u16(
    uint8_t slave_address,
    uint8_t parameter_group,
    uint8_t parameter_index,
    uint16_t expected_value,
    const char *parameter_name)
{
    uint16_t readback_value = 0U;

    esp_err_t read_result = dsy_rs485_read_parameter_u16(
        slave_address,
        parameter_group,
        parameter_index,
        &readback_value);

    if (read_result != ESP_OK) {
        ESP_LOGW(
            dsy_rs485_log_tag,
            "%s readback failed addr=%u result=%s exception=%u",
            parameter_name,
            (unsigned)slave_address,
            esp_err_to_name(read_result),
            (unsigned)dsy_rs485_last_modbus_exception_code);
        return read_result;
    }

    if (readback_value != expected_value) {
        ESP_LOGW(
            dsy_rs485_log_tag,
            "%s readback mismatch addr=%u expected=%u got=%u",
            parameter_name,
            (unsigned)slave_address,
            (unsigned)expected_value,
            (unsigned)readback_value);
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    ESP_LOGI(
        dsy_rs485_log_tag,
        "%s confirmed addr=%u value=%u",
        parameter_name,
        (unsigned)slave_address,
        (unsigned)readback_value);

    return ESP_OK;
}


static esp_err_t read_back_and_verify_parameter_pair_u16(
    uint8_t slave_address,
    uint16_t start_register_address,
    uint16_t expected_value_a,
    uint16_t expected_value_b,
    const char *parameter_name)
{
    uint16_t readback_values[2] = {0U, 0U};

    esp_err_t read_result = dsy_rs485_read_holding_registers(
        slave_address,
        start_register_address,
        2U,
        readback_values);

    if (read_result != ESP_OK) {
        ESP_LOGW(
            dsy_rs485_log_tag,
            "%s pair readback failed addr=%u reg=0x%04X result=%s exception=%u",
            parameter_name,
            (unsigned)slave_address,
            (unsigned)start_register_address,
            esp_err_to_name(read_result),
            (unsigned)dsy_rs485_last_modbus_exception_code);
        return read_result;
    }

    if ((readback_values[0] != expected_value_a) || (readback_values[1] != expected_value_b)) {
        ESP_LOGW(
            dsy_rs485_log_tag,
            "%s pair readback mismatch addr=%u reg=0x%04X expected=[%u,%u] got=[%u,%u]",
            parameter_name,
            (unsigned)slave_address,
            (unsigned)start_register_address,
            (unsigned)expected_value_a,
            (unsigned)expected_value_b,
            (unsigned)readback_values[0],
            (unsigned)readback_values[1]);
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    ESP_LOGI(
        dsy_rs485_log_tag,
        "%s confirmed addr=%u values=[%u,%u]",
        parameter_name,
        (unsigned)slave_address,
        (unsigned)readback_values[0],
        (unsigned)readback_values[1]);

    return ESP_OK;
}
esp_err_t dsy_rs485_set_drive_control_mode(int drive_index, dsy_rs485_control_mode_t control_mode)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");
    return dsy_rs485_write_single_register(slave_address, DSY_P00_00_CONTROL_MODE_SELECTION, (uint16_t)control_mode);
}

esp_err_t dsy_rs485_configure_drive_for_internal_digit_torque_mode_debug(int drive_index)
{
    uint8_t slave_address = 0U;
    uint32_t mode_switch_settle_us = DSY_RS485_SERVO_MODE_SWITCH_SETTLE_US;
    uint32_t parameter_write_settle_us = DSY_RS485_SERVO_PARAMETER_WRITE_SETTLE_US;

    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    ESP_LOGI(
        dsy_rs485_log_tag,
        "starting torque-mode write ladder drive=%d addr=%u",
        drive_index,
        (unsigned)slave_address);
    dsy_rs485_debug_log_servo_enable_line("pre_torque_mode_write");
    (void)dsy_rs485_debug_dump_drive_snapshot(drive_index, "pre_torque_mode_write");
    (void)dsy_rs485_debug_dump_drive_io_config(drive_index, "pre_torque_mode_write");
    (void)dsy_rs485_debug_dump_drive_mode_config(drive_index, "pre_torque_mode_write");

    esp_err_t result = dsy_rs485_write_single_register(
        slave_address,
        DSY_P00_00_CONTROL_MODE_SELECTION,
        (uint16_t)dsy_rs485_control_mode_torque);
    if (result != ESP_OK) {
        ESP_LOGE(
            dsy_rs485_log_tag,
            "set torque mode failed drive=%d addr=%u result=%s exception=%u; dumping drive state",
            drive_index,
            (unsigned)slave_address,
            esp_err_to_name(result),
            (unsigned)dsy_rs485_last_modbus_exception_code);
        (void)dsy_rs485_debug_dump_drive_snapshot(drive_index, "torque_mode_write_failed");
        (void)dsy_rs485_debug_dump_drive_io_config(drive_index, "torque_mode_write_failed");
        (void)dsy_rs485_debug_dump_drive_mode_config(drive_index, "torque_mode_write_failed");
        return result;
    }

    delay_microseconds(mode_switch_settle_us);

    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            0U,
            0U,
            (uint16_t)dsy_rs485_control_mode_torque,
            "P00.00 control mode"),
        dsy_rs485_log_tag,
        "verify torque mode failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            6U,
            0U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set torque source failed");

    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            6U,
            0U,
            0U,
            "P06.00 torque source"),
        dsy_rs485_log_tag,
        "verify torque source failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            6U,
            2U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set torque selection failed");

    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            6U,
            2U,
            0U,
            "P06.02 torque selection"),
        dsy_rs485_log_tag,
        "verify torque selection failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            6U,
            13U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set torque speed-limit source failed");

    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            6U,
            13U,
            0U,
            "P06.13 speed-limit source"),
        dsy_rs485_log_tag,
        "verify torque speed-limit source failed");

    uint16_t torque_speed_limit_values[2] = {200U, 200U};
    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_multiple_registers(
            slave_address,
            DSY_P06_15_POSITIVE_SPEED_LIMIT,
            torque_speed_limit_values,
            2U),
        dsy_rs485_log_tag,
        "P06.15/P06.16 failed");
    delay_microseconds(parameter_write_settle_us);
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_pair_u16(
            slave_address,
            DSY_P06_15_POSITIVE_SPEED_LIMIT,
            200U,
            200U,
            "P06.15/P06.16"),
        dsy_rs485_log_tag,
        "P06.15/P06.16 verify failed");

    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_parameter_s16(
            slave_address,
            6U,
            5U,
            0),
        dsy_rs485_log_tag,
        "P06.05 zero failed");
    delay_microseconds(parameter_write_settle_us);

    int16_t torque_command_readback = 0;
    ESP_RETURN_ON_ERROR(
        dsy_rs485_read_parameter_s16(
            slave_address,
            6U,
            5U,
            &torque_command_readback),
        dsy_rs485_log_tag,
        "P06.05 readback failed");
    if (torque_command_readback != 0) {
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    ESP_LOGI(
        dsy_rs485_log_tag,
        "P06.05 confirmed addr=%u value=%d",
        (unsigned)slave_address,
        (int)torque_command_readback);

    ESP_LOGI(
        dsy_rs485_log_tag,
        "torque-mode write ladder complete drive=%d addr=%u",
        drive_index,
        (unsigned)slave_address);

    return ESP_OK;
}

esp_err_t dsy_rs485_configure_drive_for_internal_digit_torque_mode(int drive_index)
{
    return dsy_rs485_configure_drive_for_internal_digit_torque_mode_debug(drive_index);
}

esp_err_t dsy_rs485_write_drive_torque_command(int drive_index, int16_t torque_command_tenths_percent_rated)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");
    return dsy_rs485_write_single_register(
        slave_address,
        DSY_P06_05_TORQUE_COMMAND_DIGIT_VALUE,
        (uint16_t)torque_command_tenths_percent_rated);
}

esp_err_t dsy_rs485_set_drive_torque_mode_speed_limits(int drive_index, uint16_t positive_speed_limit_rpm, uint16_t negative_speed_limit_rpm)
{
    uint8_t slave_address = 0U;
    uint16_t register_values[2] = {positive_speed_limit_rpm, negative_speed_limit_rpm};
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");
    return dsy_rs485_write_multiple_registers(slave_address, DSY_P06_15_POSITIVE_SPEED_LIMIT, register_values, 2U);
}

esp_err_t dsy_rs485_set_drive_internal_torque_limits(
    int drive_index,
    uint16_t forward_limit_tenths_percent_rated,
    uint16_t backward_limit_tenths_percent_rated)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_single_register(
            slave_address,
            DSY_P06_06_TORQUE_LIMIT_SOURCE_SELECTION,
            0U),
        dsy_rs485_log_tag,
        "set torque-limit source failed");

    uint16_t torque_limit_values[2] = {
        forward_limit_tenths_percent_rated,
        backward_limit_tenths_percent_rated,
    };

    return dsy_rs485_write_multiple_registers(
        slave_address,
        DSY_P06_08_FORWARD_INTERNAL_TORQUE_LIMIT,
        torque_limit_values,
        2U);
}

esp_err_t dsy_rs485_configure_drive_for_internal_digit_speed_mode(
    int drive_index,
    uint16_t forward_speed_limit_rpm,
    uint16_t backward_speed_limit_rpm,
    uint16_t accel_time_ms,
    uint16_t decel_time_ms,
    uint16_t torque_limit_tenths_percent_rated)
{
    uint8_t slave_address = 0U;
    uint32_t mode_switch_settle_us = DSY_RS485_SERVO_MODE_SWITCH_SETTLE_US;
    uint32_t parameter_write_settle_us = DSY_RS485_SERVO_PARAMETER_WRITE_SETTLE_US;

    ESP_RETURN_ON_ERROR(
        validate_drive_index(drive_index, &slave_address),
        dsy_rs485_log_tag,
        "invalid drive index");

    ESP_LOGI(
        dsy_rs485_log_tag,
        "starting speed-mode write ladder drive=%d addr=%u servo_enable_asserted=%d gpio_level=%d active_level_runtime=%d",
        drive_index,
        (unsigned)slave_address,
        dsy_rs485_is_servo_enable_asserted() ? 1 : 0,
        dsy_rs485_get_servo_enable_gpio_level(),
        dsy_rs485_get_servo_enable_active_level_runtime());
    dsy_rs485_debug_log_servo_enable_line("pre_speed_mode_write");

    esp_err_t set_speed_mode_result = write_parameter_u16_with_settle(
        slave_address,
        0U,
        0U,
        (uint16_t)dsy_rs485_control_mode_speed,
        mode_switch_settle_us);
    if (set_speed_mode_result != ESP_OK) {
        ESP_LOGE(
            dsy_rs485_log_tag,
            "set speed mode failed drive=%d addr=%u result=%s exception=%u",
            drive_index,
            (unsigned)slave_address,
            esp_err_to_name(set_speed_mode_result),
            (unsigned)dsy_rs485_last_modbus_exception_code);
        dsy_rs485_debug_log_servo_enable_line("set_speed_mode_failed");
        return set_speed_mode_result;
    }
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            0U,
            0U,
            (uint16_t)dsy_rs485_control_mode_speed,
            "P00.00 speed mode"),
        dsy_rs485_log_tag,
        "verify speed mode failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            5U,
            0U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set P05.00 speed source failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            5U,
            0U,
            0U,
            "P05.00"),
        dsy_rs485_log_tag,
        "verify P05.00 failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            5U,
            2U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set P05.02 speed selection failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            5U,
            2U,
            0U,
            "P05.02"),
        dsy_rs485_log_tag,
        "verify P05.02 failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            5U,
            7U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set P05.07 speed-limit selection failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            5U,
            7U,
            0U,
            "P05.07"),
        dsy_rs485_log_tag,
        "verify P05.07 failed");

    uint16_t speed_limit_values[2] = {
        forward_speed_limit_rpm,
        backward_speed_limit_rpm,
    };
    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_multiple_registers(
            slave_address,
            DSY_P05_08_FORWARD_SPEED_LIMIT_RPM,
            speed_limit_values,
            2U),
        dsy_rs485_log_tag,
        "set speed limits failed");
    delay_microseconds(parameter_write_settle_us);
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_pair_u16(
            slave_address,
            DSY_P05_08_FORWARD_SPEED_LIMIT_RPM,
            forward_speed_limit_rpm,
            backward_speed_limit_rpm,
            "P05.08/P05.09"),
        dsy_rs485_log_tag,
        "verify P05.08/P05.09 failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            5U,
            5U,
            accel_time_ms,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set accel time failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            5U,
            5U,
            accel_time_ms,
            "P05.05"),
        dsy_rs485_log_tag,
        "verify P05.05 failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            5U,
            6U,
            decel_time_ms,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "set decel time failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            5U,
            6U,
            decel_time_ms,
            "P05.06"),
        dsy_rs485_log_tag,
        "verify P05.06 failed");

    ESP_RETURN_ON_ERROR(
        write_parameter_u16_with_settle(
            slave_address,
            6U,
            6U,
            0U,
            parameter_write_settle_us),
        dsy_rs485_log_tag,
        "P06.06 failed");
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_u16(
            slave_address,
            6U,
            6U,
            0U,
            "P06.06"),
        dsy_rs485_log_tag,
        "P06.06 verify failed");

    uint16_t torque_limit_values[2] = {
        torque_limit_tenths_percent_rated,
        torque_limit_tenths_percent_rated,
    };
    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_multiple_registers(
            slave_address,
            DSY_P06_08_FORWARD_INTERNAL_TORQUE_LIMIT,
            torque_limit_values,
            2U),
        dsy_rs485_log_tag,
        "P06.08/P06.09 failed");
    delay_microseconds(parameter_write_settle_us);
    ESP_RETURN_ON_ERROR(
        read_back_and_verify_parameter_pair_u16(
            slave_address,
            DSY_P06_08_FORWARD_INTERNAL_TORQUE_LIMIT,
            torque_limit_tenths_percent_rated,
            torque_limit_tenths_percent_rated,
            "P06.08/P06.09"),
        dsy_rs485_log_tag,
        "P06.08/P06.09 verify failed");

    ESP_RETURN_ON_ERROR(
        dsy_rs485_write_parameter_s16(
            slave_address,
            5U,
            3U,
            0),
        dsy_rs485_log_tag,
        "P05.03 zero failed");
    delay_microseconds(parameter_write_settle_us);

    int16_t speed_command_readback = 0;
    ESP_RETURN_ON_ERROR(
        dsy_rs485_read_parameter_s16(
            slave_address,
            5U,
            3U,
            &speed_command_readback),
        dsy_rs485_log_tag,
        "P05.03 readback failed");
    if (speed_command_readback != 0) {
        return DSY_RS485_ERROR_INVALID_RESPONSE;
    }

    ESP_LOGI(
        dsy_rs485_log_tag,
        "P05.03 confirmed addr=%u value=%d",
        (unsigned)slave_address,
        (int)speed_command_readback);

    ESP_LOGI(
        dsy_rs485_log_tag,
        "speed-mode write ladder complete drive=%d addr=%u",
        drive_index,
        (unsigned)slave_address);

    return ESP_OK;
}

esp_err_t dsy_rs485_write_drive_speed_command_rpm(int drive_index, int16_t speed_command_rpm)
{
    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");
    return dsy_rs485_write_parameter_s16(slave_address, 5U, 3U, speed_command_rpm);
}

static esp_err_t poll_one_drive(int drive_index)
{
    uint8_t slave_address = 0U;
    uint16_t register_value_u16 = 0U;
    int16_t register_value_s16 = 0;
    int32_t register_value_s32 = 0;

    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    dsy_rs485_servo_states[drive_index].slave_address = slave_address;
    dsy_rs485_servo_states[drive_index].last_modbus_exception_code = 0U;

    esp_err_t result = dsy_rs485_read_parameter_u16(slave_address, 18U, 0U, &register_value_u16);
    if (result != ESP_OK) {
        dsy_rs485_servo_states[drive_index].communication_ok = false;
        dsy_rs485_servo_states[drive_index].last_communication_result = result;
        dsy_rs485_servo_states[drive_index].last_modbus_exception_code = dsy_rs485_last_modbus_exception_code;
        return result;
    }
    dsy_rs485_servo_states[drive_index].servo_status_word = register_value_u16;

    ESP_RETURN_ON_ERROR(dsy_rs485_read_parameter_s16(slave_address, 18U, 1U, &register_value_s16), dsy_rs485_log_tag, "actual speed read failed");
    dsy_rs485_servo_states[drive_index].actual_speed_rpm = register_value_s16;

    ESP_RETURN_ON_ERROR(dsy_rs485_read_parameter_u16(slave_address, 18U, 6U, &register_value_u16), dsy_rs485_log_tag, "bus voltage read failed");
    dsy_rs485_servo_states[drive_index].bus_voltage_deci_volts = register_value_u16;

    ESP_RETURN_ON_ERROR(dsy_rs485_read_parameter_u16(slave_address, 18U, 5U, &register_value_u16), dsy_rs485_log_tag, "phase current read failed");
    dsy_rs485_servo_states[drive_index].phase_current_rms_centiamps = register_value_u16;

    ESP_RETURN_ON_ERROR(dsy_rs485_read_parameter_s32(slave_address, 18U, 17U, &register_value_s32), dsy_rs485_log_tag, "feedback pulse counter read failed");
    dsy_rs485_servo_states[drive_index].absolute_position_counts = register_value_s32;

    ESP_RETURN_ON_ERROR(dsy_rs485_read_parameter_s32(slave_address, 18U, 13U, &register_value_s32), dsy_rs485_log_tag, "position deviation read failed");
    dsy_rs485_servo_states[drive_index].position_deviation_counts = register_value_s32;

    dsy_rs485_servo_states[drive_index].communication_ok = true;
    dsy_rs485_servo_states[drive_index].last_communication_result = ESP_OK;
    dsy_rs485_servo_states[drive_index].last_modbus_exception_code = 0U;

    return ESP_OK;
}

void dsy_rs485_servo_task(void *argument)
{
    (void)argument;

    esp_err_t initialize_result = initialize_dsy_rs485_bus();
    if (initialize_result != ESP_OK) {
        ESP_LOGE(dsy_rs485_log_tag, "RS485 init failed: %s", esp_err_to_name(initialize_result));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        for (int drive_index = 0; drive_index < DSY_RS485_SERVO_ACTIVE_DRIVE_COUNT; drive_index++) {
            esp_err_t poll_result = poll_one_drive(drive_index);
            if (poll_result != ESP_OK) {
                ESP_LOGW(
                    dsy_rs485_log_tag,
                    "poll failed drive=%d addr=%u result=%s exception=%u",
                    drive_index,
                    dsy_rs485_servo_slave_addresses[drive_index],
                    esp_err_to_name(poll_result),
                    (unsigned)dsy_rs485_last_modbus_exception_code);
            }
            if (DSY_RS485_SERVO_INTER_DRIVE_POLL_DELAY_US > 0) {
                delay_microseconds(DSY_RS485_SERVO_INTER_DRIVE_POLL_DELAY_US);
            }
        }

        if (DSY_RS485_SERVO_POLL_PERIOD_US > 0) {
            delay_microseconds(DSY_RS485_SERVO_POLL_PERIOD_US);
        }
    }
}


esp_err_t dsy_rs485_wait_for_drive_ready(
    int drive_index,
    int stable_reads_required,
    int timeout_us,
    int poll_interval_us)
{
    if (stable_reads_required <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (timeout_us <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (poll_interval_us <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dsy_rs485_bus_initialized) {
        ESP_RETURN_ON_ERROR(dsy_rs485_initialize_bus(), dsy_rs485_log_tag, "bus init failed");
    }

    uint8_t slave_address = 0U;
    ESP_RETURN_ON_ERROR(validate_drive_index(drive_index, &slave_address), dsy_rs485_log_tag, "invalid drive index");

    const uint64_t start_time_us = (uint64_t)esp_timer_get_time();
    int consecutive_good_reads = 0;

    while (((uint64_t)esp_timer_get_time() - start_time_us) < (uint64_t)timeout_us) {
        uint16_t status_word = 0U;
        uint16_t bus_voltage_deci_volts = 0U;
        int32_t feedback_pulse_counts = 0;

        esp_err_t status_result =
            dsy_rs485_read_parameter_u16(slave_address, 18U, 0U, &status_word);
        esp_err_t bus_result =
            dsy_rs485_read_parameter_u16(slave_address, 18U, 6U, &bus_voltage_deci_volts);
        esp_err_t position_result =
            dsy_rs485_read_parameter_s32(slave_address, 18U, 17U, &feedback_pulse_counts);

        if ((status_result == ESP_OK) &&
            (bus_result == ESP_OK) &&
            (position_result == ESP_OK) &&
            (bus_voltage_deci_volts > 0U)) {
            dsy_rs485_servo_states[drive_index].slave_address = slave_address;
            dsy_rs485_servo_states[drive_index].communication_ok = true;
            dsy_rs485_servo_states[drive_index].last_communication_result = ESP_OK;
            dsy_rs485_servo_states[drive_index].last_modbus_exception_code = 0U;
            dsy_rs485_servo_states[drive_index].servo_status_word = status_word;
            dsy_rs485_servo_states[drive_index].bus_voltage_deci_volts = bus_voltage_deci_volts;
            dsy_rs485_servo_states[drive_index].absolute_position_counts = feedback_pulse_counts;

            consecutive_good_reads++;
            if (consecutive_good_reads >= stable_reads_required) {
                ESP_LOGI(
                    dsy_rs485_log_tag,
                    "drive ready drive=%d addr=%u stable_reads=%d status=0x%04X bus=%.1fV feedback_pulses=%ld gpio_level=%d active_level_runtime=%d",
                    drive_index,
                    (unsigned)slave_address,
                    consecutive_good_reads,
                    (unsigned)status_word,
                    0.1f * (float)bus_voltage_deci_volts,
                    (long)feedback_pulse_counts,
                    dsy_rs485_get_servo_enable_gpio_level(),
                    dsy_rs485_get_servo_enable_active_level_runtime());
                return ESP_OK;
            }
        } else {
            dsy_rs485_servo_states[drive_index].communication_ok = false;
            dsy_rs485_servo_states[drive_index].last_communication_result =
                (status_result != ESP_OK) ? status_result :
                (bus_result != ESP_OK) ? bus_result :
                position_result;
            dsy_rs485_servo_states[drive_index].last_modbus_exception_code = dsy_rs485_last_modbus_exception_code;
            consecutive_good_reads = 0;
        }

        delay_microseconds((uint32_t)poll_interval_us);
    }

    ESP_LOGW(
        dsy_rs485_log_tag,
        "drive ready wait timed out drive=%d addr=%u timeout_us=%d gpio_level=%d active_level_runtime=%d exception=%u",
        drive_index,
        (unsigned)slave_address,
        timeout_us,
        dsy_rs485_get_servo_enable_gpio_level(),
        dsy_rs485_get_servo_enable_active_level_runtime(),
        (unsigned)dsy_rs485_last_modbus_exception_code);
    (void)dsy_rs485_debug_dump_drive_snapshot(drive_index, "drive_ready_timeout");
    (void)dsy_rs485_debug_dump_drive_io_config(drive_index, "drive_ready_timeout");
    (void)dsy_rs485_debug_dump_drive_mode_config(drive_index, "drive_ready_timeout");

    return ESP_ERR_TIMEOUT;
}


esp_err_t dsy_rs485_poll_one_drive(int drive_index)
{
    if (!dsy_rs485_bus_initialized) {
        ESP_RETURN_ON_ERROR(dsy_rs485_initialize_bus(), dsy_rs485_log_tag, "bus init failed");
    }
    return poll_one_drive(drive_index);
}
