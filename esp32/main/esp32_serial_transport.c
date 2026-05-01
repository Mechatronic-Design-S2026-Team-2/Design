/**
 * @file esp32_serial_transport.c
 * @brief micro-ROS custom UART transport for ESP-IDF.
 */

#include "esp32_serial_transport.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_US
#ifdef CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_MS
#define CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_US ((uint32_t)CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_US 2000
#endif
#endif

#ifndef CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_US
#ifdef CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_MS
#define CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_US ((uint32_t)CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_US 100000
#endif
#endif

#ifndef CONFIG_MICRO_ROS_UART_OPEN_FLUSH_INPUT
#define CONFIG_MICRO_ROS_UART_OPEN_FLUSH_INPUT 1
#endif

#ifndef CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_US
#ifdef CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_MS
#define CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_US ((uint32_t)CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_MS * 1000U)
#else
#define CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_US 0
#endif
#endif

#ifndef CONFIG_MICRO_ROS_SILENCE_CONSOLE_LOGGING_ON_START
#define CONFIG_MICRO_ROS_SILENCE_CONSOLE_LOGGING_ON_START 1
#endif

static const char *esp32_serial_transport_log_tag = "uros_uart";
static bool esp32_serial_transport_initialized = false;
static bool esp32_serial_console_logging_silenced = false;
static vprintf_like_t esp32_serial_previous_log_vprintf = NULL;

static int esp32_serial_silent_log_vprintf(const char *format, va_list arguments)
{
    (void)format;
    (void)arguments;
    return 0;
}

static esp32_serial_transport_config_t *get_transport_config(struct uxrCustomTransport *transport)
{
    if ((transport == NULL) || (transport->args == NULL)) {
        return NULL;
    }

    return (esp32_serial_transport_config_t *)transport->args;
}

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

void esp32_serial_silence_console_logging_if_requested(
    const esp32_serial_transport_config_t *transport_config)
{
#if CONFIG_MICRO_ROS_SILENCE_CONSOLE_LOGGING_ON_START
    if ((transport_config == NULL) ||
        (transport_config->uart_port != UART_NUM_0) ||
        esp32_serial_console_logging_silenced) {
        return;
    }

    (void)uart_wait_tx_done(
        transport_config->uart_port,
        ticks_from_microseconds(CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_US));
    esp32_serial_previous_log_vprintf = esp_log_set_vprintf(esp32_serial_silent_log_vprintf);
    (void)esp32_serial_previous_log_vprintf;
    esp_log_level_set("*", ESP_LOG_NONE);
    esp32_serial_console_logging_silenced = true;
#else
    (void)transport_config;
#endif
}

bool esp32_serial_is_console_logging_silenced(void)
{
    return esp32_serial_console_logging_silenced;
}

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    esp32_serial_transport_config_t *config = get_transport_config(transport);
    if (config == NULL) {
        return false;
    }

    if (!esp32_serial_transport_initialized) {
        uart_config_t uart_configuration = {
            .baud_rate = config->baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        if (uart_driver_install(
                config->uart_port,
                config->rx_buffer_size,
                config->tx_buffer_size,
                0,
                NULL,
                0) != ESP_OK) {
            return false;
        }

        if (uart_param_config(config->uart_port, &uart_configuration) != ESP_OK) {
            return false;
        }

        if (uart_set_pin(
                config->uart_port,
                config->tx_pin,
                config->rx_pin,
                UART_PIN_NO_CHANGE,
                UART_PIN_NO_CHANGE) != ESP_OK) {
            return false;
        }

#if CONFIG_MICRO_ROS_UART_OPEN_FLUSH_INPUT
        uart_flush_input(config->uart_port);
#endif
        if (CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_US > 0) {
            delay_microseconds(CONFIG_MICRO_ROS_UART_POST_OPEN_SETTLE_US);
        }
        esp32_serial_transport_initialized = true;

        if (!esp32_serial_console_logging_silenced) {
            ESP_LOGI(
                esp32_serial_transport_log_tag,
                "micro-ROS UART transport ready on UART%d TX=%d RX=%d baud=%d",
                (int)config->uart_port,
                config->tx_pin,
                config->rx_pin,
                config->baud_rate);
        }
    }

    return true;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    esp32_serial_transport_config_t *config = get_transport_config(transport);
    if (config == NULL) {
        return false;
    }

    uart_flush(config->uart_port);
    return true;
}

size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code)
{
    esp32_serial_transport_config_t *config = get_transport_config(transport);
    if ((config == NULL) || (buffer == NULL) || (length == 0U)) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    int write_count = uart_write_bytes(config->uart_port, (const char *)buffer, length);
    if (write_count < 0) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    if (uart_wait_tx_done(
            config->uart_port,
            ticks_from_microseconds(CONFIG_MICRO_ROS_UART_WRITE_TIMEOUT_US)) != ESP_OK) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
    }

    return (size_t)write_count;
}

size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout_milliseconds,
    uint8_t *error_code)
{
    esp32_serial_transport_config_t *config = get_transport_config(transport);
    if ((config == NULL) || (buffer == NULL) || (length == 0U)) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    const uint64_t start_time_us = (uint64_t)esp_timer_get_time();
    const uint32_t requested_timeout_us =
        (timeout_milliseconds > 0) ? (uint32_t)timeout_milliseconds * 1000U : 0U;
    size_t total_bytes_read = 0U;

    while (total_bytes_read < length) {
        uint32_t remaining_timeout_us = requested_timeout_us;
        if (timeout_milliseconds > 0) {
            const uint64_t elapsed_us = (uint64_t)esp_timer_get_time() - start_time_us;
            if (elapsed_us >= (uint64_t)requested_timeout_us) {
                break;
            }
            remaining_timeout_us = requested_timeout_us - (uint32_t)elapsed_us;
        }

        uint32_t chunk_timeout_us = remaining_timeout_us;
        if (timeout_milliseconds > 0) {
            const uint32_t configured_chunk_timeout_us = CONFIG_MICRO_ROS_UART_READ_CHUNK_TIMEOUT_US;
            if ((configured_chunk_timeout_us > 0U) &&
                (configured_chunk_timeout_us < chunk_timeout_us)) {
                chunk_timeout_us = configured_chunk_timeout_us;
            }
        }

        int read_count = uart_read_bytes(
            config->uart_port,
            buffer + total_bytes_read,
            length - total_bytes_read,
            ticks_from_microseconds(chunk_timeout_us));

        if (read_count < 0) {
            if (error_code != NULL) {
                *error_code = 1U;
            }
            return 0U;
        }

        if (read_count == 0) {
            if (timeout_milliseconds <= 0) {
                break;
            }
            continue;
        }

        total_bytes_read += (size_t)read_count;

        if (timeout_milliseconds <= 0) {
            break;
        }
    }

    return total_bytes_read;
}
