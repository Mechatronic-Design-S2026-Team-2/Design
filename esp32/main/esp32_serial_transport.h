/**
 * @file esp32_serial_transport.h
 * @brief micro-ROS custom UART transport for ESP-IDF.
 */

#ifndef ESP32_SERIAL_TRANSPORT_H
#define ESP32_SERIAL_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

typedef struct
{
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int rx_buffer_size;
    int tx_buffer_size;
} esp32_serial_transport_config_t;

bool esp32_serial_open(struct uxrCustomTransport *transport);
bool esp32_serial_close(struct uxrCustomTransport *transport);
size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code);
size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout_milliseconds,
    uint8_t *error_code);
void esp32_serial_silence_console_logging_if_requested(
    const esp32_serial_transport_config_t *transport_config);
bool esp32_serial_is_console_logging_silenced(void);

#ifdef __cplusplus
}
#endif

#endif
