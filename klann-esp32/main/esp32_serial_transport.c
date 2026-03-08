#include <uxr/client/transport.h>

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define UART_TXD_GPIO_NUM  (CONFIG_MICROROS_UART_TXD)
#define UART_RXD_GPIO_NUM  (CONFIG_MICROROS_UART_RXD)
#define UART_RTS_GPIO_NUM  (CONFIG_MICROROS_UART_RTS)
#define UART_CTS_GPIO_NUM  (CONFIG_MICROROS_UART_CTS)

/* ---------- micro-ROS transport config ---------- */

#define MICRO_ROS_UART_BAUD_RATE        115200
#define MICRO_ROS_UART_BUFFER_SIZE      512

/**
 * @brief Transport args -> UART port helper.
 *
 * Decode custom transport args as UART port pointer.
 *
 * @param transport
 *     micro-ROS custom transport pointer.
 *     Carries caller-supplied args pointer.
 *     Used to recover UART port handle.
 *
 * @return UART port pointer on success.
 *     Caller-provided transport arg cast.
 *     Used by UART helper functions.
 * @return NULL on invalid transport.
 *     Missing transport or args.
 *     Used for early-fail path.
 */
static uart_port_t *get_uart_port_from_transport(struct uxrCustomTransport *transport)
{
    if ((transport == NULL) || (transport->args == NULL)) {
        return NULL;
    }

    return (uart_port_t *)transport->args;
}

/**
 * @brief micro-ROS UART open.
 *
 * Configure UART peripheral, route pins, install driver.
 *
 * @param transport
 *     micro-ROS custom transport pointer.
 *     Carries UART port in args field.
 *     Used for UART open path.
 *
 * @return true on success.
 *     UART transport ready.
 *     Normal open path.
 * @return false on failure.
 *     UART config or install failed.
 *     Used by micro-ROS transport layer.
 */
bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    uart_port_t *uart_port = get_uart_port_from_transport(transport);

    if (uart_port == NULL) {
        return false;
    }

    uart_config_t uart_configuration = {
        .baud_rate = MICRO_ROS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(*uart_port, &uart_configuration) != ESP_OK) {
        return false;
    }

    if (uart_set_pin(
            *uart_port,
            UART_TXD_GPIO_NUM,
            UART_RXD_GPIO_NUM,
            UART_RTS_GPIO_NUM,
            UART_CTS_GPIO_NUM) != ESP_OK) {
        return false;
    }

    if (uart_driver_install(
            *uart_port,
            MICRO_ROS_UART_BUFFER_SIZE * 2,
            0,
            0,
            NULL,
            0) != ESP_OK) {
        return false;
    }

    return true;
}

/**
 * @brief micro-ROS UART close.
 *
 * Delete UART driver for selected port.
 *
 * @param transport
 *     micro-ROS custom transport pointer.
 *     Carries UART port in args field.
 *     Used for UART close path.
 *
 * @return true on success.
 *     UART driver removed.
 *     Normal close path.
 * @return false on failure.
 *     Invalid args or driver delete failed.
 *     Used by micro-ROS transport layer.
 */
bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    uart_port_t *uart_port = get_uart_port_from_transport(transport);

    if (uart_port == NULL) {
        return false;
    }

    return (uart_driver_delete(*uart_port) == ESP_OK);
}

/**
 * @brief micro-ROS UART write.
 *
 * Write one byte range to selected UART port.
 *
 * @param transport
 *     micro-ROS custom transport pointer.
 *     Carries UART port in args field.
 *     Used for UART write path.
 * @param buffer
 *     Byte buffer pointer.
 *     Source payload for UART TX.
 *     Used by uart_write_bytes.
 * @param length
 *     Requested byte count.
 *     Number of bytes to transmit.
 *     Used by uart_write_bytes.
 * @param error_code
 *     micro-ROS transport error pointer.
 *     Set nonzero on local failure.
 *     Used by upper transport layer.
 *
 * @return Number of bytes written on success.
 *     UART accepted byte count.
 *     Used by micro-ROS transport layer.
 * @return 0 on failure.
 *     Invalid args or UART write failure.
 *     Used as transport error signal.
 */
size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code)
{
    uart_port_t *uart_port = get_uart_port_from_transport(transport);

    if (error_code != NULL) {
        *error_code = 0;
    }

    if ((uart_port == NULL) || (buffer == NULL)) {
        if (error_code != NULL) {
            *error_code = 1;
        }
        return 0;
    }

    int transmitted_byte_count = uart_write_bytes(*uart_port, (const char *)buffer, length);

    if (transmitted_byte_count < 0) {
        if (error_code != NULL) {
            *error_code = 1;
        }
        return 0;
    }

    return (size_t)transmitted_byte_count;
}

/**
 * @brief micro-ROS UART read.
 *
 * Read up to one byte range from selected UART port.
 *
 * @param transport
 *     micro-ROS custom transport pointer.
 *     Carries UART port in args field.
 *     Used for UART read path.
 * @param buffer
 *     Destination byte buffer.
 *     Receives UART RX payload.
 *     Used by uart_read_bytes.
 * @param length
 *     Maximum requested byte count.
 *     Upper read bound.
 *     Used by uart_read_bytes.
 * @param timeout
 *     Read timeout in milliseconds.
 *     Converted to RTOS ticks.
 *     Used by uart_read_bytes.
 * @param error_code
 *     micro-ROS transport error pointer.
 *     Set nonzero on local failure.
 *     Used by upper transport layer.
 *
 * @return Number of bytes read on success.
 *     UART delivered byte count.
 *     Used by micro-ROS transport layer.
 * @return 0 on failure or timeout.
 *     Invalid args or UART read failure.
 *     Used as transport short-read signal.
 */
size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout,
    uint8_t *error_code)
{
    uart_port_t *uart_port = get_uart_port_from_transport(transport);

    if (error_code != NULL) {
        *error_code = 0;
    }

    if ((uart_port == NULL) || (buffer == NULL)) {
        if (error_code != NULL) {
            *error_code = 1;
        }
        return 0;
    }

    int received_byte_count = uart_read_bytes(
        *uart_port,
        buffer,
        length,
        pdMS_TO_TICKS(timeout));

    if (received_byte_count < 0) {
        if (error_code != NULL) {
            *error_code = 1;
        }
        return 0;
    }

    return (size_t)received_byte_count;
}
