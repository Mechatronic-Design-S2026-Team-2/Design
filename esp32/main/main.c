/**
 * @file main.c
 * @brief Direct motor-RPM micro-ROS bridge main entry with precharge /
 * contactor control and shared-servo enable handling.
 */

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rmw_microros/rmw_microros.h"

#include "dsy_rs485_servo.h"
#include "esp32_serial_transport.h"
#include "hexapod_hw_config.h"
#include "motor_power_manager.h"
#include "motor_velocity_bridge.h"

#ifndef CONFIG_MICRO_ROS_UART_BAUD_RATE
#define CONFIG_MICRO_ROS_UART_BAUD_RATE 115200
#endif

#ifndef CONFIG_MICRO_ROS_UART_RX_BUFFER_SIZE
#define CONFIG_MICRO_ROS_UART_RX_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_MICRO_ROS_UART_TX_BUFFER_SIZE
#define CONFIG_MICRO_ROS_UART_TX_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 8192
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#ifndef CONFIG_MICRO_ROS_TASK_CORE
#define CONFIG_MICRO_ROS_TASK_CORE 0
#endif

#ifndef CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US
#ifdef CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_MS
#define CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US ((uint32_t)CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_MS * 1000U)
#else
#define CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US 2000000
#endif
#endif

#ifndef CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US
#ifdef CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_MS
#define CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US ((uint32_t)CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_MS * 1000U)
#else
#define CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US 5000000
#endif
#endif

#ifndef CONFIG_POWER_PRECHARGE_FAILURE_RETRY_FOREVER
#define CONFIG_POWER_PRECHARGE_FAILURE_RETRY_FOREVER 1
#endif

#ifndef CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES
#define CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES 3
#endif

#ifndef CONFIG_POWER_PRECHARGE_REBOOT_AFTER_FAILED_RETRIES
#define CONFIG_POWER_PRECHARGE_REBOOT_AFTER_FAILED_RETRIES 0
#endif

#define MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US         CONFIG_MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US
#define MOTOR_POWER_MANAGER_TASK_STACK_BYTES        4096
#define MOTOR_POWER_MANAGER_TASK_PRIORITY           3

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


static const char *main_log_tag = "main";
static esp32_serial_transport_config_t micro_ros_uart_transport_config = {
    .uart_port = HEXAPOD_MICROROS_UART_PORT,
    .tx_pin = HEXAPOD_MICROROS_UART_TX_GPIO,
    .rx_pin = HEXAPOD_MICROROS_UART_RX_GPIO,
    .baud_rate = CONFIG_MICRO_ROS_UART_BAUD_RATE,
    .rx_buffer_size = CONFIG_MICRO_ROS_UART_RX_BUFFER_SIZE,
    .tx_buffer_size = CONFIG_MICRO_ROS_UART_TX_BUFFER_SIZE,
};

static void force_startup_power_outputs_safe_off(void)
{
    (void)motor_power_manager_set_precharge_enabled(false);
    (void)motor_power_manager_set_contactor_enabled(false);
    (void)dsy_rs485_set_servo_enable_output(false);
}

static void wait_for_power_path_ready_before_motor_bus(void)
{
    uint32_t failed_attempt_count = 0U;

    while (true) {
        const esp_err_t startup_result = motor_power_manager_run_startup_sequence();
        if (startup_result == ESP_OK) {
            return;
        }

        failed_attempt_count++;
        force_startup_power_outputs_safe_off();

        const motor_power_manager_status_t power_status = get_motor_power_manager_status();
        ESP_LOGE(
            main_log_tag,
            "power-path startup failed attempt=%lu result=%s adc_ready=%d bus=%.2fV sensor=%dmV raw=%d elapsed=%luus; outputs forced off",
            (unsigned long)failed_attempt_count,
            esp_err_to_name(startup_result),
            power_status.adc_ready ? 1 : 0,
            power_status.latest_bus_voltage_v,
            power_status.latest_sensor_millivolts,
            power_status.latest_raw_adc_count,
            (unsigned long)power_status.precharge_elapsed_us);

#if CONFIG_POWER_PRECHARGE_FAILURE_RETRY_FOREVER
        ESP_LOGW(
            main_log_tag,
            "retrying power-path startup in %d us",
            (int)CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US);
        delay_microseconds(CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US);
#else
        if (failed_attempt_count < (uint32_t)CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES) {
            ESP_LOGW(
                main_log_tag,
                "retrying power-path startup in %d us (%lu/%d failures)",
                (int)CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US,
                (unsigned long)failed_attempt_count,
                (int)CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES);
            delay_microseconds(CONFIG_POWER_PRECHARGE_FAILURE_RETRY_DELAY_US);
        } else {
#if CONFIG_POWER_PRECHARGE_REBOOT_AFTER_FAILED_RETRIES
            ESP_LOGE(
                main_log_tag,
                "precharge failed %d times; rebooting because menuconfig requests reboot",
                (int)CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES);
            delay_microseconds(250000U);
            esp_restart();
#else
            ESP_LOGE(
                main_log_tag,
                "precharge failed %d times; entering fail-stop with precharge, contactor, and servo-enable off",
                (int)CONFIG_POWER_PRECHARGE_FAILURE_MAX_RETRIES);
            while (true) {
                force_startup_power_outputs_safe_off();
                delay_microseconds(1000000U);
            }
#endif
        }
#endif
    }
}

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if ((nvs_result != ESP_OK) &&
        (nvs_result != ESP_ERR_NVS_NO_FREE_PAGES) &&
        (nvs_result != ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_LOGE(main_log_tag, "nvs_flash_init failed: %s", esp_err_to_name(nvs_result));
        return;
    }

    if ((nvs_result == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    dsy_rs485_restore_default_servo_enable_active_level();
    ESP_ERROR_CHECK(dsy_rs485_initialize_servo_enable_output());
    ESP_ERROR_CHECK(dsy_rs485_set_servo_enable_output(false));
    ESP_LOGI(
        main_log_tag,
        "startup forced shared servo-enable OFF gpio_level=%d active_level_cfg=%d active_level_runtime=%d gpio=%d",
        dsy_rs485_get_servo_enable_gpio_level(),
        (int)HEXAPOD_SERVO_ENABLE_ACTIVE_LEVEL,
        dsy_rs485_get_servo_enable_active_level_runtime(),
        (int)HEXAPOD_SERVO_ENABLE_GPIO);
    ESP_ERROR_CHECK(motor_power_manager_initialize());

    xTaskCreate(
        motor_power_manager_task,
        "motor_power_manager_task",
        MOTOR_POWER_MANAGER_TASK_STACK_BYTES,
        NULL,
        MOTOR_POWER_MANAGER_TASK_PRIORITY,
        NULL);

    wait_for_power_path_ready_before_motor_bus();

    while (true) {
        const esp_err_t prepare_result = motor_velocity_bridge_prepare_drives_before_ros();
        if (prepare_result == ESP_OK) {
            break;
        }

        dsy_rs485_restore_default_servo_enable_active_level();
        (void)dsy_rs485_set_servo_enable_output(false);
        dsy_rs485_debug_log_servo_enable_line("main_prepare_failed_forced_off");
        ESP_LOGW(
            main_log_tag,
            "pre-ROS motor bridge startup failed: %s; forced servo-enable GPIO %d back to servo-off state before retry in %d us",
            esp_err_to_name(prepare_result),
            (int)HEXAPOD_SERVO_ENABLE_GPIO,
            MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US);
        delay_microseconds(MOTOR_BRIDGE_PREPARE_RETRY_DELAY_US);
    }

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&micro_ros_uart_transport_config,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transport must be built with RMW_UXRCE_TRANSPORT=custom
#endif

    ESP_LOGI(
        main_log_tag,
        "starting direct motor-RPM bridge on UART%d baud=%d precharge=%d contactor=%d bus_adc=%d servo_en=%d ros_core=%d",
        (int)micro_ros_uart_transport_config.uart_port,
        micro_ros_uart_transport_config.baud_rate,
        (int)HEXAPOD_PRECHARGE_SSR_GPIO,
        (int)HEXAPOD_CONTACTOR_SSR_GPIO,
        (int)HEXAPOD_PRECHARGE_SENSE_ADC_GPIO,
        (int)HEXAPOD_SERVO_ENABLE_GPIO,
        CONFIG_MICRO_ROS_TASK_CORE);

    esp32_serial_silence_console_logging_if_requested(&micro_ros_uart_transport_config);

    xTaskCreatePinnedToCore(
        motor_velocity_bridge_task,
        "motor_velocity_bridge_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL,
        CONFIG_MICRO_ROS_TASK_CORE);
}
