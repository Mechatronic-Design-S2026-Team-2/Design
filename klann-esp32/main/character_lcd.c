/**
 * @file character_lcd.c
 * @brief HD44780-compatible 16x2 character LCD driver.
 *
 * Implements GPIO-based 4-bit LCD initialization and text output for the
 * ESP32 status display. All writes are synchronous and intended for low-rate
 * status updates rather than high-throughput rendering.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"

#include "character_lcd.h"

/*
 * ESP-IDF refs:
 * - GPIO driver: driver/gpio.h
 * - esp_rom_delay_us: esp_rom_sys.h
 *
 * HD44780 refs:
 * - 4-bit init by instruction
 * - instruction register / data register
 * - DDRAM addressing
 */

/* ---------- Character LCD config ---------- */

#define CHARACTER_LCD_RS_GPIO_NUM                      25
#define CHARACTER_LCD_E_GPIO_NUM                       26

#define CHARACTER_LCD_D4_GPIO_NUM                      18
#define CHARACTER_LCD_D5_GPIO_NUM                      27
#define CHARACTER_LCD_D6_GPIO_NUM                      14
#define CHARACTER_LCD_D7_GPIO_NUM                      13

/* Command timing. */
#define CHARACTER_LCD_POWER_ON_DELAY_MS                50
#define CHARACTER_LCD_INIT_STEP_DELAY_MS               5
#define CHARACTER_LCD_INIT_STEP_DELAY_US               500
#define CHARACTER_LCD_ENABLE_PULSE_WIDTH_US            5
#define CHARACTER_LCD_ENABLE_CYCLE_DELAY_US            5
#define CHARACTER_LCD_STANDARD_COMMAND_DELAY_US        100
#define CHARACTER_LCD_CLEAR_COMMAND_DELAY_US           3000
#define CHARACTER_LCD_HOME_COMMAND_DELAY_US            3000

/* DDRAM line starts for 2-line modules. */
static const uint8_t character_lcd_row_start_addresses[CHARACTER_LCD_NUM_ROWS] = {
    0x00,
    0x40
};

/* ---------- Character LCD instruction set ---------- */

#define CHARACTER_LCD_COMMAND_CLEAR_DISPLAY            0x01
#define CHARACTER_LCD_COMMAND_RETURN_HOME              0x02
#define CHARACTER_LCD_COMMAND_ENTRY_MODE_SET           0x04
#define CHARACTER_LCD_COMMAND_DISPLAY_CONTROL          0x08
#define CHARACTER_LCD_COMMAND_FUNCTION_SET             0x20
#define CHARACTER_LCD_COMMAND_SET_DDRAM_ADDRESS        0x80

/* Entry mode bits. */
#define CHARACTER_LCD_ENTRY_INCREMENT                  0x02
#define CHARACTER_LCD_ENTRY_SHIFT_OFF                  0x00

/* Display control bits. */
#define CHARACTER_LCD_DISPLAY_ON                       0x04
#define CHARACTER_LCD_CURSOR_OFF                       0x00
#define CHARACTER_LCD_BLINK_OFF                        0x00

/* Function set bits. */
#define CHARACTER_LCD_FUNCTION_SET_4_BIT               0x00
#define CHARACTER_LCD_FUNCTION_SET_2_LINE              0x08
#define CHARACTER_LCD_FUNCTION_SET_5X8_FONT            0x00

/* ---------- Character LCD state ---------- */

static const char *character_lcd_log_tag = "character_lcd";

static const gpio_num_t character_lcd_data_gpio_numbers[4] = {
    CHARACTER_LCD_D4_GPIO_NUM,
    CHARACTER_LCD_D5_GPIO_NUM,
    CHARACTER_LCD_D6_GPIO_NUM,
    CHARACTER_LCD_D7_GPIO_NUM
};

static bool character_lcd_is_initialized = false;

/**
 * @brief Short LCD delay helper.
 *
 * Delay in microseconds for LCD pulse / command timing.
 *
 * @param delay_microseconds
 *     Delay interval in microseconds.
 *     Used for pulse width and command settle.
 *     Passed to ROM delay helper.
 *
 * @return None.
 */
static void delay_character_lcd_microseconds(uint32_t delay_microseconds)
{
    esp_rom_delay_us(delay_microseconds);
}

/**
 * @brief One GPIO level write helper.
 *
 * Set one LCD GPIO pin high or low.
 *
 * @param gpio_number
 *     ESP32 GPIO number.
 *     Target LCD control or data pin.
 *     Used as gpio_set_level target.
 * @param pin_level
 *     Logic level to drive.
 *     0 for low, nonzero for high.
 *     Used as gpio_set_level value.
 *
 * @return None.
 */
static void set_character_lcd_gpio_level(gpio_num_t gpio_number, uint32_t pin_level)
{
    ESP_ERROR_CHECK(gpio_set_level(gpio_number, pin_level));
}

/**
 * @brief D4..D7 write helper.
 *
 * Drive one 4-bit nibble onto LCD data pins.
 *
 * @param nibble_value
 *     Low 4 bits to send.
 *     Bit 0 -> D4, bit 1 -> D5, bit 2 -> D6, bit 3 -> D7.
 *     Used for 4-bit command / data transfer.
 *
 * @return None.
 */
static void set_character_lcd_data_lines_from_nibble(uint8_t nibble_value)
{
    set_character_lcd_gpio_level(CHARACTER_LCD_D4_GPIO_NUM, (nibble_value >> 0) & 0x01U);
    set_character_lcd_gpio_level(CHARACTER_LCD_D5_GPIO_NUM, (nibble_value >> 1) & 0x01U);
    set_character_lcd_gpio_level(CHARACTER_LCD_D6_GPIO_NUM, (nibble_value >> 2) & 0x01U);
    set_character_lcd_gpio_level(CHARACTER_LCD_D7_GPIO_NUM, (nibble_value >> 3) & 0x01U);
}

/**
 * @brief LCD enable pulse helper.
 *
 * Toggle E high then low to latch current nibble.
 *
 * @param None.
 *
 * @return None.
 */
static void pulse_character_lcd_enable(void)
{
    set_character_lcd_gpio_level(CHARACTER_LCD_E_GPIO_NUM, 1U);
    delay_character_lcd_microseconds(CHARACTER_LCD_ENABLE_PULSE_WIDTH_US);

    set_character_lcd_gpio_level(CHARACTER_LCD_E_GPIO_NUM, 0U);
    delay_character_lcd_microseconds(CHARACTER_LCD_ENABLE_CYCLE_DELAY_US);
}

/**
 * @brief One nibble write helper.
 *
 * Present nibble on D4..D7 and latch with E pulse.
 *
 * @param nibble_value
 *     Low 4 bits to send.
 *     Used as one half-byte LCD transfer.
 *     Written onto D4..D7.
 *
 * @return None.
 */
static void write_character_lcd_nibble(uint8_t nibble_value)
{
    set_character_lcd_data_lines_from_nibble(nibble_value & 0x0FU);
    pulse_character_lcd_enable();
}

/**
 * @brief One byte transfer helper.
 *
 * Send one command or data byte in 4-bit mode.
 *
 * @param register_select_is_data
 *     LCD RS level.
 *     false -> instruction register, true -> data register.
 *     Selects command vs character write.
 * @param byte_value
 *     Full 8-bit LCD payload.
 *     Sent high nibble first, then low nibble.
 *     Used for command or data transfer.
 *
 * @return None.
 */
static void write_character_lcd_byte(bool register_select_is_data, uint8_t byte_value)
{
    set_character_lcd_gpio_level(
        CHARACTER_LCD_RS_GPIO_NUM,
        register_select_is_data ? 1U : 0U);

    write_character_lcd_nibble((byte_value >> 4) & 0x0FU);
    write_character_lcd_nibble(byte_value & 0x0FU);

    delay_character_lcd_microseconds(CHARACTER_LCD_STANDARD_COMMAND_DELAY_US);
}

/**
 * @brief One command write helper.
 *
 * Send one instruction byte to LCD instruction register.
 *
 * @param command_value
 *     LCD instruction byte.
 *     Used for mode, cursor, clear, and address control.
 *     Sent with RS low.
 *
 * @return None.
 */
static void write_character_lcd_command_internal(uint8_t command_value)
{
    write_character_lcd_byte(false, command_value);

    if (command_value == CHARACTER_LCD_COMMAND_CLEAR_DISPLAY) {
        delay_character_lcd_microseconds(CHARACTER_LCD_CLEAR_COMMAND_DELAY_US);
    } else if (command_value == CHARACTER_LCD_COMMAND_RETURN_HOME) {
        delay_character_lcd_microseconds(CHARACTER_LCD_HOME_COMMAND_DELAY_US);
    }
}

/**
 * @brief One data-byte write helper.
 *
 * Send one character byte to LCD data register.
 *
 * @param data_value
 *     LCD data byte.
 *     Typically ASCII character code.
 *     Sent with RS high.
 *
 * @return None.
 */
static void write_character_lcd_data_byte(uint8_t data_value)
{
    write_character_lcd_byte(true, data_value);
}

/**
 * @brief 4-bit init-sequence helper.
 *
 * Force LCD into known 4-bit state after power-up.
 *
 * @param None.
 *
 * @return None.
 */
static void initialize_character_lcd_4_bit_mode_sequence(void)
{
    /* Power-up settle. */
    vTaskDelay(pdMS_TO_TICKS(CHARACTER_LCD_POWER_ON_DELAY_MS));

    set_character_lcd_gpio_level(CHARACTER_LCD_RS_GPIO_NUM, 0U);
    set_character_lcd_gpio_level(CHARACTER_LCD_E_GPIO_NUM, 0U);

    /* Force 8-bit path three times. */
    write_character_lcd_nibble(0x03U);
    vTaskDelay(pdMS_TO_TICKS(CHARACTER_LCD_INIT_STEP_DELAY_MS));

    write_character_lcd_nibble(0x03U);
    vTaskDelay(pdMS_TO_TICKS(CHARACTER_LCD_INIT_STEP_DELAY_MS));

    write_character_lcd_nibble(0x03U);
    delay_character_lcd_microseconds(CHARACTER_LCD_INIT_STEP_DELAY_US);

    /* Switch to 4-bit path. */
    write_character_lcd_nibble(0x02U);
    delay_character_lcd_microseconds(CHARACTER_LCD_INIT_STEP_DELAY_US);
}

/**
 * @brief Character LCD GPIO init.
 *
 * Configure RS, E, and D4..D7 as GPIO outputs.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     GPIO outputs ready.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     GPIO config failure.
 *     Fatal in current design.
 */
static esp_err_t initialize_character_lcd_gpio(void)
{
    gpio_config_t character_lcd_gpio_configuration = {
        .pin_bit_mask =
            (1ULL << CHARACTER_LCD_RS_GPIO_NUM) |
            (1ULL << CHARACTER_LCD_E_GPIO_NUM) |
            (1ULL << CHARACTER_LCD_D4_GPIO_NUM) |
            (1ULL << CHARACTER_LCD_D5_GPIO_NUM) |
            (1ULL << CHARACTER_LCD_D6_GPIO_NUM) |
            (1ULL << CHARACTER_LCD_D7_GPIO_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        gpio_config(&character_lcd_gpio_configuration),
        character_lcd_log_tag,
        "character LCD GPIO config failed");

    /* Default low. */
    set_character_lcd_gpio_level(CHARACTER_LCD_RS_GPIO_NUM, 0U);
    set_character_lcd_gpio_level(CHARACTER_LCD_E_GPIO_NUM, 0U);

    for (int data_pin_index = 0; data_pin_index < 4; data_pin_index++) {
        set_character_lcd_gpio_level(character_lcd_data_gpio_numbers[data_pin_index], 0U);
    }

    return ESP_OK;
}

/**
 * @brief Character LCD full init.
 *
 * Configure GPIOs, force 4-bit mode, set 2-line 5x8 mode,
 * clear display, set entry mode, turn display on.
 *
 * @param None.
 *
 * @return ESP_OK on success.
 *     LCD ready for writes.
 *     Normal init path.
 * @return Other esp_err_t on failure.
 *     GPIO init failure.
 *     Caller should abort or retry.
 */
esp_err_t initialize_character_lcd(void)
{
    ESP_RETURN_ON_ERROR(
        initialize_character_lcd_gpio(),
        character_lcd_log_tag,
        "character LCD GPIO init failed");

    initialize_character_lcd_4_bit_mode_sequence();

    write_character_lcd_command_internal(
        CHARACTER_LCD_COMMAND_FUNCTION_SET |
        CHARACTER_LCD_FUNCTION_SET_4_BIT |
        CHARACTER_LCD_FUNCTION_SET_2_LINE |
        CHARACTER_LCD_FUNCTION_SET_5X8_FONT);

    write_character_lcd_command_internal(CHARACTER_LCD_COMMAND_DISPLAY_CONTROL);
    write_character_lcd_command_internal(CHARACTER_LCD_COMMAND_CLEAR_DISPLAY);
    write_character_lcd_command_internal(
        CHARACTER_LCD_COMMAND_ENTRY_MODE_SET |
        CHARACTER_LCD_ENTRY_INCREMENT |
        CHARACTER_LCD_ENTRY_SHIFT_OFF);
    write_character_lcd_command_internal(
        CHARACTER_LCD_COMMAND_DISPLAY_CONTROL |
        CHARACTER_LCD_DISPLAY_ON |
        CHARACTER_LCD_CURSOR_OFF |
        CHARACTER_LCD_BLINK_OFF);

    character_lcd_is_initialized = true;

    return ESP_OK;
}

/**
 * @brief Character LCD deinit.
 *
 * Drive LCD pins low and clear local init flag.
 *
 * @param None.
 *
 * @return None.
 */
void deinitialize_character_lcd(void)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    set_character_lcd_gpio_level(CHARACTER_LCD_RS_GPIO_NUM, 0U);
    set_character_lcd_gpio_level(CHARACTER_LCD_E_GPIO_NUM, 0U);

    for (int data_pin_index = 0; data_pin_index < 4; data_pin_index++) {
        set_character_lcd_gpio_level(character_lcd_data_gpio_numbers[data_pin_index], 0U);
    }

    character_lcd_is_initialized = false;
}

/**
 * @brief LCD clear helper.
 *
 * Clear display and return cursor home.
 *
 * @param None.
 *
 * @return None.
 */
void clear_character_lcd(void)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    write_character_lcd_command_internal(CHARACTER_LCD_COMMAND_CLEAR_DISPLAY);
}

/**
 * @brief LCD home helper.
 *
 * Return cursor and display shift to home position.
 *
 * @param None.
 *
 * @return None.
 */
void home_character_lcd(void)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    write_character_lcd_command_internal(CHARACTER_LCD_COMMAND_RETURN_HOME);
}

/**
 * @brief Cursor-position helper.
 *
 * Set cursor to one row / column position.
 *
 * @param row_index
 *     Target row index.
 *     Expected range 0..CHARACTER_LCD_NUM_ROWS-1.
 *     Used for DDRAM base select.
 * @param column_index
 *     Target column index.
 *     Expected range 0..CHARACTER_LCD_NUM_COLUMNS-1.
 *     Used for DDRAM offset select.
 *
 * @return None.
 */
void set_character_lcd_cursor_position(int row_index, int column_index)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    if (row_index < 0) {
        row_index = 0;
    }
    if (row_index >= CHARACTER_LCD_NUM_ROWS) {
        row_index = CHARACTER_LCD_NUM_ROWS - 1;
    }

    if (column_index < 0) {
        column_index = 0;
    }
    if (column_index >= CHARACTER_LCD_NUM_COLUMNS) {
        column_index = CHARACTER_LCD_NUM_COLUMNS - 1;
    }

    write_character_lcd_command_internal(
        CHARACTER_LCD_COMMAND_SET_DDRAM_ADDRESS |
        (character_lcd_row_start_addresses[row_index] + (uint8_t)column_index));
}

/**
 * @brief One character write helper.
 *
 * Write one character at current cursor position.
 *
 * @param character_value
 *     Character to display.
 *     Sent as one LCD data byte.
 *     Used for text output.
 *
 * @return None.
 */
void write_character_lcd_character(char character_value)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    write_character_lcd_data_byte((uint8_t)character_value);
}

/**
 * @brief String write helper.
 *
 * Write null-terminated string from current cursor position.
 *
 * @param string_to_write
 *     Null-terminated text pointer.
 *     Characters written until null terminator.
 *     Used for LCD text output.
 *
 * @return None.
 */
void write_character_lcd_string(const char *string_to_write)
{
    if ((!character_lcd_is_initialized) || (string_to_write == NULL)) {
        return;
    }

    while (*string_to_write != '\0') {
        write_character_lcd_character(*string_to_write);
        string_to_write++;
    }
}

/**
 * @brief Fixed-width line write helper.
 *
 * Write one LCD row and pad remainder with spaces.
 *
 * @param row_index
 *     Target row index.
 *     Expected range 0..CHARACTER_LCD_NUM_ROWS-1.
 *     Used for DDRAM row select.
 * @param line_text
 *     Null-terminated text pointer.
 *     Written up to display width.
 *     Remaining cells padded with spaces.
 *
 * @return None.
 */
void write_character_lcd_line(int row_index, const char *line_text)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    set_character_lcd_cursor_position(row_index, 0);

    for (int column_index = 0; column_index < CHARACTER_LCD_NUM_COLUMNS; column_index++) {
        char character_to_write = ' ';

        if ((line_text != NULL) && (line_text[column_index] != '\0')) {
            character_to_write = line_text[column_index];
        }

        write_character_lcd_character(character_to_write);
    }
}

/**
 * @brief Two-line status helper.
 *
 * Rewrite both LCD rows from two text buffers.
 *
 * @param first_line_text
 *     Null-terminated first-row text.
 *     Written to row 0, padded with spaces.
 *     Used for top-line status.
 * @param second_line_text
 *     Null-terminated second-row text.
 *     Written to row 1, padded with spaces.
 *     Used for bottom-line status.
 *
 * @return None.
 */
void write_character_lcd_two_line_status(
    const char *first_line_text,
    const char *second_line_text)
{
    if (!character_lcd_is_initialized) {
        return;
    }

    write_character_lcd_line(0, first_line_text);
    write_character_lcd_line(1, second_line_text);
}
