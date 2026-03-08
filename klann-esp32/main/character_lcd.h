/**
 * @file character_lcd.h
 * @brief Public API for the HD44780-compatible 16x2 LCD driver.
 *
 * Exposes initialization, cursor, and line-write helpers used by the
 * ESP32 status task.
 */

#ifndef CHARACTER_LCD_H
#define CHARACTER_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* ---------- Public constants ---------- */

#define CHARACTER_LCD_NUM_ROWS    2
#define CHARACTER_LCD_NUM_COLUMNS 16

/* ---------- Public init / deinit ---------- */

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
esp_err_t initialize_character_lcd(void);

/**
 * @brief Character LCD deinit.
 *
 * Drive LCD pins low and clear local init flag.
 *
 * @param None.
 *
 * @return None.
 */
void deinitialize_character_lcd(void);

/* ---------- Public write helpers ---------- */

/**
 * @brief LCD clear helper.
 *
 * Clear display and return cursor home.
 *
 * @param None.
 *
 * @return None.
 */
void clear_character_lcd(void);

/**
 * @brief LCD home helper.
 *
 * Return cursor and display shift to home position.
 *
 * @param None.
 *
 * @return None.
 */
void home_character_lcd(void);

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
void set_character_lcd_cursor_position(int row_index, int column_index);

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
void write_character_lcd_character(char character_value);

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
void write_character_lcd_string(const char *string_to_write);

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
void write_character_lcd_line(int row_index, const char *line_text);

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
    const char *second_line_text);

#ifdef __cplusplus
}
#endif

#endif
