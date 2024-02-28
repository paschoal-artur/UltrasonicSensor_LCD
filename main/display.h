#pragma once

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "display.h"

// flags for backlight control
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

typedef struct {
    uint8_t addr;
	uint8_t displayfunction;
	uint8_t displaycontrol;
	uint8_t displaymode;
	uint8_t cols;
	uint8_t rows;
	uint8_t charsize;
	uint8_t backlightval;
	uint8_t sda;
	uint8_t scl;
} lcd_t;

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02

#define LCD_FUNCTIONSET 0x20

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// flags for RS pin modes
#define B00000100 0x04
#define B00000010 0x02
#define B00000001 0x01

#define En B00000100 // Enable bit
#define Rw B00000010 // Read/Write bit
#define Rs B00000001 // Register select bit

void i2c_init(lcd_t *lcd)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = lcd->sda,
        .scl_io_num = lcd->scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

i2c_cmd_handle_t Wire_beginTransmission(uint8_t addr)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd_handle));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, addr << 1 | I2C_MASTER_WRITE, 1));
    return cmd_handle;
}

void Wire_endTransmission(i2c_cmd_handle_t cmd_handle)
{
    ESP_ERROR_CHECK(i2c_master_stop(cmd_handle));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd_handle);
}

void Wire_write(i2c_cmd_handle_t cmd_handle, uint8_t data)
{
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, data, 1));
}

void lcd_expanderWrite(lcd_t *lcd, uint8_t data)
{
    i2c_cmd_handle_t cmd_handle = Wire_beginTransmission(lcd->addr);
    Wire_write(cmd_handle, (int)(data) | lcd->backlightval);
    Wire_endTransmission(cmd_handle);
}

void lcd_pulseEnable(lcd_t *lcd, uint8_t data)
{
    lcd_expanderWrite(lcd, data | En);  // En high
    vTaskDelay(1 / portTICK_PERIOD_MS); // enable pulse must be >450ns

    lcd_expanderWrite(lcd, data & ~En);  // En low
    vTaskDelay(50 / portTICK_PERIOD_MS); // commands need > 37us to settle
}

void lcd_write4bits(lcd_t *lcd, uint8_t value)
{
    lcd_expanderWrite(lcd, value);
    lcd_pulseEnable(lcd, value);
}

void lcd_send(lcd_t *lcd, uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xf0;
    uint8_t lownib = (value << 4) & 0xf0;
    lcd_write4bits(lcd, (highnib) | mode);
    lcd_write4bits(lcd, (lownib) | mode);
}

inline void lcd_command(lcd_t *lcd, uint8_t value)
{
    lcd_send(lcd, value, 0);
}

inline size_t lcd_write(lcd_t *lcd, uint8_t value)
{
    lcd_send(lcd, value, Rs);
    return 1;
}

void lcd_display(lcd_t *lcd)
{
    lcd->displaycontrol |= LCD_DISPLAYON;
    lcd_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}

void lcd_clear(lcd_t *lcd)
{
    lcd_command(lcd, LCD_CLEARDISPLAY);   // clear display, set cursor position to zero
    vTaskDelay(200 / portTICK_PERIOD_MS); // this command takes a long time!
}

void lcd_home(lcd_t *lcd)
{
    lcd_command(lcd, LCD_RETURNHOME);     // set cursor position to zero
    vTaskDelay(200 / portTICK_PERIOD_MS); // this command takes a long time!
}

void lcd_print(lcd_t *lcd, char *str)
{
    while (*str)
    {
        lcd_write(lcd, *str++);
    }
}

void lcd_begin(lcd_t *lcd)
{
    // Wire.begin();
    lcd->displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

    if (lcd->rows > 1)
    {
        lcd->displayfunction |= LCD_2LINE;
    }

    // for some 1 line displays you can select a 10 pixel high font
    if ((lcd->charsize != 0) && (lcd->rows == 1))
    {
        lcd->displayfunction |= LCD_5x10DOTS;
    }

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Now we pull both RS and R/W low to begin commands
    lcd_expanderWrite(lcd, lcd->backlightval); // reset expanderand turn backlight off (Bit 8 =1)
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(lcd, 0x03 << 4);
    vTaskDelay(45 / portTICK_PERIOD_MS); // wait min 4.1ms

    // second try
    lcd_write4bits(lcd, 0x03 << 4);
    vTaskDelay(45 / portTICK_PERIOD_MS); // wait min 4.1ms

    // third go!
    lcd_write4bits(lcd, 0x03 << 4);
    vTaskDelay(15 / portTICK_PERIOD_MS); // wait min 150us

    // finally, set to 4-bit interface
    lcd_write4bits(lcd, 0x02 << 4);

    // set # lines, font size, etc.
    lcd_command(lcd, LCD_FUNCTIONSET | lcd->displayfunction);

    // turn the display on with no cursor or blinking default
    lcd->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_display(lcd);

    // clear it off
    lcd_clear(lcd);

    // Initialize to default text direction (for roman languages)
    lcd->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

    // set the entry mode
    lcd_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);

    lcd_home(lcd);
}

void lcd_backlight(lcd_t *lcd)
{
    lcd->backlightval = LCD_BACKLIGHT;
    lcd_expanderWrite(lcd, 0);
}

void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col)
{
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > lcd->rows)
    {
        row = lcd->rows - 1; // we count rows starting w/0
    }
    lcd_command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}