#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include <esp_log.h>

// I2C driver defines
#define SDA_PIN   19
#define SCL_PIN   18
#define LCD_ADDR  0x27
#define LCD_WRITE 0
#define LCD_READ  1

// LCD module defines
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
#define lcd_LineThree   0x14                    // start of line 3
#define lcd_lineFour    0x54                    // start of line 4

#define lcd_backlight 0x08
#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

// LCD instructions
#define lcd_writeFour = (1 << lcd_RS_bit) | (1 << lcd_E_bit) | (1 << lcd_RW_bit)
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

static char tag[] = "LCD Driver";

static void LCD_writeNibble(uint8_t cmd);
static void LCD_writeByte(uint8_t cmd);
static void LCD_pulseEnable(uint8_t nibble);

static esp_err_t I2C_init(void)
{
    i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}

void LCD_init(void)
{
    I2C_init();
    vTaskDelay(100 / portTICK_RATE_MS);                                // initial 40 mSec delay

// IMPORTANT - At this point the LCD module is in the 8-bit mode and it is expecting to receive  
//   8 bits of data, one bit on each of its 8 data lines, each time the 'E' line is pulsed.
//
// Since the LCD module is wired for the 4-bit mode, only the upper four data lines are connected to 
//   the microprocessor and the lower four data lines are typically left open.  Therefore, when 
//   the 'E' line is pulsed, the LCD controller will read whatever data has been set up on the upper 
//   four data lines and the lower four data lines will be high (due to internal pull-up circuitry).
//
// Fortunately the 'FunctionReset' instruction does not care about what is on the lower four bits so  
//   this instruction can be sent on just the four available data lines and it will be interpreted 
//   properly by the LCD controller.  The 'lcd_write_4' subroutine will accomplish this if the 
//   control lines have previously been configured properly.

// Set up the RS, E, and RW lines for the 'lcd_write_4' function.
    // lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    // lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    // lcd_RW_port &= ~(1<<lcd_RW_bit);                // write to LCD module (RW low)

// // Reset the LCD controller
    LCD_writeNibble(lcd_FunctionReset);                 // first part of reset sequence
    vTaskDelay(10 / portTICK_RATE_MS);                                   // 4.1 mS delay (min)

    LCD_writeNibble(lcd_FunctionReset);                 // second part of reset sequence
    ets_delay_us(200);                                  // 100 uS delay (min)

    LCD_writeNibble(lcd_FunctionReset);                 // third part of reset sequence
    // vTaskDelay(80 / portTICK_RATE_MS);                                   // this delay is omitted in the data sheet
    // ets_delay_us(80);

// // // Preliminary Function Set instruction - used only to set the 4-bit mode.
// // The number of lines or the font cannot be set at this time since the controller is still in the
// //  8-bit mode, but the data transfer mode can be changed since this parameter is determined by one 
// //  of the upper four bits of the instruction.
 
    LCD_writeNibble(lcd_FunctionSet4bit);               // set 4-bit mode
    // ets_delay_us(80);                                 //  40 uS delay (min)
// --> from this point on the busy flag is available <--   

// Function Set instruction
    LCD_writeByte(lcd_FunctionSet4bit);  // set mode, lines, and font
    // ets_delay_us(80); 
    vTaskDelay(2 / portTICK_RATE_MS);

// The next three instructions are specified in the data sheet as part of the initialization routine, 
//  so it is a good idea (but probably not necessary) to do them just as specified and then redo them 
//  later if the application requires a different configuration.

// // Display On/Off Control instruction
    LCD_writeByte(0b00001111);       // turn display OFF
    // ets_delay_us(80); 

// // Clear Display instruction
    LCD_writeByte(lcd_Clear);            // clear display RAM
    vTaskDelay(4 / portTICK_RATE_MS);
    
// ; Entry Mode Set instruction
    LCD_writeByte(lcd_EntryMode);        // set desired shift characteristics
    ets_delay_us(80); 

// This is the end of the LCD controller initialization as specified in the data sheet, but the display
//  has been left in the OFF condition.  This is a good time to turn the display back ON.

// Display On/Off Control instruction
    LCD_writeByte(lcd_DisplayOn);        // turn the display ON
    ets_delay_us(160); 
}

static void LCD_writeNibble(uint8_t nibble)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_ADDR << 1) | LCD_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (nibble & 0xF0) | lcd_backlight, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   

    LCD_pulseEnable(nibble);
}

static void LCD_pulseEnable(uint8_t nibble)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_ADDR << 1) | LCD_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (nibble & 0xF0) | En, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);  
    ets_delay_us(1);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_ADDR << 1) | LCD_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ((nibble & 0xF0) & ~En) | lcd_backlight, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);  
    ets_delay_us(50);
}

static void LCD_writeByte(uint8_t data)
{
    LCD_writeNibble(data & 0xF0);
    LCD_writeNibble((data << 4) & 0xF0);
}