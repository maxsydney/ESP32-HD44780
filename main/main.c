#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "HD44780.h"

static char tag[] = "LCD test";

void task_i2cscanner(void *ignore);

void app_main(void)
{
    ESP_LOGE(tag, "Starting up application");
    LCD_init();
}