/*
 * @Author: jixingnian@gmail.com
 * @Date: 2025-06-12 13:31:18
 * @LastEditTime: 2025-06-13 16:44:00
 * @LastEditors: 星年
 * @Description:  
 * @FilePath: \05_LVGL_WITH_RAM\main\example_qspi_with_ram.c
 * 遇事不决，可问春风
 */
#include <stdio.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/i2c.h"
// #include "driver/spi_master.h"
// #include "esp_timer.h"
// #include "esp_lcd_panel_io.h"
// #include "esp_lcd_panel_vendor.h"
// #include "esp_lcd_panel_ops.h"
// #include "esp_err.h"
#include "esp_log.h"

// #include "lvgl.h"
// #include "lv_demos.h"
// #include "esp_lcd_sh8601.h"
// #include "esp_lcd_touch_ft5x06.h"

// #include "esp_io_expander_tca9554.h"
#include "ui.h"
#include "display.h"

// 日志标签
static const char *TAG = "example";

void app_main(void)
{
    // 初始化显示屏
    ESP_ERROR_CHECK(display_init());

    // 显示LVGL界面
    ESP_LOGI(TAG, "Display LVGL demos");
    if (display_lvgl_lock(-1))
    {
        ui_init();
        display_lvgl_unlock();
    }

    if (display_lvgl_lock(-1))
    {
        ui_events_init();
        display_lvgl_unlock();
    }
}