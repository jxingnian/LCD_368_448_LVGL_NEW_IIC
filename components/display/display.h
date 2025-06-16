#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

// LCD主机使用的SPI总线
#define LCD_HOST SPI2_HOST
// 触摸芯片使用的I2C总线
#define TOUCH_HOST I2C_NUM_0

// 根据LVGL颜色深度配置LCD每像素位数
#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif

// LCD分辨率
#define EXAMPLE_LCD_H_RES 368 // 水平像素数
#define EXAMPLE_LCD_V_RES 448 // 垂直像素数

// LVGL缓冲区高度（建议为屏幕高度的1/4）
#define EXAMPLE_LVGL_BUF_HEIGHT (EXAMPLE_LCD_V_RES / 4)
// LVGL时钟周期（ms）
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
// LVGL任务最大/最小延时（ms）
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
// LVGL任务堆栈大小和优先级
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

// 显示屏初始化函数
esp_err_t display_init(void);

// LVGL互斥锁相关函数
bool display_lvgl_lock(int timeout_ms);
void display_lvgl_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */ 