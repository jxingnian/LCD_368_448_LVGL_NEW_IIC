/*
 * @Author: jixingnian@gmail.com
 * @Date: 2025-06-12 15:42:08
 * @LastEditTime: 2025-06-13 13:09:49
 * @LastEditors: 星年
 * @Description: 
 * @FilePath: \05_LVGL_WITH_RAM\components\ui\ui_events.c
 * 遇事不决，可问春风
 */
#include "ui.h"
#include "esp_log.h"

static const char *TAG = "ui_events";

#define CHAT_HEIGHT 50
#define CHAT_WIDTH 300
#define CHAT_X 0
#define CHAT_Y -140
#define CHAT_MARGIN 20

/* 电池电量低 */
#define BATTERY_LOW "1"
/* 电池电量中 */
#define BATTERY_MID "2"
/* 电池电量高 */
#define BATTERY_HIGH "3"
/* 网络WIFI */
#define NETWORK_WIFI "4"
/* 网络有线 */
#define NETWORK_CABLE "5"
/* 网络4G */
#define NETWORK_4G "6"
/* 音频播放 */
#define STATUS_PLAY "7"
/* 音频待机 */
#define STATUS_WAIT "8" 
/* 音频MIC */
#define STATUS_MIC "9"

lv_obj_t * ui_chat;

int chat_count = 0;
void new_chat(lv_obj_t *ui_chat, char *text){
    ui_chat = lv_textarea_create(ui_Panel1);
    lv_obj_set_width(ui_chat, CHAT_WIDTH);
    lv_obj_set_height(ui_chat, CHAT_HEIGHT);
    lv_obj_set_x(ui_chat, CHAT_X);
    lv_obj_set_y(ui_chat, CHAT_Y + chat_count++ * (CHAT_HEIGHT+CHAT_MARGIN));
    lv_obj_set_align(ui_chat, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_chat, LV_OBJ_FLAG_CLICK_FOCUSABLE);  // 禁止获取焦点
    lv_textarea_set_text(ui_chat, text);
    lv_obj_set_style_text_font(ui_chat, &ui_font_dinglie32, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_textarea_set_one_line(ui_chat, true);
}

void ui_events_init(void){

lv_label_set_text(ui_Time, "13:14");

lv_label_set_text(ui_status, STATUS_WAIT);
lv_label_set_text(ui_network, NETWORK_WIFI);
lv_label_set_text(ui_battery, BATTERY_LOW);

new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
new_chat(ui_chat, "你好,我是小星1");
new_chat(ui_chat, "你好,我是小星2");
new_chat(ui_chat, "你好,我是小星3");
}
