1、cmake中
add_library(ui ${SOURCES})
替换为
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES lvgl__lvgl
)

2、cmake中添加   
    ui_events.c

3、ui_event.h中添加
    void ui_events_init(void);

4、ui.h中
#include "lvgl/lvgl.h"
替换为
#include "lvgl.h"

