/*******************************************************************************
 * Size: 32 px
 * Bpp: 1
 * Opts: --bpp 1 --size 32 --font C:/XingNian/ESP32/ChunFeng/lvgl_8.3/assets/network.ttf -o C:/XingNian/ESP32/ChunFeng/lvgl_8.3/assets\ui_font_network.c --format lvgl -r 0x34 -r 0x35 -r 0x36 --symbols 456 --no-compress --no-prefilter
 ******************************************************************************/

#include "../ui.h"

#ifndef UI_FONT_NETWORK
#define UI_FONT_NETWORK 1
#endif

#if UI_FONT_NETWORK

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0034 "4" */
    0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xff, 0xc0, 0x7,
    0xc0, 0x3f, 0x1, 0xe0, 0x0, 0x78, 0x38, 0x0,
    0x1, 0xe7, 0x0, 0x0, 0x7, 0xc0, 0x0, 0x0,
    0x30, 0x3, 0xfc, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x3e, 0x7, 0xc0, 0x7, 0x80, 0xe, 0x0, 0x60,
    0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x3f, 0xc0, 0x0, 0x3, 0x9c, 0x0,
    0x0, 0x1f, 0x80, 0x0, 0x0, 0xf0, 0x0, 0x0,
    0x6, 0x0, 0x0,

    /* U+0035 "5" */
    0x1, 0xff, 0x80, 0x1, 0xff, 0x80, 0x7, 0x81,
    0xe0, 0x7, 0x81, 0xe0, 0x6, 0x0, 0x60, 0xfe,
    0x0, 0x7f, 0xfe, 0x0, 0x7f, 0xc0, 0x0, 0x3,
    0xc0, 0x0, 0x3, 0xcf, 0xff, 0xf3, 0xcf, 0xff,
    0xf3, 0xcc, 0x0, 0x33, 0xcc, 0x0, 0x33, 0xcc,
    0x0, 0x33, 0xcc, 0x99, 0x33, 0xcd, 0x99, 0xb3,
    0xcd, 0x99, 0xb3, 0xcf, 0xff, 0xf3, 0xcf, 0xff,
    0xf3, 0xc0, 0x0, 0x3, 0xc0, 0x0, 0x3, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff,

    /* U+0036 "6" */
    0x0, 0x60, 0x7, 0xf0, 0x0, 0xe0, 0x1f, 0xf8,
    0x0, 0xe0, 0x38, 0x1c, 0x1, 0xe0, 0x70, 0xe,
    0x3, 0xe0, 0x60, 0x6, 0x3, 0x60, 0x60, 0x0,
    0x6, 0x60, 0xc0, 0x0, 0xe, 0x60, 0xc0, 0x0,
    0xc, 0x60, 0xc0, 0x0, 0x18, 0x60, 0xc1, 0xfe,
    0x38, 0x60, 0xc1, 0xff, 0x30, 0x60, 0xc0, 0x3,
    0x60, 0x60, 0xc0, 0x3, 0xff, 0xff, 0xc0, 0x3,
    0x7f, 0xfe, 0x60, 0x6, 0x0, 0x60, 0x60, 0x6,
    0x0, 0x60, 0x70, 0xe, 0x0, 0x60, 0x38, 0x1c,
    0x0, 0x60, 0x1f, 0xf8, 0x0, 0x60, 0x7, 0xf0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 512, .box_w = 28, .box_h = 19, .ofs_x = 2, .ofs_y = 2},
    {.bitmap_index = 67, .adv_w = 512, .box_w = 24, .box_h = 23, .ofs_x = 4, .ofs_y = 2},
    {.bitmap_index = 136, .adv_w = 512, .box_w = 32, .box_h = 20, .ofs_x = 0, .ofs_y = 2}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 52, .range_length = 3, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t ui_font_network = {
#else
lv_font_t ui_font_network = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 23,          /*The maximum line height required by the font*/
    .base_line = -2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = 0,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if UI_FONT_NETWORK*/

