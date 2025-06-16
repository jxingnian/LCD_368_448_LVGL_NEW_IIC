/*******************************************************************************
 * Size: 40 px
 * Bpp: 1
 * Opts: --bpp 1 --size 40 --font C:/XingNian/ESP32/ChunFeng/lvgl_8.3/assets/head_portrait.ttf -o C:/XingNian/ESP32/ChunFeng/lvgl_8.3/assets\ui_font_headPortrait.c --format lvgl -r 0x61 -r 0x62 --symbols ab --no-compress --no-prefilter
 ******************************************************************************/

#include "../ui.h"

#ifndef UI_FONT_HEADPORTRAIT
#define UI_FONT_HEADPORTRAIT 1
#endif

#if UI_FONT_HEADPORTRAIT

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0061 "a" */
    0x0, 0x3c, 0x0, 0x0, 0xff, 0x0, 0x1, 0xc3,
    0x80, 0x1, 0x81, 0x80, 0x3, 0x0, 0xc0, 0x3,
    0x0, 0xc0, 0x3, 0x0, 0xc0, 0x3, 0x0, 0xc0,
    0x1, 0x81, 0x80, 0x1, 0xc3, 0x80, 0x0, 0xff,
    0x0, 0x0, 0x3c, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x7, 0xff, 0xe0, 0x1f, 0xff, 0xf8,
    0x38, 0x0, 0x1c, 0x60, 0x0, 0x6, 0xe0, 0x0,
    0x7, 0xc0, 0x0, 0x3, 0xc0, 0x0, 0x3, 0xc0,
    0x0, 0x3, 0xe0, 0x0, 0x7, 0x60, 0x0, 0x6,
    0x38, 0x0, 0x1c, 0x1f, 0xff, 0xf8, 0x7, 0xff,
    0xe0,

    /* U+0062 "b" */
    0x0, 0xf, 0xe0, 0x0, 0x0, 0x7f, 0xf0, 0x0,
    0x1, 0xff, 0xf0, 0x0, 0x7, 0xff, 0xf0, 0x0,
    0x1f, 0xff, 0xf0, 0x0, 0x3f, 0xff, 0xe0, 0x0,
    0xff, 0xff, 0xe0, 0x1, 0xff, 0xff, 0xc0, 0x3,
    0xff, 0xff, 0x80, 0x7, 0xff, 0xff, 0x0, 0xf,
    0xff, 0xfe, 0x0, 0x1f, 0xff, 0xfc, 0x0, 0x1f,
    0xff, 0xf0, 0x0, 0x3f, 0xff, 0xe0, 0x0, 0x3f,
    0xff, 0x80, 0x0, 0x3f, 0xfe, 0x0, 0x0, 0x3f,
    0xf8, 0x0, 0x0, 0x1f, 0xc0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0xff,
    0x0, 0x3, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff,
    0xf0, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff,
    0xf, 0xff, 0xff, 0xf8, 0x7, 0xff, 0xff, 0xc0,
    0x0, 0xff, 0xf8, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 640, .box_w = 24, .box_h = 27, .ofs_x = 8, .ofs_y = 2},
    {.bitmap_index = 81, .adv_w = 640, .box_w = 31, .box_h = 34, .ofs_x = 5, .ofs_y = -1}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 97, .range_length = 2, .glyph_id_start = 1,
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
const lv_font_t ui_font_headPortrait = {
#else
lv_font_t ui_font_headPortrait = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 34,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
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



#endif /*#if UI_FONT_HEADPORTRAIT*/

