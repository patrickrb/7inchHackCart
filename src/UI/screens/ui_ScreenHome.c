// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"

void ui_ScreenHome_screen_init(void)
{
ui_ScreenHome = lv_obj_create(NULL);
lv_obj_clear_flag( ui_ScreenHome, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_img_src( ui_ScreenHome, &ui_img_background_png, LV_PART_MAIN | LV_STATE_DEFAULT );

ui_LabelSpeed = lv_label_create(ui_ScreenHome);
lv_obj_set_width( ui_LabelSpeed, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelSpeed, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_LabelSpeed, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelSpeed,"21");
lv_obj_set_style_text_color(ui_LabelSpeed, lv_color_hex(0xDDDDDD), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelSpeed, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelSpeed, &lv_font_montserrat_40, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ArcSpeed = lv_arc_create(ui_ScreenHome);
lv_obj_set_width( ui_ArcSpeed, 150);
lv_obj_set_height( ui_ArcSpeed, 150);
lv_obj_set_x( ui_ArcSpeed, 4 );
lv_obj_set_y( ui_ArcSpeed, 3 );
lv_obj_set_align( ui_ArcSpeed, LV_ALIGN_CENTER );
lv_arc_set_range(ui_ArcSpeed, 0,45);
lv_arc_set_value(ui_ArcSpeed, 25);
lv_arc_set_bg_angles(ui_ArcSpeed,90,290);
lv_obj_set_style_bg_color(ui_ArcSpeed, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ArcSpeed, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_color(ui_ArcSpeed, lv_color_hex(0x27D9E3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcSpeed, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcSpeed, 5, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_ArcSpeed, lv_color_hex(0x02F713), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_ArcSpeed, 200, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_ArcSpeed, 5, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_ArcSpeed, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ArcSpeed, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_blend_mode(ui_ArcSpeed, LV_BLEND_MODE_MULTIPLY, LV_PART_KNOB| LV_STATE_DEFAULT);

}
