#include "page_ppm.h"

#include "beep.h"
#include "head_tracker.h"
#include "page_menu.h"

#define PPM_PAGE_REFRESH_MS 100
#define PPM_PAGE_ROW_HEIGHT 15

static lv_obj_t *s_page_contain;
static lv_obj_t *s_channel_labels[HEAD_TRACKER_PPM_CHANNELS];
static lv_group_t *s_ppm_group;
static lv_timer_t *s_refresh_timer;

static void ppm_values_update(lv_timer_t *timer)
{
    (void)timer;

    /* 两列显示八个通道，数值就是当前送入 RMT 的脉宽。 */
    for (uint8_t channel = 0; channel < HEAD_TRACKER_PPM_CHANNELS; ++channel) {
        lv_label_set_text_fmt(s_channel_labels[channel], "%u:%4uus",
                              channel + 1,
                              head_tracker_get_ppm_channel(channel));
    }
}

static void page_ppm_exit(void)
{
    if (s_refresh_timer != NULL) {
        lv_timer_del(s_refresh_timer);
        s_refresh_timer = NULL;
    }
    if (s_ppm_group != NULL) {
        lv_group_del(s_ppm_group);
        s_ppm_group = NULL;
    }
    if (s_page_contain != NULL) {
        lv_obj_del(s_page_contain);
        s_page_contain = NULL;
    }

    page_menu_create(item_ppm);
}

static void ppm_page_event_callback(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_KEY) {
        return;
    }

    beep_turn_on();
    if (lv_indev_get_key(lv_indev_get_act()) == LV_KEY_LEFT) {
        page_ppm_exit();
    }
}

void page_ppm_create(void)
{
    s_page_contain = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_page_contain);
    lv_obj_set_size(s_page_contain, 160, 80);
    lv_obj_set_pos(s_page_contain, 0, 0);
    lv_obj_set_style_bg_color(s_page_contain, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_page_contain, LV_OPA_COVER, LV_STATE_DEFAULT);
    lv_obj_clear_flag(s_page_contain, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_page_contain);
    lv_label_set_text_fmt(title, "PPM GPIO%d", HEAD_TRACKER_PPM_GPIO);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(title, lv_color_make(0x60, 0xff, 0xff), LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    for (uint8_t channel = 0; channel < HEAD_TRACKER_PPM_CHANNELS; ++channel) {
        const uint8_t column = channel / 4;
        const uint8_t row = channel % 4;

        s_channel_labels[channel] = lv_label_create(s_page_contain);
        lv_obj_set_pos(s_channel_labels[channel], column == 0 ? 5 : 84,
                       16 + row * PPM_PAGE_ROW_HEIGHT);
        lv_obj_set_style_text_font(s_channel_labels[channel], &lv_font_montserrat_12,
                                   LV_STATE_DEFAULT);

        /* CH1/CH2 是头追通道，用不同颜色便于观察动作变化。 */
        if (channel == HEAD_TRACKER_YAW_CHANNEL) {
            lv_obj_set_style_text_color(s_channel_labels[channel],
                                        lv_color_make(0x40, 0xff, 0xff), LV_STATE_DEFAULT);
        } else if (channel == HEAD_TRACKER_PITCH_CHANNEL) {
            lv_obj_set_style_text_color(s_channel_labels[channel],
                                        lv_color_make(0x80, 0xff, 0x60), LV_STATE_DEFAULT);
        } else {
            lv_obj_set_style_text_color(s_channel_labels[channel],
                                        lv_color_make(0xc0, 0xc0, 0xc0), LV_STATE_DEFAULT);
        }
    }

    s_ppm_group = lv_group_create();
    lv_indev_set_group(indev_keypad, s_ppm_group);
    lv_group_add_obj(s_ppm_group, title);
    lv_group_set_editing(s_ppm_group, false);
    lv_obj_add_event_cb(title, ppm_page_event_callback, LV_EVENT_KEY, NULL);

    ppm_values_update(NULL);
    s_refresh_timer = lv_timer_create(ppm_values_update, PPM_PAGE_REFRESH_MS, NULL);
}
