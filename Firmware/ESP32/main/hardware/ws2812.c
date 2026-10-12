#include "ws2812.h"

#include <stddef.h>

#include "esp_log.h"

static const char *TAG = "ws2812";

led_strip_t *led_strip_new_rmt_ws2812(const led_strip_config_t *config)
{
    (void)config;
    return NULL;
}

led_strip_t *led_strip_init(uint8_t channel, uint8_t gpio, uint16_t led_num)
{
    (void)channel;
    (void)gpio;
    (void)led_num;
    ESP_LOGW(TAG, "WS2812 disabled: GPIO19 is used by the composite-video switch");
    return NULL;
}

esp_err_t led_strip_denit(led_strip_t *strip)
{
    (void)strip;
    return ESP_ERR_NOT_SUPPORTED;
}

void ws2812_init(void)
{
    /* 原工程没有启用灯带，且 GPIO19 已被复合视频切换功能占用。 */
    ESP_LOGW(TAG, "WS2812 disabled: GPIO19 is used by the composite-video switch");
}
