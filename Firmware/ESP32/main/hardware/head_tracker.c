/*
 * 头部追踪：MPU6050（I2C）姿态解算 + 8 通道 PPM 输出。
 *
 * 运行流程
 *   1. head_tracker_init() 先启动 ppm_output_task，此时各通道输出中位 1500 us；
 *      随后创建 head_tracker_task 负责姿态解算。
 *   2. head_tracker_task 启动阶段：配置 MPU6050（100 Hz 采样、±2 g、±250 dps、
 *      约 44 Hz 低通）→ 丢弃 PLL 与陀螺稳定期的 50 个采样 → 用随后 500 个静止
 *      采样求初始零偏并打印（dps）→ 以校准时的姿态为中心进入主循环。
 *   3. 主循环每 10 ms 执行一次：
 *      - 读 14 字节原始数据，用 esp_timer 计算 dt（异常时回退到 10 ms）；
 *      - 加速度计解算 pitch_acc；MPU6050 无磁力计，yaw 只能由 Z 轴陀螺积分；
 *      - 两轴角速度经 gyro_axis_filter()：死区 → 静止检测 → 在线零偏估计；
 *      - pitch 用互补滤波融合加速度计（长期稳定），yaw 直接积分；
 *      - 相对中心的角度映射为 1000~2000 us 脉宽写入 s_ppm_channels。
 *   4. 收到 head_tracker_request_center() 请求时，把当前姿态设为新的中心。
 *
 * 抗漂移设计（按生效顺序）
 *   - 启动零偏校准：开机保持静止约 5.5 s，消除静态零偏。
 *   - 小角速度死区（0.3 dps）：吸收残余零偏与噪声，静止时积分严格停住，
 *     yaw 不会再缓慢爬向 1000/2000 us。
 *   - 静止检测（2.0 dps 连续 5 个采样）：确认头部确实没动，避免把真实转动
 *     当成零偏。
 *   - 在线零偏估计：确认静止后按 τ≈1 s 用残差修正零偏，跟随温漂；静止阈值
 *     刻意大于死区，保证超出死区的漂移也能被拉回死区以内。
 *
 * PPM 输出
 *   ppm_output_task 按 HEAD_TRACKER_PPM_FRAME_US（22.5 ms）组帧，通道 0/1 为
 *   yaw/pitch，其余通道保持中位。脉宽存于 s_ppm_channels，UI 可通过
 *   head_tracker_get_ppm_channel() 读取。
 *
 * 依赖：I2C 总线必须先由 eeprom_24cxx_init() 初始化。
 */

#include "head_tracker.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "MyI2C.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#else
#include "driver/rmt.h"
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hwvers.h"

#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_SAMPLE_PERIOD_MS    10
#define MPU6050_CALIB_SAMPLES       500
#define MPU6050_CALIB_SKIP_SAMPLES  50
#define MPU6050_ACCEL_SCALE         16384.0f
#define MPU6050_GYRO_SCALE          131.0f
#define MPU6050_COMPLEMENTARY_ALPHA 0.98f

/* 死区吸收残余零偏；静止检测则让在线零偏估计跟随温漂。 */
#define MPU6050_DEADBAND_DPS        0.3f
#define MPU6050_STILL_RATE_DPS      2.0f
#define MPU6050_STILL_SAMPLES       5
#define MPU6050_BIAS_TRACK_ALPHA    0.01f

#if ESP_IDF_VERSION_MAJOR < 5
#define PPM_RMT_CHANNEL             RMT_CHANNEL_1
#define PPM_RMT_CLK_DIV             80
#endif
#define PPM_ITEM_COUNT              (HEAD_TRACKER_PPM_CHANNELS + 1)

static const char *TAG = "head_tracker";
static i2c_master_dev_handle_t s_mpu6050;

static volatile bool s_center_requested;
static volatile uint16_t s_ppm_channels[HEAD_TRACKER_PPM_CHANNELS] = {
    HEAD_TRACKER_PPM_CENTER_US, HEAD_TRACKER_PPM_CENTER_US,
    HEAD_TRACKER_PPM_CENTER_US, HEAD_TRACKER_PPM_CENTER_US,
    HEAD_TRACKER_PPM_CENTER_US, HEAD_TRACKER_PPM_CENTER_US,
    HEAD_TRACKER_PPM_CENTER_US, HEAD_TRACKER_PPM_CENTER_US,
};

#if ESP_IDF_VERSION_MAJOR >= 5
static rmt_channel_handle_t s_ppm_channel;
static rmt_encoder_handle_t s_ppm_encoder;
typedef rmt_symbol_word_t ppm_item_t;
#else
typedef rmt_item32_t ppm_item_t;
#endif

static int16_t bytes_to_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(s_mpu6050, data, sizeof(data), 50);
}

static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu6050, &reg, 1, data, len, 50);
}

static esp_err_t mpu6050_configure(void)
{
    if (s_mpu6050 == NULL) {
        esp_err_t add_ret = my_i2c_add_device(HEAD_TRACKER_MPU6050_ADDR, &s_mpu6050);
        if (add_ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to register MPU6050: %s", esp_err_to_name(add_ret));
            return add_ret;
        }
    }

    uint8_t who_am_i = 0;
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 I2C communication failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (who_am_i != 0x68 && who_am_i != 0x69) {
        ESP_LOGE(TAG, "unexpected MPU6050 WHO_AM_I: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    /* 选用 PLL、100 Hz 采样、约 44 Hz 低通、±2 g 和 ±250 dps。 */
    if ((ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01)) != ESP_OK ||
        (ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 9)) != ESP_OK ||
        (ret = mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03)) != ESP_OK ||
        (ret = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00)) != ESP_OK ||
        (ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00)) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static uint16_t angle_to_ppm(float angle_deg, float range_deg)
{
    float pulse = HEAD_TRACKER_PPM_CENTER_US +
                  angle_deg * (500.0f / range_deg);
    if (pulse < HEAD_TRACKER_PPM_MIN_US) {
        pulse = HEAD_TRACKER_PPM_MIN_US;
    } else if (pulse > HEAD_TRACKER_PPM_MAX_US) {
        pulse = HEAD_TRACKER_PPM_MAX_US;
    }
    return (uint16_t)(pulse + 0.5f);
}

static void ppm_fill_item(ppm_item_t *item, uint16_t width_us)
{
    item->level0 = 1;
    item->duration0 = HEAD_TRACKER_PPM_PULSE_US;
    item->level1 = 0;
    item->duration1 = width_us - HEAD_TRACKER_PPM_PULSE_US;
}

static void ppm_output_task(void *param)
{
    ppm_item_t items[PPM_ITEM_COUNT];

    while (1) {
        uint32_t used_us = 0;
        for (int i = 0; i < HEAD_TRACKER_PPM_CHANNELS; ++i) {
            /* 16 位通道值在 ESP32 上可原子读取，避免阻塞姿态任务。 */
            uint16_t width_us = s_ppm_channels[i];
            ppm_fill_item(&items[i], width_us);
            used_us += width_us;
        }

        uint16_t sync_us = HEAD_TRACKER_PPM_FRAME_US - used_us;
        if (sync_us < (HEAD_TRACKER_PPM_PULSE_US + 1000)) {
            sync_us = HEAD_TRACKER_PPM_PULSE_US + 1000;
        }
        ppm_fill_item(&items[HEAD_TRACKER_PPM_CHANNELS], sync_us);

#if ESP_IDF_VERSION_MAJOR >= 5
        const rmt_transmit_config_t tx_config = {
            .loop_count = 0,
        };
        esp_err_t ret = rmt_transmit(s_ppm_channel, s_ppm_encoder, items,
                                     sizeof(items), &tx_config);
        if (ret == ESP_OK) {
            ret = rmt_tx_wait_all_done(s_ppm_channel, -1);
        }
#else
        esp_err_t ret = rmt_write_items(PPM_RMT_CHANNEL, items, PPM_ITEM_COUNT, true);
#endif
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPM output failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static esp_err_t ppm_output_init(void)
{
#if ESP_IDF_VERSION_MAJOR >= 5
    const rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = HEAD_TRACKER_PPM_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 1000000, /* 每个 RMT tick 为 1 us。 */
        .trans_queue_depth = 2,
    };
    esp_err_t ret = rmt_new_tx_channel(&channel_config, &s_ppm_channel);
    if (ret != ESP_OK) {
        return ret;
    }

    const rmt_copy_encoder_config_t encoder_config = {};
    ret = rmt_new_copy_encoder(&encoder_config, &s_ppm_encoder);
    if (ret != ESP_OK) {
        rmt_del_channel(s_ppm_channel);
        s_ppm_channel = NULL;
        return ret;
    }
    ret = rmt_enable(s_ppm_channel);
    if (ret != ESP_OK) {
        rmt_del_encoder(s_ppm_encoder);
        rmt_del_channel(s_ppm_channel);
        s_ppm_encoder = NULL;
        s_ppm_channel = NULL;
        return ret;
    }
#else
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(HEAD_TRACKER_PPM_GPIO, PPM_RMT_CHANNEL);
    config.clk_div = PPM_RMT_CLK_DIV; /* APB 80 MHz / 80 = 1 us。 */
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    esp_err_t ret = rmt_config(&config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = rmt_driver_install(config.channel, 0, 0);
    if (ret != ESP_OK) {
        return ret;
    }
#endif

    if (xTaskCreate(ppm_output_task, "ppm_output", 2048, NULL, 3, NULL) != pdPASS) {
#if ESP_IDF_VERSION_MAJOR >= 5
        rmt_disable(s_ppm_channel);
        rmt_del_encoder(s_ppm_encoder);
        rmt_del_channel(s_ppm_channel);
        s_ppm_encoder = NULL;
        s_ppm_channel = NULL;
#else
        rmt_driver_uninstall(config.channel);
#endif
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "PPM: GPIO%d, CH1=Yaw, CH2=Pitch", HEAD_TRACKER_PPM_GPIO);
    return ESP_OK;
}

typedef struct {
    float bias_dps;
    int still_samples;
} gyro_axis_t;

/**
 * @brief 小角速度死区 + 静止检测 + 在线零偏估计，返回用于积分的角速度。
 *
 * 死区吸收低于阈值的残余零偏，静止时输出严格为 0；
 * 连续静止若干个采样后开始用残差修正零偏，消除温漂造成的缓慢漂移。
 *
 * @param axis   该轴的零偏与静止计数状态。
 * @param raw_dps 原始角速度，单位 dps。
 * @return 死区后的角速度，单位 dps。
 */
static float gyro_axis_filter(gyro_axis_t *axis, float raw_dps)
{
    float rate = raw_dps - axis->bias_dps;

    if (fabsf(rate) < MPU6050_STILL_RATE_DPS) {
        if (axis->still_samples < MPU6050_STILL_SAMPLES) {
            ++axis->still_samples;
        }
        if (axis->still_samples >= MPU6050_STILL_SAMPLES) {
            axis->bias_dps += MPU6050_BIAS_TRACK_ALPHA * rate;
        }
    } else {
        axis->still_samples = 0;
    }

    if (fabsf(rate) < MPU6050_DEADBAND_DPS) {
        return 0.0f;
    }
    return rate;
}

static void head_tracker_task(void *param)
{
    uint8_t data[14];
    float gyro_y_sum = 0.0f;
    float gyro_z_sum = 0.0f;
    float accel_x_sum = 0.0f;
    float accel_y_sum = 0.0f;
    float accel_z_sum = 0.0f;

    if (mpu6050_configure() != ESP_OK) {
        ESP_LOGW(TAG, "head tracking disabled; PPM remains centered");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "keep MPU6050 still: calibrating gyro (about 5.5 s)");
    int valid_samples = 0;
    for (int i = 0; i < (MPU6050_CALIB_SKIP_SAMPLES + MPU6050_CALIB_SAMPLES); ++i) {
        if (mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data)) == ESP_OK &&
            i >= MPU6050_CALIB_SKIP_SAMPLES) {
            accel_x_sum += bytes_to_i16(&data[0]);
            accel_y_sum += bytes_to_i16(&data[2]);
            accel_z_sum += bytes_to_i16(&data[4]);
            gyro_y_sum += bytes_to_i16(&data[10]);
            gyro_z_sum += bytes_to_i16(&data[12]);
            ++valid_samples;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_PERIOD_MS));
    }
    if (valid_samples < (MPU6050_CALIB_SAMPLES * 9 / 10)) {
        ESP_LOGE(TAG, "gyro calibration failed: only %d valid samples", valid_samples);
        vTaskDelete(NULL);
        return;
    }

    float gyro_bias_y_dps = gyro_y_sum / (valid_samples * MPU6050_GYRO_SCALE);
    float gyro_bias_z_dps = gyro_z_sum / (valid_samples * MPU6050_GYRO_SCALE);
    ESP_LOGI(TAG, "gyro bias: Y=%.3f dps, Z=%.3f dps", gyro_bias_y_dps, gyro_bias_z_dps);

    /* 校准刚结束时头部仍静止，直接允许在线零偏估计从第一个采样生效。 */
    gyro_axis_t pitch_axis = {
        .bias_dps = gyro_bias_y_dps,
        .still_samples = MPU6050_STILL_SAMPLES,
    };
    gyro_axis_t yaw_axis = {
        .bias_dps = gyro_bias_z_dps,
        .still_samples = MPU6050_STILL_SAMPLES,
    };

    float pitch_deg = atan2f(-(accel_x_sum / valid_samples),
                             sqrtf((accel_y_sum / valid_samples) *
                                       (accel_y_sum / valid_samples) +
                                   (accel_z_sum / valid_samples) *
                                       (accel_z_sum / valid_samples))) *
                      (180.0f / (float)M_PI);
    float yaw_deg = 0.0f;
    float yaw_center = yaw_deg;
    float pitch_center = pitch_deg;
    int64_t last_time_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "gyro calibration completed; head tracker centered");
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MPU6050_SAMPLE_PERIOD_MS));
        if (mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data)) != ESP_OK) {
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;
        if (dt <= 0.0f || dt > 0.05f) {
            dt = MPU6050_SAMPLE_PERIOD_MS / 1000.0f;
        }

        float ax = bytes_to_i16(&data[0]) / MPU6050_ACCEL_SCALE;
        float ay = bytes_to_i16(&data[2]) / MPU6050_ACCEL_SCALE;
        float az = bytes_to_i16(&data[4]) / MPU6050_ACCEL_SCALE;
        float gyro_y = gyro_axis_filter(
            &pitch_axis, bytes_to_i16(&data[10]) / MPU6050_GYRO_SCALE);
        float gyro_z = gyro_axis_filter(
            &yaw_axis, bytes_to_i16(&data[12]) / MPU6050_GYRO_SCALE);

        /* 加速度计约束 Pitch；MPU6050 无磁力计，Yaw 由 Z 轴陀螺积分。 */
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) *
                          (180.0f / (float)M_PI);
        pitch_deg = MPU6050_COMPLEMENTARY_ALPHA * (pitch_deg + gyro_y * dt) +
                    (1.0f - MPU6050_COMPLEMENTARY_ALPHA) * pitch_acc;
        yaw_deg += gyro_z * dt;

        if (s_center_requested) {
            s_center_requested = false;
            yaw_center = yaw_deg;
            pitch_center = pitch_deg;
            ESP_LOGI(TAG, "gimbal recentered");
        }

        s_ppm_channels[HEAD_TRACKER_YAW_CHANNEL] =
            angle_to_ppm(yaw_deg - yaw_center, HEAD_TRACKER_YAW_RANGE_DEG);
        s_ppm_channels[HEAD_TRACKER_PITCH_CHANNEL] =
            angle_to_ppm(pitch_deg - pitch_center, HEAD_TRACKER_PITCH_RANGE_DEG);
    }
}

esp_err_t head_tracker_init(void)
{
    esp_err_t ret = ppm_output_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PPM initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (xTaskCreate(head_tracker_task, "head_tracker", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create head tracker task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void head_tracker_request_center(void)
{
    s_center_requested = true;
}

uint16_t head_tracker_get_ppm_channel(uint8_t channel)
{
    if (channel >= HEAD_TRACKER_PPM_CHANNELS) {
        return HEAD_TRACKER_PPM_CENTER_US;
    }
    /* ESP32 对齐的 16 位读操作是原子的，可直接供 UI 定时器读取。 */
    return s_ppm_channels[channel];
}
