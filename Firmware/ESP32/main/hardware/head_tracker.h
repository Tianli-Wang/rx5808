#ifndef __HEAD_TRACKER_H
#define __HEAD_TRACKER_H

#include <stdint.h>
#include "esp_err.h"

/* 头追硬件与 PPM 参数集中放在这里，便于按实际接线修改。 */
#define HEAD_TRACKER_MPU6050_ADDR          0x68

/* PICO-D4 的 GPIO16/17 被封装内 Flash 占用，GPIO21 改作 PPM 输出。 */
#define HEAD_TRACKER_PPM_GPIO              21
#define HEAD_TRACKER_PPM_CHANNELS          8
#define HEAD_TRACKER_YAW_CHANNEL           0
#define HEAD_TRACKER_PITCH_CHANNEL         1

#define HEAD_TRACKER_PPM_MIN_US            1000
#define HEAD_TRACKER_PPM_CENTER_US         1500
#define HEAD_TRACKER_PPM_MAX_US            2000
#define HEAD_TRACKER_PPM_FRAME_US          22500
#define HEAD_TRACKER_PPM_PULSE_US          300

/* 转到该角度时达到对应通道的 1000/2000 us。 */
#define HEAD_TRACKER_YAW_RANGE_DEG          90.0f
#define HEAD_TRACKER_PITCH_RANGE_DEG        45.0f

/**
 * @brief 启动 MPU6050 采样、姿态解算和 PPM 输出任务。
 *
 * I2C 总线必须已经由 eeprom_24cxx_init() 初始化。
 */
esp_err_t head_tracker_init(void);

/**
 * @brief 请求把当前头部姿态设为新的云台中心。
 *
 * 该函数可由按键任务安全调用，实际回中在 MPU6050 任务中完成。
 */
void head_tracker_request_center(void);

/**
 * @brief 读取当前实际用于 PPM 输出的通道脉宽。
 *
 * @param channel 从 0 开始的通道索引。
 * @return 通道脉宽，单位 us；索引无效时返回中位值。
 */
uint16_t head_tracker_get_ppm_channel(uint8_t channel);

#endif
