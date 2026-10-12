#ifndef MY_I2C_H
#define MY_I2C_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "hardware/hwvers.h"

void i2c_master_init(void);

/* 在共用 I2C 总线上注册一个 7 位地址设备。 */
esp_err_t my_i2c_add_device(uint16_t address, i2c_master_dev_handle_t *device);

#endif
