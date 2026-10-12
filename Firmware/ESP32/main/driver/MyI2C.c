#include "MyI2C.h"

#include "esp_log.h"

static const char *TAG = "i2c_bus";
static i2c_master_bus_handle_t s_i2c_bus;

void i2c_master_init(void)
{
    if (s_i2c_bus != NULL) {
        return;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = io_i2c_gpio_sda,
        .scl_io_num = io_i2c_gpio_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C master initialized: SDA=%d, SCL=%d", io_i2c_gpio_sda,
             io_i2c_gpio_scl);
}

esp_err_t my_i2c_add_device(uint16_t address, i2c_master_dev_handle_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_bus == NULL) {
        i2c_master_init();
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = i2c_fre,
    };
    return i2c_master_bus_add_device(s_i2c_bus, &device_config, device);
}
