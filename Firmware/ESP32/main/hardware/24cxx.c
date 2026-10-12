#include "24cxx.h"

#include <stdlib.h>
#include <string.h>

#include "MyI2C.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EEPROM_I2C_TIMEOUT_MS 1000
#define EEPROM_WRITE_CYCLE_MS 10

static const char *TAG = "24cxx";
static i2c_master_dev_handle_t s_eeprom;

void eeprom_24cxx_init(void)
{
    i2c_master_init();
    if (s_eeprom == NULL) {
        /* EEPROM_24CXX_ADDR 是旧代码使用的 8 位地址，需要右移为 7 位地址。 */
        ESP_ERROR_CHECK(my_i2c_add_device(EEPROM_24CXX_ADDR >> 1, &s_eeprom));
    }
}

void eeprom_24cxx_read_byte_len(uint16_t addr, uint8_t *databuf, uint16_t len)
{
    if (databuf == NULL || len == 0) {
        return;
    }
    if (s_eeprom == NULL) {
        eeprom_24cxx_init();
    }

    uint8_t address[2] = {(uint8_t)(addr >> 8), (uint8_t)addr};
    esp_err_t ret = i2c_master_transmit_receive(s_eeprom, address, sizeof(address),
                                                databuf, len, EEPROM_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        memset(databuf, 0, len);
        ESP_LOGE(TAG, "EEPROM read failed: %s", esp_err_to_name(ret));
    }
}

void eeprom_24cxx_write_byte_len(uint16_t addr, uint8_t *databuf, uint16_t len)
{
    if (databuf == NULL || len == 0) {
        return;
    }
    if (s_eeprom == NULL) {
        eeprom_24cxx_init();
    }

    uint8_t *packet = malloc((size_t)len + 2);
    if (packet == NULL) {
        ESP_LOGE(TAG, "EEPROM write buffer allocation failed");
        return;
    }
    packet[0] = (uint8_t)(addr >> 8);
    packet[1] = (uint8_t)addr;
    memcpy(&packet[2], databuf, len);

    esp_err_t ret = i2c_master_transmit(s_eeprom, packet, (size_t)len + 2,
                                        EEPROM_I2C_TIMEOUT_MS);
    free(packet);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM write failed: %s", esp_err_to_name(ret));
    }
    /* 等待 EEPROM 内部写周期完成，避免紧接着访问时 NACK。 */
    vTaskDelay(pdMS_TO_TICKS(EEPROM_WRITE_CYCLE_MS));
}

void eeprom_24cxx_write_half_word_len(uint16_t addr, uint16_t *databuf, uint16_t len)
{
    if (databuf == NULL || len == 0) {
        return;
    }

    uint8_t *bytes = malloc((size_t)len * 2);
    if (bytes == NULL) {
        ESP_LOGE(TAG, "EEPROM conversion buffer allocation failed");
        return;
    }
    for (uint16_t i = 0; i < len; ++i) {
        bytes[i * 2] = (uint8_t)(databuf[i] >> 8);
        bytes[i * 2 + 1] = (uint8_t)databuf[i];
    }
    eeprom_24cxx_write_byte_len(addr, bytes, len * 2);
    free(bytes);
}

void eeprom_24cxx_read_half_word_len(uint16_t addr, uint16_t *databuf, uint16_t len)
{
    if (databuf == NULL || len == 0) {
        return;
    }

    uint8_t *bytes = malloc((size_t)len * 2);
    if (bytes == NULL) {
        ESP_LOGE(TAG, "EEPROM conversion buffer allocation failed");
        return;
    }
    eeprom_24cxx_read_byte_len(addr, bytes, len * 2);
    for (uint16_t i = 0; i < len; ++i) {
        databuf[i] = ((uint16_t)bytes[i * 2] << 8) | bytes[i * 2 + 1];
    }
    free(bytes);
}
