#include "bsp_iic.h"
#include "esp_log.h"

esp_err_t iic_device_master_init(uint8_t dev, uint16_t sda, uint16_t scl, uint32_t freq)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq,
        .clk_flags = 0,
    };
    i2c_param_config(dev, &conf);
    return i2c_driver_install(dev, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}