#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK      /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_NACK    /*!< I2C nack value */

#define BSP_IIC_DEV I2C_NUM_1

esp_err_t iic_device_master_init(uint8_t dev, uint16_t sda, uint16_t scl, uint32_t freq);


#endif