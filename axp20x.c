#include "axp20x.h"

#define AXP_NOT(x) ((x)) ? 0 : 1
#define AXP_ASSERT(x) ((x)) ? 1 : 0
#define AXP_BIT(x) (1U << (x))
#define AXP_RANGE(min, x, max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) \
                                                                     : (x)

#define AXP_BUILD_I12(m, l) (((m) << 4) | ((l)&0x0f))

#define AXP_ASSERT_BYTE(x, b) \
    AXP_ASSERT(((x) & (AXP_BIT((b)))))

#define AXP_ERROR_CHECK(x)                                                                \
    do                                                                                    \
    {                                                                                     \
        esp_err_t __err_rc__ = (x);                                                       \
        if (__err_rc__ != ESP_OK)                                                         \
        {                                                                                 \
            ESP_LOGE(TAG, "error%d in line:%d  file:%s", __err_rc__, __LINE__, __FILE__); \
            return __err_rc__;                                                            \
        }                                                                                 \
    } while (0)

const static char *TAG = "axp_pmu";

static const uint16_t LDO4_LUT[16] = {1250, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2500, 2700, 2800, 3000, 3100, 3200, 3300};

static uint8_t axp_ldo4_match_voltage(uint16_t v)
{
    v = AXP_RANGE(1250, v, 3300);
    int16_t delta[16];
    for (uint8_t i = 0; i < 16; i++)
    {
        delta[i] = LDO4_LUT[i] - v;
    }
    for (uint8_t i = 0; i < 16; i++)
    {
        if (delta[i] > 0)
        {
            return i - 1;
        }
    }
    return 0x0f;
}

#ifdef AXP_CHECK_CONN
static esp_err_t axp_checkConnect()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(AXP_IIC_DEV, cmd, 1);
    i2c_cmd_link_delete(cmd);

    return ret;
}
#endif

static esp_err_t axp_iic_readReg(uint8_t address, uint8_t *dst)
{
#ifdef AXP_CHECK_CONN
    AXP_ERROR_CHECK(axp_checkConnect);
#endif
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, dst, NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(AXP_IIC_DEV, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);

    return ret;
}
static esp_err_t axp_iic_writeReg(uint8_t address, uint8_t src)
{
#ifdef AXP_CHECK_CONN
    AXP_ERROR_CHECK(axp_checkConnect);
#endif
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, src, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(AXP_IIC_DEV, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t axp_iic_readRegs(uint8_t address, uint16_t len, uint8_t *dst)
{
#ifdef AXP_CHECK_CONN
    AXP_ERROR_CHECK(axp_checkConnect);
#endif
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, dst, NACK_VAL);
    i2c_master_read(cmd, dst, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(AXP_IIC_DEV, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);

    return ret;
}
// static esp_err_t axp_iic_writeRegs(uint8_t address, uint8_t len, uint8_t *src)
// {
// #ifdef AXP_CHECK_CONN
//     AXP_ERROR_CHECK(axp_checkConnect);
// #endif
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (AXP_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
//     // i2c_master_write_byte(cmd, src, ACK_CHECK_EN);
//     i2c_master_write(cmd, src, len, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(AXP_IIC_DEV, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

/*====================RT APi====================*/
esp_err_t axp_pmic_poweroff(uint8_t allow_gpio_weakUp)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x31;
    esp_err_t rc = ESP_OK;

    if (allow_gpio_weakUp)
    {
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        tmp = tmp | AXP_BIT(3); //此 bit 写完后自动清 0，因此每次进 Sleep 模式前需再次写 1

        rc = axp_iic_writeReg(target_subAddr, tmp);
    }

    target_subAddr = 0x32;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    tmp = tmp | AXP_BIT(7); //此位写 1 会关闭 AXP209 的输出

    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

/*====================read only====================*/
esp_err_t axp_pmic_get_input_status(axp_input_status_t *dst)
{
    memset(dst, 0x00, sizeof(axp_input_status_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x00;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->ACIN_exist = AXP_ASSERT_BYTE(tmp, 7);
    dst->ACIN_availiable = AXP_ASSERT_BYTE(tmp, 6);
    dst->VBUS_exist = AXP_ASSERT_BYTE(tmp, 5);
    dst->VBUS_availiable = AXP_ASSERT_BYTE(tmp, 4);
    dst->VBUS_above_VHOLD = AXP_ASSERT_BYTE(tmp, 3);
    dst->current_direction = AXP_ASSERT_BYTE(tmp, 2);
    dst->ACIN_short_VBUS = AXP_ASSERT_BYTE(tmp, 1);
    dst->boot_power_sources = AXP_ASSERT_BYTE(tmp, 0);

    return rc;
}

esp_err_t axp_pmic_get_power_status(axp_power_status_t *dst)
{
    memset(dst, 0x00, sizeof(axp_power_status_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x01;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->over_temperature = AXP_ASSERT_BYTE(tmp, 7);
    dst->is_charging = AXP_ASSERT_BYTE(tmp, 6);
    dst->battery_exists = AXP_ASSERT_BYTE(tmp, 5);
    dst->battery_active = AXP_ASSERT_BYTE(tmp, 3);
    dst->charging_current_below_expected = AXP_ASSERT_BYTE(tmp, 2);

    return rc;
}

esp_err_t axp_pmic_get_otg_vbus_status(axp_usb_otg_vbus_status_t *dst)
{
    memset(dst, 0x00, sizeof(axp_usb_otg_vbus_status_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x01;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->VBUS_valid = AXP_ASSERT_BYTE(tmp, 2);
    dst->VBUS_Session_A_B_valid = AXP_ASSERT_BYTE(tmp, 1);
    dst->Session_End = AXP_ASSERT_BYTE(tmp, 0);

    return rc;
}

esp_err_t axp_pmic_get_counter_resault(axp_counter_resault_t *dst)
{
    memset(dst, 0x00, sizeof(axp_counter_resault_t));
    uint8_t tmp[10] = {0x00};
    uint8_t target_subAddr = 0xb0;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readRegs(target_subAddr, 10, tmp);

    for (uint8_t i = 0; i < 4; i++)
    {
        dst->coulomb_count_for_charge |= tmp[3 - i] << (8 * i);
        dst->coulomb_count_for_discharge |= tmp[7 - i] << (8 * i);
    }
    dst->electricity = tmp[9] & 0b01111111;

    return rc;
}

esp_err_t axp_pmic_get_adc_resault(axp_adc_resault_t *dst)
{
    uint8_t regmap[27] = {0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x70, 0x71, 0x72, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f};
    memset(dst, 0x00, sizeof(axp_counter_resault_t));
    uint8_t tmp[27] = {0x00};
    esp_err_t rc = ESP_OK;

    for (uint8_t i = 0; i < 27; ++i)
    {
        rc = axp_iic_readReg(regmap[i], &tmp[i]);
        AXP_ERROR_CHECK(rc);
    }

    dst->ACIN_voltage = AXP_BUILD_I12(tmp[0], tmp[1]);
    dst->ACIN_current = AXP_BUILD_I12(tmp[2], tmp[4]);
    dst->VBUS_voltage = AXP_BUILD_I12(tmp[4], tmp[5]);
    dst->VBUS_current = AXP_BUILD_I12(tmp[6], tmp[7]);
    dst->internal_temperature = AXP_BUILD_I12(tmp[8], tmp[9]);
    dst->TS = AXP_BUILD_I12(tmp[10], tmp[11]);
    dst->GPIO0 = AXP_BUILD_I12(tmp[12], tmp[13]);
    dst->GPIO1 = AXP_BUILD_I12(tmp[14], tmp[15]);
    dst->battery_Instantaneous_power = (tmp[16] << 16) | (tmp[17] << 8) | tmp[18];
    dst->battery_voltage = AXP_BUILD_I12(tmp[19], tmp[20]);
    dst->battery_charging_current = AXP_BUILD_I12(tmp[21], tmp[22]);
    dst->battery_discharging_current = (tmp[23] << 5) | (tmp[24] & 0x1f);
    dst->IPS_voltage = AXP_BUILD_I12(tmp[25], tmp[26]);

    //axp209 p25
    dst->ACIN_voltage = (float)dst->ACIN_voltage * 1.7f;
    dst->ACIN_current = (float)dst->ACIN_current * 0.625f;
    dst->VBUS_voltage = (float)dst->VBUS_voltage * 1.7f;
    dst->VBUS_current = (float)dst->VBUS_current * 0.375f;
    dst->internal_temperature = (float)dst->internal_temperature * 0.1f;
    dst->TS = (float)dst->TS * 0.8f;
    dst->GPIO0 = (float)dst->GPIO0 * 0.5f;
    dst->GPIO1 = (float)dst->GPIO1 * 0.5f;
    //dst->battery_Instantaneous_power = (float)dst->battery_Instantaneous_power * 1.1f;
    dst->battery_voltage = (float)dst->battery_voltage * 1.1f;
    dst->battery_charging_current = (float)dst->battery_charging_current * 0.5f;
    dst->battery_discharging_current = (float)dst->battery_discharging_current * 0.5f;
    dst->IPS_voltage = (float)dst->IPS_voltage * 1.4f;

    return rc;
}

uint8_t axp_pmic_get_gpio_status(axp_gpio_channel_t ch)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    target_subAddr = 0x94;
    axp_iic_readReg(target_subAddr, &tmp);

    switch (ch)
    {
    case AXP_GPIO0:
        return AXP_ASSERT_BYTE(tmp, 4);
        break;
    case AXP_GPIO1:
        return AXP_ASSERT_BYTE(tmp, 5);
        break;
    case AXP_GPIO2:
        return AXP_ASSERT_BYTE(tmp, 6);
        break;
    case AXP_GPIO3:
        target_subAddr = 0x95;
        axp_iic_readReg(target_subAddr, &tmp);
        return AXP_NOT((AXP_ASSERT_BYTE(tmp, 0)));
        break;

    default:
        ESP_LOGE(TAG, "no this ch");
        break;
    }
    return 0;
}

/*====================read====================*/
esp_err_t axp_pmic_read_data_ram(axp_data_flash_t *dst)
{
    esp_err_t rc = ESP_OK;
    for (size_t i = 0; i < 12; i++)
    {
        rc = axp_iic_readReg((0x04 + i), &dst->data_flash[i]);
        AXP_ERROR_CHECK(rc);
    }
    return rc;
}

esp_err_t axp_pmic_read_data_ram_addr(uint8_t addr, uint8_t *dst, uint8_t len)
{
    //addr:0...11(0x00->0x0b)
    if (addr > 11)
        addr = 11;
    if (len > (12 - addr))
        len = (12 - addr);
    esp_err_t rc = ESP_OK;
    for (size_t i = 0; i < len; i++)
    {
        rc = axp_iic_readReg((0x04 + addr + i), &dst[i]);
        AXP_ERROR_CHECK(rc);
    }
    return rc;
}

/*====================irq====================*/
esp_err_t axp_pmic_irq_handle(axp_irq_status_t *dst)
{
    /*
    在某些特定事件发生时，AXP209 通过拉低 IRQ 的中断机制来提醒 Host，并将中断状态保存在中断
    状态寄存器中(参见寄存器 REG48H、寄存器 REG49H、寄存器 REG4AH、寄存器 REG4BH、寄存器
    REG4CH)，向相应的状态寄存器位写 1 则清除相应的中断，当无中断事件时，IRQ 输出拉高(通过外部上
    拉 51K 电阻)。每个中断都可以通过中断控制寄存器来屏蔽(参见寄存器 REG40H、寄存器 REG41H、寄
    存器 REG42H、寄存器 REG43H、寄存器 REG44H)。

    */
    memset(dst, 0x00, sizeof(axp_irq_status_t));
    uint8_t tmp[5] = {0x00};
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x48;
    rc = axp_iic_readRegs(target_subAddr, 5, tmp);
    // ESP_LOGI(TAG, "%x  %x  %x  %x  %x", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4]);
    AXP_ERROR_CHECK(rc);

    for (uint8_t i = 0; i < 5; i++)
    {
        dst->regmap |= ((uint64_t)tmp[i] << (8 * i));
        // ESP_LOGI(TAG,"i = %d  ,  tmp = %x   ,   dst = %llx",i,tmp[i],dst->regmap);
    }

    //向相应的状态寄存器位写 1 ,清除相应的中断
    rc = axp_iic_writeReg(0x48, tmp[0]);
    AXP_ERROR_CHECK(rc);
    rc = axp_iic_writeReg(0x49, tmp[1]);
    AXP_ERROR_CHECK(rc);
    rc = axp_iic_writeReg(0x4a, tmp[2]);
    AXP_ERROR_CHECK(rc);
    rc = axp_iic_writeReg(0x4b, tmp[3]);
    AXP_ERROR_CHECK(rc);
    rc = axp_iic_writeReg(0x4c, tmp[4]);

    return rc;
}

/*====================write====================*/
esp_err_t axp_pmic_write_data_ram(axp_data_flash_t dat)
{
    esp_err_t rc = ESP_OK;
    for (size_t i = 0; i < 12; i++)
    {
        axp_iic_writeReg((0x04 + i), dat.data_flash[i]);
        AXP_ERROR_CHECK(rc);
    }

    return rc;
}

esp_err_t axp_pmic_write_data_ram_addr(uint8_t addr, uint8_t *dat, uint8_t len)
{
    //addr:0...11(0x00->0x0b)
    if (addr > 11)
        addr = 11;
    if (len > (12 - addr))
        len = (12 - addr);
    esp_err_t rc = ESP_OK;
    for (size_t i = 0; i < len; i++)
    {
        rc = axp_iic_writeReg((0x04 + addr + i), dat[i]);
        AXP_ERROR_CHECK(rc);
    }
    return rc;
}

esp_err_t axp_pmic_set_power_output(axp_power_output_channel_t ch, axp_power_out_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x12;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    switch (ch)
    {
    case AXP_DCDC2:
        target_subAddr = 0x12;
        if (cfg.enable)
        {
            tmp = tmp | AXP_BIT(4);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(4)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 12

        target_subAddr = 0x23;
        cfg.voltage = AXP_RANGE(700, cfg.voltage, 2275);
        cfg.voltage = cfg.voltage - 700;
        cfg.voltage = (cfg.voltage / 25);
        tmp = cfg.voltage & 0b00111111;
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 23

        target_subAddr = 0x25;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        if (cfg.vrc_enable)
        {
            tmp = tmp | AXP_BIT(2);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(2)));
        }
        if (cfg.vrc_config)
        {
            tmp = tmp | AXP_BIT(0);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(0)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 25

        break; // dcdc2

    case AXP_DCDC3:
        target_subAddr = 0x12;
        if (cfg.enable)
        {
            tmp = tmp | AXP_BIT(1);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(1)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 12

        target_subAddr = 0x27;
        cfg.voltage = AXP_RANGE(700, cfg.voltage, 3500);
        cfg.voltage = cfg.voltage - 700;
        cfg.voltage = (cfg.voltage / 25);
        tmp = cfg.voltage & 0b01111111;
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 27

        break;
    case AXP_LDO2:
        target_subAddr = 0x12;
        if (cfg.enable)
        {
            tmp = tmp | AXP_BIT(2);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(2)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 12

        target_subAddr = 0x28;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        cfg.voltage = AXP_RANGE(1800, cfg.voltage, 3300);
        cfg.voltage = cfg.voltage - 1800;
        cfg.voltage = (cfg.voltage / 100);
        tmp &= 0b00001111;
        tmp |= (cfg.voltage & 0b00001111) << 4;
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 28

        break;
    case AXP_LDO3:
        target_subAddr = 0x12;
        if (cfg.enable)
        {
            tmp = tmp | AXP_BIT(6);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(6)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 12

        target_subAddr = 0x25;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        if (cfg.vrc_enable)
        {
            tmp = tmp | AXP_BIT(3);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(3)));
        }
        if (cfg.vrc_config)
        {
            tmp = tmp | AXP_BIT(1);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(1)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 25

        target_subAddr = 0x29;
        cfg.voltage = AXP_RANGE(700, cfg.voltage, 2275);
        cfg.voltage = cfg.voltage - 700;
        cfg.voltage = (cfg.voltage / 25);
        tmp = cfg.voltage & 0b01111111;
        if (cfg.mode)
        {
            tmp = tmp | AXP_BIT(7);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(7)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 29

        break;
    case AXP_LDO4:
        target_subAddr = 0x12;
        if (cfg.enable)
        {
            tmp = tmp | AXP_BIT(3);
        }
        else
        {
            tmp = tmp & (~(AXP_BIT(3)));
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 12

        target_subAddr = 0x28;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        tmp &= 0b11110000;
        tmp |= (axp_ldo4_match_voltage(cfg.voltage) & 0b00001111);
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 28

        break;
    case AXP_LDO5:
        target_subAddr = 0x90;
        if (cfg.enable)
        {
            tmp = tmp & 0b11111000;
            tmp |= 0b00000011;
        }
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 90

        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        cfg.voltage = AXP_RANGE(1800, cfg.voltage, 3300);
        cfg.voltage = cfg.voltage - 1800;
        cfg.voltage = (cfg.voltage / 100);
        tmp &= 0b00001111;
        tmp |= (cfg.voltage & 0b00001111) << 4;
        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 91

        break;
    default:
        ESP_LOGE(TAG, "no such channel");
        break;
    }

    return rc;
}

esp_err_t axp_pmic_set_VBUS_IPSOUT_channel_config(axp_VBUS_IPSOUT_channel_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x30;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    if (cfg.from)
    {
        tmp = tmp | AXP_BIT(7);
    }
    else
    {
        tmp = tmp & (~(AXP_BIT(7)));
    }
    if (cfg.VBUS_VHOLD_enable)
    {
        tmp = tmp | AXP_BIT(6);
    }
    else
    {
        tmp = tmp & (~(AXP_BIT(6)));
    }

    cfg.VHOLD_voltage = AXP_RANGE(4000, cfg.VHOLD_voltage, 4700);
    cfg.VHOLD_voltage = cfg.VHOLD_voltage - 4000;
    cfg.VHOLD_voltage = (cfg.VHOLD_voltage / 100);
    tmp = tmp & 0b11000100;
    tmp |= (cfg.VHOLD_voltage & 0b00000111) << 3;
    tmp |= (cfg.VBUS_current_imit_selection & 0b00000011);
    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_VOFF_config(axp_VOFF_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x31;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    if (cfg.PEK_GPIO_edge_weakUp_enable)
    {
        tmp = tmp | AXP_BIT(3);
    }
    else
    {
        tmp = tmp & (~(AXP_BIT(3)));
    }

    cfg.VOFF_set_voltage = AXP_RANGE(2600, cfg.VOFF_set_voltage, 3300);
    cfg.VOFF_set_voltage = cfg.VOFF_set_voltage - 2600;
    cfg.VOFF_set_voltage = (cfg.VOFF_set_voltage / 100);
    tmp = tmp & 0b11111000;
    tmp |= (cfg.VOFF_set_voltage & 0b00000111);
    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_power_off_config(axp_power_off_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x32;
    esp_err_t rc = ESP_OK;

    if (cfg.shutdown)
    {
        tmp = tmp | AXP_BIT(7);
    }
    if (cfg.power_monitor_enable)
    {
        tmp = tmp | AXP_BIT(6);
    }
    if (cfg.CHGLED_auto)
    {
        tmp = tmp | AXP_BIT(3);
    }
    if (cfg.shutdown_time_config)
    {
        tmp = tmp | AXP_BIT(2);
    }

    tmp |= (cfg.CHGLED_config & 0b00000011) << 4;
    tmp |= (cfg.shutdown_delay & 0b00000011);

    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_charge_config(axp_charge_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x33;
    esp_err_t rc = ESP_OK;
    if (cfg.enable)
    {
        tmp = tmp | AXP_BIT(7);
    }
    if (cfg.end_current)
    {
        tmp = tmp | AXP_BIT(4);
    }

    tmp |= (cfg.target_voltage & 0b00000011) << 5;

    cfg.current = AXP_RANGE(300, cfg.current, 1800);
    cfg.current = cfg.current - 300;
    cfg.current = (cfg.current / 100);
    tmp |= (cfg.current & 0b00001111);

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 33 end

    target_subAddr = 0x34;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    if (cfg.CHGLED_flash_config)
    {
        tmp = tmp | AXP_BIT(4);
    }
    else
    {
        tmp = tmp & (~(AXP_BIT(4)));
    }

    tmp = tmp & 0b00111100;

    tmp |= (cfg.pre_charge_tomeout & 0b00000011) << 6;
    tmp |= (cfg.CC_mode_timeout & 0b00000011);

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 34 end

    target_subAddr = 0x35;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    if (cfg.backup_battery_enable)
    {
        tmp = tmp | AXP_BIT(7);
    }
    else
    {
        tmp = tmp & (~(AXP_BIT(7)));
    }

    tmp = tmp & 0b10011100;

    tmp |= (cfg.backup_battery_target_voltage & 0b00000011) << 5;
    tmp |= (cfg.backup_battery_current & 0b00000011);

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 35 end

    target_subAddr = 0x38;
    // warning : float number
    cfg.VLTF_charge = AXP_RANGE(0.0f, cfg.VLTF_charge, 3264.0f);
    cfg.VLTF_charge = cfg.VLTF_charge / 0.8f;
    cfg.VLTF_charge = cfg.VLTF_charge / 16.0f;
    tmp = (int)cfg.VLTF_charge;

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 38 end

    target_subAddr = 0x39;
    // warning : float number
    cfg.VHTF_charge = AXP_RANGE(0.0f, cfg.VHTF_charge, 3264.0f);
    cfg.VHTF_charge = cfg.VHTF_charge / 0.8f;
    cfg.VHTF_charge = cfg.VHTF_charge / 16.0f;
    tmp = (int)cfg.VHTF_charge;

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 39 end

    target_subAddr = 0x3c;
    // warning : float number
    cfg.VLTF_discharge = AXP_RANGE(0.0f, cfg.VLTF_discharge, 3264.0f);
    cfg.VLTF_discharge = cfg.VLTF_discharge / 0.8f;
    cfg.VLTF_discharge = cfg.VLTF_discharge / 16.0f;
    tmp = (int)cfg.VLTF_discharge;

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // 3c end

    target_subAddr = 0x3d;
    // warning : float number
    cfg.VHTF_discharge = AXP_RANGE(0.0f, cfg.VHTF_discharge, 3264.0f);
    cfg.VHTF_discharge = cfg.VHTF_discharge / 0.8f;
    cfg.VHTF_discharge = cfg.VHTF_discharge / 16.0f;
    tmp = (int)cfg.VHTF_discharge;

    rc = axp_iic_writeReg(target_subAddr, tmp);
    // 3d end

    return rc;
}

esp_err_t axp_pmic_set_PEK_key_config(axp_PEK_key_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x36;
    esp_err_t rc = ESP_OK;

    if (cfg.long_press_to_shutdown)
    {
        tmp = tmp | AXP_BIT(3);
    }

    if (cfg.power_ok_singal_delay)
    {
        tmp = tmp | AXP_BIT(2);
    }

    tmp |= (cfg.power_on_press_time & 0b00000011) << 6;
    tmp |= (cfg.long_press_time & 0b00000011) << 4;
    tmp |= (cfg.power_off_delay & 0b00000011);
    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_DCDC_physics_config(axp_DCDC_physics_config_t cfg)
{
    //<NOT USEFUL>
    /*
    10.18 REG 37H:DC-DC 工作频率设置
    每一级改变 5%，默认值 1.5MHz
    F=[1+/- (Bit3-0)*5%)]*1.5MHz
    (0b 0000 1000)

    10.25 REG 80H:DC-DC 工作模式选择
    2 DC-DC2 工作模式控制
    1 DC-DC3 工作模式控制

    0:PFM/PWM 自动切换
    1:固定 PWM
    */
    return ESP_OK;
}

esp_err_t axp_pmic_set_IPSOUT_warning_level_config(axp_IPSOUT_warning_level_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x3a;
    esp_err_t rc = ESP_OK;
    // warning : float number
    cfg.level1 = AXP_RANGE(2867.2f, cfg.level1, 4295.2f);
    cfg.level1 = cfg.level1 - 2867.2f;
    cfg.level1 = cfg.level1 / 4.0f;
    cfg.level1 = cfg.level1 / 1.4f;
    tmp = (int)cfg.level1;
    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 3a

    target_subAddr = 0x3b;
    // warning : float number
    cfg.level2 = AXP_RANGE(2867.2f, cfg.level2, 4295.2f);
    cfg.level2 = cfg.level2 - 2867.2f;
    cfg.level2 = cfg.level2 / 4.0f;
    cfg.level2 = cfg.level2 / 1.4f;
    tmp = (int)cfg.level2;
    rc = axp_iic_writeReg(target_subAddr, tmp);
    // end 3b

    return rc;
}

esp_err_t axp_pmic_set_adc_config(axp_adc_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x82;
    esp_err_t rc = ESP_OK;

    if (cfg.batt_voltage_enable)
        tmp = tmp | AXP_BIT(7);
    if (cfg.batt_current_enable)
        tmp = tmp | AXP_BIT(6);
    if (cfg.acin_voltage_enable)
        tmp = tmp | AXP_BIT(5);
    if (cfg.acin_current_enable)
        tmp = tmp | AXP_BIT(4);
    if (cfg.vbus_voltage_enable)
        tmp = tmp | AXP_BIT(3);
    if (cfg.vbus_current_enable)
        tmp = tmp | AXP_BIT(2);
    if (cfg.aps_voltage_enable)
        tmp = tmp | AXP_BIT(1);
    if (cfg.ts_pin_enable)
        tmp = tmp | AXP_BIT(0);

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 82

    target_subAddr = 0x83;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.internal_temperature_detection_enable)
        tmp = tmp | AXP_BIT(7);
    else
        tmp = tmp & (~(AXP_BIT(7)));

    if (cfg.GPIO1_analog_in_enable)
        tmp = tmp | AXP_BIT(3);
    else
        tmp = tmp & (~(AXP_BIT(3)));

    if (cfg.GPIO0_analog_in_enable)
        tmp = tmp | AXP_BIT(2);
    else
        tmp = tmp & (~(AXP_BIT(2)));

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 83

    target_subAddr = 0x84;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    tmp = tmp & 0b00111111;
    tmp |= (cfg.sample_rate & 0b00000011) << 6;

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 84

    target_subAddr = 0x85;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.gpio1_adc_range)
        tmp = tmp | AXP_BIT(1);
    else
        tmp = tmp & (~(AXP_BIT(1)));

    if (cfg.gpio0_adc_range)
        tmp = tmp | AXP_BIT(0);
    else
        tmp = tmp & (~(AXP_BIT(0)));

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 85

    target_subAddr = 0x86;
    cfg.gpio1_adc_interrupt_raising_Threshold = AXP_RANGE(0, cfg.gpio1_adc_interrupt_raising_Threshold, 2040);
    tmp = cfg.gpio1_adc_interrupt_raising_Threshold / 8;
    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end 86

    target_subAddr = 0x87;
    cfg.gpio1_adc_interrupt_falling_Threshold = AXP_RANGE(0, cfg.gpio1_adc_interrupt_falling_Threshold, 2040);
    tmp = cfg.gpio1_adc_interrupt_falling_Threshold / 8;
    rc = axp_iic_writeReg(target_subAddr, tmp);
    // end 87

    return rc;
}

esp_err_t axp_pmic_set_ts_pin_config(axp_ts_pin_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x84;
    esp_err_t rc = ESP_OK;

    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.ts_mode)
        tmp = tmp | AXP_BIT(2);
    else
        tmp = tmp & (~(AXP_BIT(2)));

    tmp = tmp & 11001100;
    tmp |= (cfg.ts_output_current & 0b00000011) << 4;
    tmp |= (cfg.ts_output_method & 0b00000011);

    rc = axp_iic_writeReg(target_subAddr, tmp);
    // end 84

    return rc;
}

esp_err_t axp_pmic_set_timer_config(axp_timer_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x8a;
    esp_err_t rc = ESP_OK;

    if (cfg.clear)
        tmp = tmp | AXP_BIT(7);

    tmp |= (cfg.set_timeout & 0b01111111);

    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_srp_monitor_config(axp_srp_monitor_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x8b;
    esp_err_t rc = ESP_OK;

    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.vbus_valid_detection_enable)
        tmp = tmp | AXP_BIT(3);
    else
        tmp = tmp & (~(AXP_BIT(3)));
    if (cfg.vbus_session_detection_enable)
        tmp = tmp | AXP_BIT(2);
    else
        tmp = tmp & (~(AXP_BIT(2)));
    if (cfg.discharge_VBUS_enable)
        tmp = tmp | AXP_BIT(1);
    else
        tmp = tmp & (~(AXP_BIT(1)));
    if (cfg.charge_VBUS_enable)
        tmp = tmp | AXP_BIT(0);
    else
        tmp = tmp & (~(AXP_BIT(0)));

    tmp = tmp & 0b11001111;
    tmp |= (cfg.vbus_minium_voltage & 0b00000011) << 4;

    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_over_temperature_poweroff_config(axp_over_temperature_poweroff_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x8f;
    esp_err_t rc = ESP_OK;

    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.enable)
        tmp = tmp | AXP_BIT(2);
    else
        tmp = tmp & (~(AXP_BIT(2)));

    rc = axp_iic_writeReg(target_subAddr, tmp);

    return rc;
}

esp_err_t axp_pmic_set_GPIO_config(axp_gpio_channel_t ch, axp_gpio_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;

    switch (ch)
    {
    case AXP_GPIO0:
        target_subAddr = 0x90;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        if (cfg.interrupt_raising_wakeup_enable)
            tmp = tmp | AXP_BIT(7);
        else
            tmp = tmp & (~(AXP_BIT(7)));

        if (cfg.interrupt_filling_wakeup_enable)
            tmp = tmp | AXP_BIT(6);
        else
            tmp = tmp & (~(AXP_BIT(6)));

        tmp = tmp & 0b11111000;

        switch (cfg.mode)
        {
        case AXP_GPIO_MODE_OUTPUT_LOW:
            tmp |= 0b00000000;
            break;
        case AXP_GPIO_MODE_OUTPUT_HIGH:
            tmp |= 0b00000001;
            break;
        case AXP_GPIO_MODE_INPUT:
            tmp |= 0b00000010;
            break;
        case AXP_GPIO_MODE_LDO:
            tmp |= 0b00000011;
            break;
        case AXP_GPIO_MODE_ADC_IN:
            tmp |= 0b00000100;
            break;
        case AXP_GPIO_MODE_FLOAT:
            tmp |= 0b00000111;
            break;

        default:
            ESP_LOGE(TAG, "no this");
            break;
        } // mode

        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);
        // end 90

        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        tmp = tmp & 0b11111000;

        tmp = tmp | (cfg.gpio_high_voltage & 0b00000111);

        rc = axp_iic_writeReg(target_subAddr, tmp);
        // end 91

        break;
    case AXP_GPIO1:
        target_subAddr = 0x92;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        if (cfg.interrupt_raising_wakeup_enable)
            tmp = tmp | AXP_BIT(7);
        else
            tmp = tmp & (~(AXP_BIT(7)));

        if (cfg.interrupt_filling_wakeup_enable)
            tmp = tmp | AXP_BIT(6);
        else
            tmp = tmp & (~(AXP_BIT(6)));

        tmp = tmp & 0b11111000;

        switch (cfg.mode)
        {
        case AXP_GPIO_MODE_OUTPUT_LOW:
            tmp |= 0b00000000;
            break;
        case AXP_GPIO_MODE_OUTPUT_HIGH:
            tmp |= 0b00000001;
            break;
        case AXP_GPIO_MODE_INPUT:
            tmp |= 0b00000010;
            break;
        case AXP_GPIO_MODE_LDO:
            tmp |= 0b00000011;
            break;
        case AXP_GPIO_MODE_ADC_IN:
            tmp |= 0b00000100;
            break;
        case AXP_GPIO_MODE_FLOAT:
            tmp |= 0b00000111;
            break;

        default:
            ESP_LOGE(TAG, "no this");
            break;
        } // mode

        rc = axp_iic_writeReg(target_subAddr, tmp);
        AXP_ERROR_CHECK(rc);

        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        tmp = tmp & 0b11111000;

        tmp = tmp | (cfg.gpio_high_voltage & 0b00000111);

        rc = axp_iic_writeReg(target_subAddr, tmp);

        break;
    case AXP_GPIO2:
        target_subAddr = 0x93;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        if (cfg.interrupt_raising_wakeup_enable)
            tmp = tmp | AXP_BIT(7);
        else
            tmp = tmp & (~(AXP_BIT(7)));

        if (cfg.interrupt_filling_wakeup_enable)
            tmp = tmp | AXP_BIT(6);
        else
            tmp = tmp & (~(AXP_BIT(6)));

        tmp = tmp & 0b11111000;

        switch (cfg.mode)
        {
        case AXP_GPIO_MODE_OUTPUT_LOW:
            tmp |= 0b00000000;
            break;
        case AXP_GPIO_MODE_FLOAT:
            tmp |= 0b00000001;
            break;
        case AXP_GPIO_MODE_INPUT:
            tmp |= 0b00000010;
            break;
        default:
            ESP_LOGE(TAG, "no this");
            break;
        } // mode

        rc = axp_iic_writeReg(target_subAddr, tmp);
        break;
    case AXP_GPIO3:
        target_subAddr = 0x95;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);

        if (cfg.interrupt_raising_wakeup_enable)
            tmp = tmp | AXP_BIT(7);
        else
            tmp = tmp & (~(AXP_BIT(7)));

        if (cfg.interrupt_filling_wakeup_enable)
            tmp = tmp | AXP_BIT(6);
        else
            tmp = tmp & (~(AXP_BIT(6)));

        tmp = tmp & 0b11111001;

        switch (cfg.mode)
        {
        case AXP_GPIO_MODE_OUTPUT_LOW:
            tmp |= 0b00000000; // nmos_od | nmos_on
            break;
        case AXP_GPIO_MODE_FLOAT:
            tmp |= 0b00000010; // nmos_od | nmos_off
            break;
        case AXP_GPIO_MODE_INPUT:
            tmp |= 0b00000110; // input | nmos_off
            break;
        default:
            ESP_LOGE(TAG, "no this");
            break;
        } // mode

        rc = axp_iic_writeReg(target_subAddr, tmp);
        break;
    default:
        ESP_LOGE(TAG, "no this channel");
        break;
    }

    return rc;
}

esp_err_t axp_pmic_set_interrupt_config(axp_irq_config_t cfg)
{
    uint8_t tmp[5] = {0x00};
    esp_err_t rc = ESP_OK;

    for (uint8_t i = 0; i < 5; i++)
    {
        tmp[i] = (cfg.regmap >> (8 * i)) & 0xff;
    }
    rc =axp_iic_writeReg(0x40,tmp[0]);
    AXP_ERROR_CHECK(rc);
    rc =axp_iic_writeReg(0x41,tmp[1]);
    AXP_ERROR_CHECK(rc);
    rc =axp_iic_writeReg(0x42,tmp[2]);
    AXP_ERROR_CHECK(rc);
    rc =axp_iic_writeReg(0x43,tmp[3]);
    AXP_ERROR_CHECK(rc);
    rc =axp_iic_writeReg(0x44,tmp[4]);
    return rc;
}

esp_err_t axp_pmic_set_coulomb_counter_config(axp_coulomb_counter_config_t cfg)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0xb8;
    esp_err_t rc = ESP_OK;

    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.enable)
        tmp = tmp | AXP_BIT(7);
    else
        tmp = tmp & (~(AXP_BIT(7)));

    if (cfg.coulomb_counter_pause)
        tmp = tmp | AXP_BIT(6);
    else
        tmp = tmp & (~(AXP_BIT(6)));

    if (cfg.reset)
        tmp = tmp | AXP_BIT(5);
    else
        tmp = tmp & (~(AXP_BIT(5)));

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end B8
    target_subAddr = 0xb9;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    if (cfg.electricity_counter_pause)
        tmp = tmp | AXP_BIT(7);
    else
        tmp = tmp & (~(AXP_BIT(7)));

    rc = axp_iic_writeReg(target_subAddr, tmp);
    AXP_ERROR_CHECK(rc);
    // end B9

    return rc;
}

/*========================================read conf========================================*/
esp_err_t axp_pmic_get_power_output(axp_power_output_channel_t ch, axp_power_out_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_power_out_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr = 0x12;
    esp_err_t rc = ESP_OK;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    switch (ch)
    {
    case AXP_DCDC2:
        dst->enable = AXP_ASSERT_BYTE(tmp, 4);

        target_subAddr = 0x23;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        tmp = tmp & 0b00111111;
        dst->voltage = 700 + (tmp * 25);

        target_subAddr = 0x25;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        dst->vrc_enable = AXP_ASSERT_BYTE(tmp, 2);
        dst->vrc_config = AXP_ASSERT_BYTE(tmp, 0);
        break; // dcdc2
    case AXP_DCDC3:
        dst->enable = AXP_ASSERT_BYTE(tmp, 1);

        target_subAddr = 0x27;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        tmp = tmp & 0b01111111;
        dst->voltage = 700 + (tmp * 25);

        break; // dcdc3
    case AXP_LDO2:
        dst->enable = AXP_ASSERT_BYTE(tmp, 2);

        target_subAddr = 0x28;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        tmp = tmp & 0b11110000;
        tmp = tmp >> 4;
        dst->voltage = 1800 + (tmp * 100);

        break; // ldo2
    case AXP_LDO3:
        dst->enable = AXP_ASSERT_BYTE(tmp, 6);

        target_subAddr = 0x29;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        dst->mode = AXP_ASSERT_BYTE(tmp, 7);
        tmp = tmp & 0b01111111;
        dst->voltage = 700 + (tmp * 25);

        target_subAddr = 0x25;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        dst->vrc_enable = AXP_ASSERT_BYTE(tmp, 3);
        dst->vrc_config = AXP_ASSERT_BYTE(tmp, 1);

        break; // ldo3
    case AXP_LDO4:
        dst->enable = AXP_ASSERT_BYTE(tmp, 3);

        target_subAddr = 0x28;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        tmp = tmp & 0b00001111;
        dst->voltage = LDO4_LUT[tmp];

        break; // ldo4
    case AXP_LDO5:
        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        tmp = tmp >> 4;
        dst->voltage = 1800 + (tmp * 100);

        break;

    default:
        ESP_LOGE(TAG, "power no this channel");
        break;
    } // channel

    return rc;
}

esp_err_t axp_pmic_get_VBUS_IPSOUT_channel_config(axp_VBUS_IPSOUT_channel_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_VBUS_IPSOUT_channel_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x30;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->from = AXP_ASSERT_BYTE(tmp, 7);
    dst->VBUS_VHOLD_enable = AXP_ASSERT_BYTE(tmp, 6);

    dst->VHOLD_voltage = 4000 + (((tmp & 0b00111000) >> 3) * 100);

    dst->VBUS_current_imit_selection = (tmp & 0b00000011);
    return rc;
}

esp_err_t axp_pmic_get_VOFF_config(axp_VOFF_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_VOFF_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x31;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->PEK_GPIO_edge_weakUp_enable = AXP_ASSERT_BYTE(tmp, 3);
    tmp = tmp & 0b00000111;
    dst->VOFF_set_voltage = 2600 + (tmp * 100);

    return rc;
}

esp_err_t axp_pmic_get_power_off_config(axp_power_off_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_power_off_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x32;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->shutdown = AXP_ASSERT_BYTE(tmp, 7);
    dst->power_monitor_enable = AXP_ASSERT_BYTE(tmp, 6);
    dst->CHGLED_auto = AXP_ASSERT_BYTE(tmp, 3);
    dst->shutdown_time_config = AXP_ASSERT_BYTE(tmp, 2);

    dst->CHGLED_config = (tmp & 0b00110000) >> 4;
    dst->shutdown_delay = tmp & 0b00000011;
    return rc;
}

esp_err_t axp_pmic_get_charge_config(axp_charge_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_charge_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;

    target_subAddr = 0x33;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->enable = AXP_ASSERT_BYTE(tmp, 7);
    dst->end_current = AXP_ASSERT_BYTE(tmp, 4);
    dst->target_voltage = (tmp & 0b01100000) >> 5;
    dst->current = 300 + ((tmp & 0b00001111) * 100);

    target_subAddr = 0x34;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->CHGLED_flash_config = AXP_ASSERT_BYTE(tmp, 4);
    dst->pre_charge_tomeout = (tmp & 0b11000000) >> 6;
    dst->CC_mode_timeout = (tmp & 0b00000011);

    target_subAddr = 0x35;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->backup_battery_enable = AXP_ASSERT_BYTE(tmp, 7);
    dst->backup_battery_target_voltage = (tmp & 0b01100000) >> 5;
    dst->backup_battery_current = (tmp & 0b00000011);

    target_subAddr = 0x38;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->VLTF_charge = (tmp * 0x10) * 0.8f;

    target_subAddr = 0x39;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->VHTF_charge = (tmp * 0x10) * 0.8f;

    target_subAddr = 0x3c;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->VLTF_discharge = (tmp * 0x10) * 0.8f;

    target_subAddr = 0x3d;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    dst->VHTF_discharge = (tmp * 0x10) * 0.8f;

    return rc;
}

esp_err_t axp_pmic_get_PEK_key_config(axp_PEK_key_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_PEK_key_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x36;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    dst->power_on_press_time = (tmp & 0b11000000) >> 6;
    dst->long_press_time = (tmp & 0b00110000) >> 4;
    dst->long_press_to_shutdown = AXP_ASSERT_BYTE(tmp, 3);
    dst->power_ok_singal_delay = AXP_ASSERT_BYTE(tmp, 2);
    dst->power_off_delay = (tmp & 0b00000011);
    return rc;
}

esp_err_t axp_pmic_get_DCDC_physics_config(axp_DCDC_physics_config_t *dst)
{
    //<TODO>
    // uint8_t tmp = 0x00;
    // uint8_t target_subAddr;
    // esp_err_t rc = ESP_OK;
    // target_subAddr = 0000000000;
    // rc = axp_iic_readReg(target_subAddr, &tmp);
    // AXP_ERROR_CHECK(rc);

    // return rc
    return ESP_OK;
}

esp_err_t axp_pmic_get_IPSOUT_warning_level_config(axp_IPSOUT_warning_level_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_IPSOUT_warning_level_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x3a;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);
    dst->level1 = 2867.2f + (1.4f * tmp * 4.0);

    target_subAddr = 0x3b;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    dst->level2 = 2867.2f + (1.4f * tmp * 4.0);

    return rc;
}

esp_err_t axp_pmic_get_adc_config(axp_adc_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_adc_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x82;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->batt_voltage_enable = AXP_ASSERT_BYTE(tmp, 7);
    dst->batt_current_enable = AXP_ASSERT_BYTE(tmp, 6);
    dst->acin_voltage_enable = AXP_ASSERT_BYTE(tmp, 5);
    dst->acin_current_enable = AXP_ASSERT_BYTE(tmp, 4);
    dst->vbus_voltage_enable = AXP_ASSERT_BYTE(tmp, 3);
    dst->vbus_current_enable = AXP_ASSERT_BYTE(tmp, 2);
    dst->aps_voltage_enable = AXP_ASSERT_BYTE(tmp, 1);
    dst->ts_pin_enable = AXP_ASSERT_BYTE(tmp, 0);

    target_subAddr = 0x83;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->internal_temperature_detection_enable = AXP_ASSERT_BYTE(tmp, 7);
    dst->GPIO1_analog_in_enable = AXP_ASSERT_BYTE(tmp, 3);
    dst->GPIO0_analog_in_enable = AXP_ASSERT_BYTE(tmp, 2);

    target_subAddr = 0x84;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->sample_rate = (tmp) >> 6;

    target_subAddr = 0x85;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->gpio1_adc_range = AXP_ASSERT_BYTE(tmp, 1);
    dst->gpio0_adc_range = AXP_ASSERT_BYTE(tmp, 0);

    target_subAddr = 0x86;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->gpio1_adc_interrupt_raising_Threshold = tmp * 8;

    target_subAddr = 0x87;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->gpio1_adc_interrupt_falling_Threshold = tmp * 8;
    return rc;
}

esp_err_t axp_pmic_get_ts_pin_config(axp_ts_pin_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_ts_pin_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x84;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->ts_output_current = (tmp & 0b00110000) >> 4;
    dst->ts_output_method = (tmp & 0b00000011);
    dst->ts_mode = AXP_ASSERT_BYTE(tmp, 2);

    return rc;
}

esp_err_t axp_pmic_get_timer_config(axp_timer_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_timer_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x8a;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->clear = AXP_ASSERT_BYTE(tmp, 7);
    dst->set_timeout = tmp & 0b01111111;

    return rc;
}

esp_err_t axp_pmic_get_srp_monitor_config(axp_srp_monitor_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_srp_monitor_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x8b;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->vbus_minium_voltage = (tmp & 0b00110000) >> 4;
    dst->vbus_valid_detection_enable = AXP_ASSERT_BYTE(tmp, 3);
    dst->vbus_session_detection_enable = AXP_ASSERT_BYTE(tmp, 2);
    dst->discharge_VBUS_enable = AXP_ASSERT_BYTE(tmp, 1);
    dst->charge_VBUS_enable = AXP_ASSERT_BYTE(tmp, 0);

    return rc;
}

esp_err_t axp_pmic_get_over_temperature_poweroff_config(axp_over_temperature_poweroff_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_over_temperature_poweroff_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x8f;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->enable = AXP_ASSERT_BYTE(tmp, 2);

    return rc;
}

esp_err_t axp_pmic_get_GPIO_config(axp_gpio_channel_t ch, axp_gpio_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_gpio_config_t));
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    switch (ch)
    {
    case AXP_GPIO0:
        target_subAddr = 0x90;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        dst->interrupt_raising_wakeup_enable = AXP_ASSERT_BYTE(tmp, 7);
        dst->interrupt_filling_wakeup_enable = AXP_ASSERT_BYTE(tmp, 6);
        tmp = tmp & 0b00000111;
        if (tmp == 0)
            dst->mode = AXP_GPIO_MODE_OUTPUT_LOW;
        else if (tmp == 1)
            dst->mode = AXP_GPIO_MODE_OUTPUT_HIGH;
        else if (tmp == 2)
            dst->mode = AXP_GPIO_MODE_INPUT;
        else if (tmp == 3)
            dst->mode = AXP_GPIO_MODE_LDO;
        else if (tmp == 4)
            dst->mode = AXP_GPIO_MODE_ADC_IN;
        else
            dst->mode = AXP_GPIO_MODE_FLOAT;

        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        dst->gpio_high_voltage = tmp & 0b00000111;

        break; // gpio0
    case AXP_GPIO1:
        target_subAddr = 0x92;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        AXP_ERROR_CHECK(rc);
        dst->interrupt_raising_wakeup_enable = AXP_ASSERT_BYTE(tmp, 7);
        dst->interrupt_filling_wakeup_enable = AXP_ASSERT_BYTE(tmp, 6);
        tmp = tmp & 0b00000111;
        if (tmp == 0)
            dst->mode = AXP_GPIO_MODE_OUTPUT_LOW;
        else if (tmp == 1)
            dst->mode = AXP_GPIO_MODE_OUTPUT_HIGH;
        else if (tmp == 2)
            dst->mode = AXP_GPIO_MODE_INPUT;
        else if (tmp == 3)
            dst->mode = AXP_GPIO_MODE_LDO;
        else if (tmp == 4)
            dst->mode = AXP_GPIO_MODE_ADC_IN;
        else
            dst->mode = AXP_GPIO_MODE_FLOAT;

        target_subAddr = 0x91;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        dst->gpio_high_voltage = tmp & 0b00000111;
        break;

    case AXP_GPIO2:
        target_subAddr = 0x93;
        rc = axp_iic_readReg(target_subAddr, &tmp);
        dst->interrupt_raising_wakeup_enable = AXP_ASSERT_BYTE(tmp, 7);
        dst->interrupt_filling_wakeup_enable = AXP_ASSERT_BYTE(tmp, 6);
        tmp = tmp & 0b00000111;
        if (tmp == 0)
            dst->mode = AXP_GPIO_MODE_OUTPUT_LOW;
        else if (tmp == 1)
            dst->mode = AXP_GPIO_MODE_FLOAT;
        else if (tmp == 2)
            dst->mode = AXP_GPIO_MODE_INPUT;
        else
            dst->mode = AXP_GPIO_MODE_FLOAT;
        break;

    case AXP_GPIO3:
        target_subAddr = 0x95;
        rc = axp_iic_readReg(target_subAddr, &tmp);

        dst->interrupt_raising_wakeup_enable = AXP_ASSERT_BYTE(tmp, 7);
        dst->interrupt_filling_wakeup_enable = AXP_ASSERT_BYTE(tmp, 6);
        tmp = tmp & 0b00000110;
        tmp = tmp >> 1;
        if (tmp == 0)
            dst->mode = AXP_GPIO_MODE_OUTPUT_LOW;
        else if (tmp == 1)
            dst->mode = AXP_GPIO_MODE_FLOAT;
        else
            dst->mode = AXP_GPIO_MODE_INPUT;
        break;

    default:
        ESP_LOGE(TAG, "no this gpio");
        break;
    }

    return rc;
}

esp_err_t axp_pmic_get_interrupt_config(axp_irq_config_t *dst)
{
    memset(dst, 0x00, sizeof(axp_irq_config_t));
    uint8_t tmp[5] = {0x00};
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0x40;
    rc = axp_iic_readRegs(target_subAddr, 5, tmp);

    for (uint8_t i = 0; i < 5; i++)
    {
        dst->regmap |= (tmp[i] << (8 * i));
    }

    return rc;
}

esp_err_t axp_pmic_get_coulomb_counter_config(axp_coulomb_counter_config_t *dst)
{
    uint8_t tmp = 0x00;
    uint8_t target_subAddr;
    esp_err_t rc = ESP_OK;
    target_subAddr = 0xb8;
    rc = axp_iic_readReg(target_subAddr, &tmp);
    AXP_ERROR_CHECK(rc);

    dst->enable = AXP_ASSERT_BYTE(tmp, 7);
    dst->coulomb_counter_pause = AXP_ASSERT_BYTE(tmp, 6);
    dst->reset = AXP_ASSERT_BYTE(tmp, 5);

    target_subAddr = 0xb9;
    rc = axp_iic_readReg(target_subAddr, &tmp);

    dst->electricity_counter_pause = AXP_ASSERT_BYTE(tmp, 7);

    return rc;
}
/*end of file*/