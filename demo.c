/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

//for init only
#include "bsp_iic.h"

#include "axp20x.h"
#include "driver/gpio.h"

static const char *TAG = "app_main";

#define AXP_INT_PIN GPIO_NUM_6

#define DEBUG_MEM() ESP_LOGI("mem", "all : %fk , internal %fk , minimum %fk  all-internal %fk", esp_get_free_heap_size() / 1000.0, esp_get_free_internal_heap_size() / 1000.0, esp_get_minimum_free_heap_size() / 1000.0, (esp_get_free_heap_size() - esp_get_free_internal_heap_size()) / 1000.0);

SemaphoreHandle_t xMutexPowerIICFree;

#define AXP_IIC_DEV I2C_NUM_0

static void pmic_init()
{
    // power

    gpio_reset_pin(AXP_INT_PIN);
    gpio_set_direction(AXP_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(AXP_INT_PIN, GPIO_PULLUP_ONLY);

    // pmic init
    axp_adc_config_t ac = AXP_ADC_CONF_DEFAULT;
    ESP_ERROR_CHECK(axp_pmic_set_adc_config(ac));
    axp_charge_config_t chg = AXP_CHARGE_CONF_DEFAULT;
    ESP_ERROR_CHECK(axp_pmic_set_charge_config(chg));
    axp_irq_config_t irq = AXP_IRQ_CONF_DEFAULT;
    ESP_ERROR_CHECK(axp_pmic_set_interrupt_config(irq));
    axp_PEK_key_config_t pek = AXP_PEK_CONF_DEFAULT;
    ESP_ERROR_CHECK(axp_pmic_set_PEK_key_config(pek));
    axp_IPSOUT_warning_level_config_t ipsw = AXP_IPS_WARNING_CONF_DEFAULT;
    axp_pmic_set_IPSOUT_warning_level_config(ipsw);

    axp_power_out_config_t powerOut;
    powerOut.enable = 0;     // disable
    powerOut.mode = 0;       // ldo
    powerOut.voltage = 3300; // 3v3
    powerOut.vrc_enable = 0; // disable
    powerOut.vrc_config = 0; // 1.6mv/s
    // power down adc and pa
    axp_pmic_set_power_output(AXP_LDO2, powerOut);
    axp_pmic_set_power_output(AXP_LDO4, powerOut);
    // off not use channel
    axp_pmic_set_power_output(AXP_DCDC2, powerOut);
    axp_pmic_set_power_output(AXP_LDO3, powerOut);

    vTaskDelay(10);

    powerOut.enable = 1; // enable

    axp_pmic_set_power_output(AXP_LDO2, powerOut);
    axp_pmic_set_power_output(AXP_LDO4, powerOut);

    vTaskDelay(10);
}

void vTaskPower(void *args)
{
    /*AXP IIC operation only in this task*/
    axp_irq_status_t irq;
    axp_counter_resault_t cnt;
    axp_adc_resault_t adc;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {

        axp_pmic_get_counter_resault(&cnt);
        axp_pmic_get_adc_resault(&adc);

        if (gpio_get_level(AXP_INT_PIN) == 0)
        {
            axp_pmic_irq_handle(&irq); // clear interrupt flag

            ESP_LOGI(TAG, "AXP IRQ : %llx", irq.regmap);
            // long press
            if (irq.bit_status.PEK_long_press)
            {
                // ESP_LOGI(TAG, "long press");
            }
            // short press
            if (irq.bit_status.PEK_short_press)
            {
                // ESP_LOGI(TAG, "short press");
                DEBUG_MEM();
            }
        } // gpio_get_level
        vTaskDelayUntil(&xLastWakeTime, 10);

    } // for
}

void app_main(void)
{
    //provide by bsp_iic
    iic_device_master_init(AXP_IIC_DEV, 15, 7, 100 * 1000); // sda , scl , freq

    
    xMutexPowerIICFree = xSemaphoreCreateMutex();
    xSemaphoreGive(xMutexPowerIICFree);

    axp_pmic_set_iic_operation_prot_mutex(&xMutexPowerIICFree);
    axp_pmic_set_iic_device(AXP_IIC_DEV);

    pmic_init();

    xTaskCreatePinnedToCore(vTaskPower, "axp", 8192, NULL, 2, NULL, 0);
    for (;;)
    {
        vTaskDelay(1);
    }
}
