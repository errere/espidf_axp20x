#ifndef __BSP_AXP203_H__
#define __BSP_AXP203_H__

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_system.h"
#include "bsp_iic.h"
#include <string.h>

// only use for axp203/axp209

/*
struct
{
  unsigned int a : 1;//lsb(b0)
  unsigned int b : 1;
  unsigned int c : 1;
  unsigned int d : 1;
  unsigned int e : 1;
  unsigned int f : 1;
  unsigned int g : 1;
  unsigned int h : 1;//msb(b7)
} status;
*/

/*===========================================================AXP LIB=====================================================*/

#define AXP_IIC_DEV I2C_NUM_0

#define AXP_ADDRESS 0x34

#define AXP_ENABLE 0x1
#define AXP_DISABLE 0x0

//#define AXP_CHECK_CONN

#define AXP_CHARGE_CONF_DEFAULT             \
    {                                       \
        .enable = 1,                        \
        .target_voltage = 2,                \
        .end_current = 0,                   \
        .current = 500,                     \
        .pre_charge_tomeout = 0,            \
        .CHGLED_flash_config = 1,           \
        .CC_mode_timeout = 0,               \
        .backup_battery_enable = 1,         \
        .backup_battery_target_voltage = 1, \
        .backup_battery_current = 0,        \
        .VLTF_charge = 2112,                \
        .VHTF_charge = 397,                 \
        .VLTF_discharge = 3226,             \
        .VHTF_discharge = 282               \
    }

#define AXP_ADC_CONF_DEFAULT                           \
    {                                                  \
        .batt_voltage_enable = 1,                      \
        .batt_current_enable = 1,                      \
        .acin_voltage_enable = 1,                        \
        .acin_current_enable = 1,                        \
        .vbus_voltage_enable = 0,                      \
        .vbus_current_enable = 0,                      \
        .aps_voltage_enable = 1,                       \
        .ts_pin_enable = 0,                            \
        .internal_temperature_detection_enable = 1,    \
        .GPIO1_analog_in_enable = 0,                   \
        .GPIO0_analog_in_enable = 0,                   \
        .sample_rate = 1,                              \
        .gpio1_adc_range = 0,                          \
        .gpio0_adc_range = 0,                          \
        .gpio1_adc_interrupt_raising_Threshold = 0xff, \
        .gpio1_adc_interrupt_falling_Threshold = 0x00  \
    }

#define AXP_IRQ_CONF_DEFAULT                               \
    {                                                      \
        .bit_config.ACIN_over_voltage_enable = 1,          \
        .bit_config.ACIN_connect_enable = 0,               \
        .bit_config.ACIN_disconnect_enable = 0,            \
        .bit_config.VBUS_over_voltage = 0,                 \
        .bit_config.VBUS_connect_enable = 0,               \
        .bit_config.VBUS_disconnect_enable = 0,            \
        .bit_config.VBUS_availiable_below_hold_enable = 0, \
        .bit_config.battery_connect_enable = 0,            \
        .bit_config.battery_disconnect_enable = 0,         \
        .bit_config.battery_activation_enable = 1,         \
        .bit_config.exit_battery_activation_enable = 0,    \
        .bit_config.charging_int_enable = 0,               \
        .bit_config.charge_finish_enable = 0,              \
        .bit_config.battery_over_temperature_enable = 0,   \
        .bit_config.battery_low_temperature_enable = 0,    \
        .bit_config.axp_over_temperature_enable = 0,       \
        .bit_config.charging_current_low_enable = 0,       \
        .bit_config.DCDC2_output_low_enable = 0,           \
        .bit_config.DCDC3_output_low_enable = 0,           \
        .bit_config.LDO3_output_low_enable = 0,            \
        .bit_config.PEK_short_press_enable = 1,            \
        .bit_config.PEK_long_press_enable = 1,             \
        .bit_config.NOE_poweron_enable = 0,                \
        .bit_config.NOE_poweroff_enable = 0,               \
        .bit_config.VBUS_valid_enable = 0,                 \
        .bit_config.VBUS_invalid_enable = 0,               \
        .bit_config.VBUS_session_AB_enable = 0,            \
        .bit_config.VBUS_session_end_enable = 0,           \
        .bit_config.APS_warning_level1_enable = 0,         \
        .bit_config.APS_warning_level2_enable = 1,         \
        .bit_config.timeout_enable = 0,                    \
        .bit_config.PEK_raising_enable = 0,                \
        .bit_config.PEK_filling_enable = 0,                \
        .bit_config.GPIO3_input_edge_enable = 0,           \
        .bit_config.GPIO2_input_edge_enable = 0,           \
        .bit_config.GPIO1_input_enable = 0,                \
        .bit_config.GPIO0_input_edge_enable = 0            \
    }

#define AXP_PEK_CONF_DEFAULT         \
    {                                \
        .power_on_press_time = 2,    \
        .long_press_time = 0,        \
        .long_press_to_shutdown = 1, \
        .power_ok_singal_delay = 1,  \
        .power_off_delay = 3         \
    }
// types

/*====================READ ONLY====================*/
// REG 00H:输入电源状态
typedef struct
{
    uint8_t ACIN_exist;         // ACIN 存在指示 0:ACIN 不存在；1:ACIN 存在
    uint8_t ACIN_availiable;    //指示 ACIN 是否可用
    uint8_t VBUS_exist;         // VBUS 存在指示 0:VBUS 不存在；1:VBUS 存在
    uint8_t VBUS_availiable;    //指示 VBUS 是否可用
    uint8_t VBUS_above_VHOLD;   //指示 VBUS 接入在使用之前是否大于 VHOLD
    uint8_t current_direction;  //指示电池电流方向 0:电池在放电；1:电池被充电
    uint8_t ACIN_short_VBUS;    //指示 ACIN 和 VBUS 输入是否在 PCB 被短接
    uint8_t boot_power_sources; //指示启动源是否为 ACIN 或 VBUS 0:启动源非 ACIN/VBUS； 1:启动源为 ACIN/VBUS
} axp_input_status_t;

// REG 01H:电源工作模式以及充电状态指示
typedef struct
{
    uint8_t over_temperature;                //指示 AXP20x 是否过温 0:未过温； 1:过温
    uint8_t is_charging;                     //充电指示 0:未充电或充电已完成； 1:正在充电
    uint8_t battery_exists;                  //电池存在状态指示 0:无电池连接到 AXP20x； 1:电池已经连接到 AXP20x
    uint8_t battery_active;                  //指示电池是否进入激活模式 0:未进入电池激活模式； 1:已进入电池激活模式
    uint8_t charging_current_below_expected; //指示充电电流是否小于期望电流 0:实际充电电流等于期望电流； 1:实际充电电流小于期望电流
} axp_power_status_t;

// REG 02H:USB OTG VBUS 状态指示
typedef struct
{
    uint8_t VBUS_valid;             //指示 VBUS 是否有效，1 表示有效
    uint8_t VBUS_Session_A_B_valid; //指示 VBUS Session A/B 是否有效，1 表示有效
    uint8_t Session_End;            //指示 Session End 状态，1 表示有效
} axp_usb_otg_vbus_status_t;

// REG 56H-7FH ADC 数据(page29 on axp209)
//注：电池供电功率计算方法为
// Pbat = 2 * 寄存器值 * 电压 LSB * 电流 LSB / 1000。
//其中，电压 LSB 为 1.1mV，电流 LSB 为 0.5mA，计算结果单位为 mW。
typedef struct
{
    uint16_t ACIN_voltage;
    uint16_t ACIN_current;
    uint16_t VBUS_voltage;
    uint16_t VBUS_current;
    uint16_t internal_temperature;
    uint16_t TS;
    uint16_t GPIO0;
    uint16_t GPIO1;
    uint32_t battery_Instantaneous_power; // 24bit
    uint16_t battery_voltage;
    uint16_t battery_charging_current;
    uint16_t battery_discharging_current;
    uint16_t IPS_voltage;
} axp_adc_resault_t;

// REG 94H:GPIO[2:0]信号状态设置及监测
//<for a function>

// REG B9H:电量计量结果
// REG B0H-B7H 电池充/池放电库仑计数据寄存器(page30 on axp209)
//库仑计算方法：C= 65536 * 电流 LSB *（充电库仑计值-放电库仑计值） / 3600 / ADC 采样率。
//其中：ADC 采样率参考 REG84H 的设置；电流 LSB 为 0.5mA；计算结果单位为 mAh。
typedef struct
{
    uint32_t coulomb_count_for_charge;    //电池充电库仑计数据
    uint32_t coulomb_count_for_discharge; //电池放电库仑计数据
    uint8_t electricity;                  //计量结果，百分比
} axp_counter_resault_t;

/*====================READ AND WRITE====================*/

// REG 04-0FH:数据缓存
typedef struct
{
    uint8_t data_flash[12]; //只要外部电源、电池或备用电池某一路电源存在，此数据就会一直保存，不受开关机影响。
} axp_data_flash_t;

// REG 12H:电源输出控制
// REG 25H:DC-DC2/LDO3 动态电压调节参数设置
// REG 27H:DC-DC3 输出电压设置
// REG 28H:LDO2/4 输出电压设置
// REG 29H:LDO3 输出电压设置
// REG 91H: LDO5 输出电压以及 EXTEN/GPIO 输出高电平设置
// dcdc2
// REG 12H,23H,25H
// Vout=[0.7+(Bit5-0)*0.025]V

// ldo3
// REG 12H,25H,29H
// Vout=[0.7+(Bit6-0)*0.025]V

// ldo4
// REG 12H,28H
// Vout=index

// ldo2
// REG 12H,28H
// Vout=[1.8+(Bit7-4)*0.1]V

// dcdc3
// REG 12H,27H
// Vout=[0.7+(Bit6-0)*0.025]V

// ldo5
// REG 90H 91H
// Vout=[1.8 +( Bit7-4）*0.1]V； defalt=1.8+10*0.1=2.8V
typedef struct
{
    uint8_t enable;     //  开关控制 0:关闭； 1:打开
    uint8_t mode;       //模式选择 0：LDO 模式，电压由[6:0]设置  1：开关模式，电压由 LDO3IN 决定(LDO3)
    uint16_t voltage;   // 输出电压设置(mv)
    uint8_t vrc_enable; // VRC 使能控制 0:打开； 1:关闭
    uint8_t vrc_config; //  VRC 电压上升斜率控制 0: 25mV/15.625us=1.6mV/us , 1: 25mV/31.250us=0.8mV/us
} axp_power_out_config_t;

typedef enum
{
    AXP_DCDC2,
    AXP_DCDC3,
    AXP_LDO2,
    AXP_LDO3,
    AXP_LDO4,
    AXP_LDO5,
    AXP_PO_MAX
} axp_power_output_channel_t;

// REG 30H:VBUS-IPSOUT 通路管理
typedef struct
{
    uint8_t from;                        // VBUS 可用时 VBUS-IPSOUT 通路选择控制信号 0:由 N_VBUSEN pin 决定是否打开此通路 1:VBUS-IPSOUT 通路可以被选择打开，不管 N_VBUSEN 的状态
    uint8_t VBUS_VHOLD_enable;           // VBUS VHOLD 限压控制 0:不限压； 1:限压
    uint16_t VHOLD_voltage;              // VHOLD 设置 VHOLD= [4.0+(Bit5-3)*0.1] (4V~4.7V)
    uint8_t VBUS_current_imit_selection; // VBUS 限流控制打开时限流选择(0:900mA ; 1:500mA; 2:100mA; 3:not limit)
} axp_VBUS_IPSOUT_channel_config_t;

// REG 31H:VOFF 关机电压设置
typedef struct
{
    uint8_t PEK_GPIO_edge_weakUp_enable; // Sleep 模式下 PEK 或 GPIO 边沿唤醒功能使能设置： 0：关闭 1：打开 此 bit 写完后自动清 0，因此每次进 Sleep 模式前需再次写 1
    uint16_t VOFF_set_voltage;           // VOFF 设置 VOFF=[2.6+(Bit2-0)*0.1]V Default： 2.9V(2.6v~3.3v)
} axp_VOFF_config_t;

// REG 32H:关机设置、电池检测以及 CHGLED 管脚控制
typedef struct
{
    uint8_t shutdown;             //关机控制 此位写 1 会关闭 AXP209 的输出
    uint8_t power_monitor_enable; //电池监测功能设置位: 0:关闭； 1:打开
    uint8_t CHGLED_config;        // CHGLED 管脚功能设置(p34),CHGLED 管脚控制设置(0: 高阻 1: 25% 1Hz 闪烁 2: 25% 4Hz 闪烁 3: 输出低电平)
    uint8_t CHGLED_auto;          // CHGLED 管脚控制设置  0: 由充电功能控制  1: 由寄存器 REG 32H[5:4]控制
    uint8_t shutdown_time_config; //输出关闭时序控制 0: 同时关闭  1: 与启动时序相反
    uint8_t shutdown_delay;       // N_OE 由低变高后 AXP209 关机延迟时间(0: 128mS； 1: 1S；  2: 2S； 3: 3S )
} axp_power_off_config_t;

// REG 33H:充电控制 1
// REG 34H:充电控制 2
// REG 35H:备用电池充电控制
// REG 38H:VLTF-charge电池充电低温门限设置
// REG 39H:VHTF-charge电池充电高温门限设置
// REG 3CH:VLTF-discharge 电池放电低温门限设置
// REG 3DH:VHTF-discharge电池放电高温门限设置
typedef struct
{
    uint8_t enable;                        //充电功能使能控制位  0:关闭， 1:打开
    uint8_t target_voltage;                //充电目标电压设置  0:4.1V； 1:4.15V； 2:4.2V； 3:4.36V
    uint8_t end_current;                   //充电结束电流设置 0:充电电流小于 10%设置值时结束充电 1:充电电流小于 15%设置值时结束充电
    uint16_t current;                      //充电电流设置 Icharge= [300+(Bit3-0)*100]mA  (300ma ~ 1800ma)
    uint8_t pre_charge_tomeout;            //预充电超时设置  0: 40 min； 1: 50min； 2: 60min； 3: 70min
    uint8_t CHGLED_flash_config;           // CHGLED 模式选择  0:充电时常亮 1:充电时闪烁
    uint8_t CC_mode_timeout;               //恒流模式下超时设置 Bit1-0 0: 6Hours； 1: 8Hours； 2: 10Hours； 3: 12Hours
    uint8_t backup_battery_enable;         //备用电池充电使能控制  0:关闭；1:打开
    uint8_t backup_battery_target_voltage; //备用电池充电目标电压设置 0:3.1V；1:3.0V；2:3.6V；3:2.5V
    uint16_t backup_battery_current;       //备用电池充电电流设置 0: 50uA；1: 100uA；2: 200uA； 3:400uA
    float VLTF_charge;                     // M*10H，当 M=A5H 时对应 2.112V；可对应电压 0V~3.264V VLTF-charge = M *10H * 0.0008V (mV)
    float VHTF_charge;                     // N*10H，当 N=1FH，对应 0.397V；可对应电压 0V~3.264V  VHTF-charge = N *10H * 0.0008V (mV)
    float VLTF_discharge;                  // M*10H，当 M=FCH 时对应 3.226V；可对应电压 0V~3.264V VLTF-discharge = M *10H * 0.0008V (mV)
    float VHTF_discharge;                  // N*10H，当 N=16H，对应 0.282V；可对应电压 0V~3.264V VLTF-discharge = N *10H * 0.0008V (mV)
} axp_charge_config_t;

// 36H:PEK 按键参数设置
typedef struct
{
    uint8_t power_on_press_time;    //开机时间设置 0: 128mS； 1: 3S； 2: 1S； 3: 2S.
    uint8_t long_press_time;        //长按键时间设置 0: 1S； 1: 1.5S；2: 2S； 3: 2.5S.
    uint8_t long_press_to_shutdown; //按键时长大于关机时长时自动关机功能设置 0:关闭； 1:打开
    uint8_t power_ok_singal_delay;  //电源启动完成后 PWROK 信号延时 0:8mS； 1:64mS
    uint8_t power_off_delay;        //关机时长设置 0: 4S； 1: 6S； 2: 8S； 3: 10S.
} axp_PEK_key_config_t;

// REG 37H:DC-DC 工作频率设置
// REG 80H:DC-DC 工作模式选择
typedef struct
{
    uint16_t DCDC_freq; //每一级改变 5%，默认值 1.5MHz F=[1+/- (Bit3-0)*5%)]*1.5MHz
    uint8_t DCDC2;      // DC-DC2 工作模式控制  0:PFM/PWM 自动切换  1:固定 PWM
    uint8_t DCDC3;      // DC-DC3 工作模式控制  0:PFM/PWM 自动切换  1:固定 PWM
} axp_DCDC_physics_config_t;

// REG 3AH: 系统 IPSOUT Vwarning Level1
// REG 3BH: 系统 IPSOUT Vwarning Level2
// REG3AH、REG3BH对应的电压设置为如下关系(假设寄存器值为n): Vwarning = 2.8672 + 1.4mV * n * 4
typedef struct
{
    float level1; //(假设寄存器值为n): Vwarning = 2.8672 + 1.4mV * n * 4   (mv)
    float level2; //(假设寄存器值为n): Vwarning = 2.8672 + 1.4mV * n * 4   (mv)
} axp_IPSOUT_warning_level_config_t;

// REG 82H:ADC 使能 1
// REG 83H:ADC 使能 2
// REG 84H:ADC 采样速率设置，TS 管脚控制
// REG 85H:ADC 输入范围
// REG 86H:GPIO1 ADC IRQ 上升沿门限设置
// REG 87H:GPIO1 ADC IRQ 下降沿门限设置
typedef struct
{
    uint8_t batt_voltage_enable;                   //电池电压 ADC 使能
    uint8_t batt_current_enable;                   //电池电流 ADC 使能
    uint8_t acin_voltage_enable;                   // ACIN 电压 ADC 使能
    uint8_t acin_current_enable;                   // ACIN 电流 ADC 使能
    uint8_t vbus_voltage_enable;                   // VBUS 电压 ADC 使能
    uint8_t vbus_current_enable;                   // VBUS 电流 ADC 使能
    uint8_t aps_voltage_enable;                    // APS 电压 ADC 使能
    uint8_t ts_pin_enable;                         // TS 管脚 ADC 功能使能
    uint8_t internal_temperature_detection_enable; // AXP209 内部温度监测 ADC 使能 0:关闭， 1:打开
    uint8_t GPIO1_analog_in_enable;                // GPIO0 ADC 功能使能
    uint8_t GPIO0_analog_in_enable;                // GPIO1 ADC 功能使能
    uint8_t sample_rate;                           // ADC 采样速率设置 25×2^n 采样率分别为 0:25， 1:50， 2:100， 3:200Hz
    uint8_t gpio1_adc_range;                       // GPIO1 ADC 输入范围 0:0-2.0475V 1:0.7-2.7475V
    uint8_t gpio0_adc_range;                       // GPIO0 ADC 输入范围 0:0-2.0475V 1:0.7-2.7475V
    uint8_t gpio1_adc_interrupt_raising_Threshold; // GPIO1 ADC IRQ 上升沿门限设置  一个 LSB 为 8mV
    uint8_t gpio1_adc_interrupt_falling_Threshold; // GPIO1 ADC IRQ 下降沿门限设置  一个 LSB 为 8mV

} axp_adc_config_t;
typedef struct
{
    uint8_t ts_output_current; // TS 管脚输出电流设置: 0:20uA； 1:40uA； 2:60uA； 3:80uA
    uint8_t ts_mode;           // TS 管脚功能选择 0:电池温度监测功能，1:外部独立的 ADC 输入通路
    uint8_t ts_output_method;  // TS 管脚电流输出方式设置 0:关闭 1:充电时输出电流 2:ADC 采样时输入，可以省电 3:一直打开
} axp_ts_pin_config_t;

// REG 8AH:定时器控制
typedef struct
{
    uint8_t clear;       //定时器超时 写 1 清除此状态
    uint8_t set_timeout; //设置定时时间，单位为分 写全 0 则关闭此定时器
} axp_timer_config_t;

// REG 8BH:VBUS 管脚监测 SRP 功能控制
typedef struct
{
    uint8_t vbus_minium_voltage;           // VBUS 有效电压设置 0:4.0V； 1:4.15V； 2:4.45V； 3:4.55V
    uint8_t vbus_valid_detection_enable;   // VBUS Valid 检测功能设置:0:关闭，1:打开
    uint8_t vbus_session_detection_enable; // VBUS Session 检测功能设置:0:关闭，1:打开
    uint8_t discharge_VBUS_enable;         // Discharge VBUS 放电功能设置 0:关闭 VBUS 的放电电阻；1:使用 VBUS 的放电电阻
    uint8_t charge_VBUS_enable;            // Charge VBUS 充电功能设置 0:断开 VBUS 充电电阻；1:使用 VBUS 充电电阻给 VBUS 充电
} axp_srp_monitor_config_t;

// REG 8FH:过温关机等功能设置
typedef struct
{
    uint8_t enable; // AXP209 内部过温关机功能设置 0:不关机； 1:关机
} axp_over_temperature_poweroff_config_t;

// REG 90H:GPIO0 功能设置
// REG 92H:GPIO1 功能设置
// REG 93H:GPIO2 功能设置
// REG 95H:GPIO3 设置
typedef enum
{
    AXP_GPIO_MODE_OUTPUT_LOW = 0,  //输出低
    AXP_GPIO_MODE_OUTPUT_HIGH = 1, //输出高（3.3V）
    AXP_GPIO_MODE_INPUT = 2,       //通用输入功能
    AXP_GPIO_MODE_LDO = 3,         //低噪声 LDO5
    AXP_GPIO_MODE_ADC_IN = 4,      // ADC 输入
    AXP_GPIO_MODE_FLOAT = 5,       //浮空
} axp_gpio_mode_t;

typedef enum
{
    AXP_GPIO0,
    AXP_GPIO1,
    AXP_GPIO2,
    AXP_GPIO3,
    AXP_GPIO_MAX
} axp_gpio_channel_t;

typedef struct
{
    uint8_t interrupt_raising_wakeup_enable; //上升沿 IRQ 或 Wakeup 功能
    uint8_t interrupt_filling_wakeup_enable; // GPIO2 下降沿 IRQ 或 Wakeup 功能
    axp_gpio_mode_t mode;                    // 管脚功能设置
    uint8_t gpio_high_voltage;               // EXTEN 以及 GPIO[1:0]输出高电平设置 0:1.8V;  1:2.5V;  2:2.8V;  3:3.0V;  4:3.1V;  5:3.3V;  6:3.4V;  7:3.5V
} axp_gpio_config_t;

// REG 40H 及 48H:IRQ 使能 1 及 IRQ 状态 1
// REG 41H 及 49H:IRQ 使能 2 及 IRQ 状态 2
// REG 42H 及 4AH:IRQ 使能 3 及 IRQ 状态 3
// REG 43H 及 4BH: IRQ 使能 4 及 IRQ 状态 4
// REG 44H 及 4C:IRQ 使能 5 及 IRQ 状态 5
// REG 48H-4CH  IRQ 状态寄存器 1-5(page29&43 on axp209)
//注:所有 IRQ 状态寄存器对应位写 1 将清除相应状态。
typedef union
{
    struct
    {
        // lsb
        // reg40 b0..7
        uint8_t : 1;
        uint8_t VBUS_availiable_below_hold_enable : 1;
        uint8_t VBUS_disconnect_enable : 1;
        uint8_t VBUS_connect_enable : 1;
        uint8_t VBUS_over_voltage : 1;
        uint8_t ACIN_disconnect_enable : 1;
        uint8_t ACIN_connect_enable : 1;
        uint8_t ACIN_over_voltage_enable : 1;
        // reg41 b0..7
        uint8_t battery_low_temperature_enable : 1;
        uint8_t battery_over_temperature_enable : 1;
        uint8_t charge_finish_enable : 1;
        uint8_t charging_int_enable : 1;
        uint8_t exit_battery_activation_enable : 1;
        uint8_t battery_activation_enable : 1;
        uint8_t battery_disconnect_enable : 1;
        uint8_t battery_connect_enable : 1;
        // reg42 b0..7
        uint8_t PEK_long_press_enable : 1;
        uint8_t PEK_short_press_enable : 1;
        uint8_t LDO3_output_low_enable : 1;
        uint8_t DCDC3_output_low_enable : 1;
        uint8_t DCDC2_output_low_enable : 1;
        uint8_t : 1;
        uint8_t charging_current_low_enable : 1;
        uint8_t axp_over_temperature_enable : 1;
        // reg43 b0..7
        uint8_t APS_warning_level2_enable : 1;
        uint8_t APS_warning_level1_enable : 1;
        uint8_t VBUS_session_end_enable : 1;
        uint8_t VBUS_session_AB_enable : 1;
        uint8_t VBUS_invalid_enable : 1;
        uint8_t VBUS_valid_enable : 1;
        uint8_t NOE_poweroff_enable : 1;
        uint8_t NOE_poweron_enable : 1;
        // reg44 b0..7
        uint8_t GPIO0_input_edge_enable : 1;
        uint8_t GPIO1_input_enable : 1;
        uint8_t GPIO2_input_edge_enable : 1;
        uint8_t GPIO3_input_edge_enable : 1;
        uint8_t : 1;
        uint8_t PEK_filling_enable : 1;
        uint8_t PEK_raising_enable : 1;
        uint8_t timeout_enable : 1;
        // msb
    } bit_config;

    uint64_t regmap; // msb 00 00 00 44 43 42 41 40 lsb
} axp_irq_config_t;
typedef union
{

    struct
    {
        // lsb
        // reg40 b0..7
        uint8_t : 1;
        uint8_t VBUS_availiable_below_hold : 1;
        uint8_t VBUS_disconnect : 1;
        uint8_t VBUS_connect : 1;
        uint8_t VBUS_over_voltage : 1;
        uint8_t ACIN_disconnect : 1;
        uint8_t ACIN_connect : 1;
        uint8_t ACIN_over_voltage : 1;
        // reg41 b0..7
        uint8_t battery_low_temperature : 1;
        uint8_t battery_over_temperature : 1;
        uint8_t charge_finish : 1;
        uint8_t charging_int : 1;
        uint8_t exit_battery_activation : 1;
        uint8_t battery_activation : 1;
        uint8_t battery_disconnect : 1;
        uint8_t battery_connect : 1;
        // reg42 b0..7
        uint8_t PEK_long_press : 1;
        uint8_t PEK_short_press : 1;
        uint8_t LDO3_output_low : 1;
        uint8_t DCDC3_output_low : 1;
        uint8_t DCDC2_output_low : 1;
        uint8_t : 1;
        uint8_t charging_current_low : 1;
        uint8_t axp_over_temperature : 1;
        // reg43 b0..7
        uint8_t APS_warning_level2 : 1;
        uint8_t APS_warning_level1 : 1;
        uint8_t VBUS_session_end : 1;
        uint8_t VBUS_session_AB : 1;
        uint8_t VBUS_invalid : 1;
        uint8_t VBUS_valid : 1;
        uint8_t NOE_poweroff : 1;
        uint8_t NOE_poweron : 1;
        // reg44 b0..7
        uint8_t GPIO0_input_edge : 1;
        uint8_t GPIO1_input : 1;
        uint8_t GPIO2_input_edge : 1;
        uint8_t GPIO3_input_edge : 1;
        uint8_t : 1;
        uint8_t PEK_filling : 1;
        uint8_t PEK_raising : 1;
        uint8_t timeout : 1;
        // msb
    } bit_status;

    uint64_t regmap; // msb 00 00 00 44 43 42 41 40 lsb
} axp_irq_status_t;

// REG B8H:库仑计控制
// REG B9H:电量计量结果
typedef struct
{
    uint8_t enable;                    //库仑计开关控制
    uint8_t coulomb_counter_pause;     //库仑计暂停控制，此位写 1 将暂停库仑计数，同时此位将自清零
    uint8_t reset;                     //清除库仑计控制，此位写 1 会将库仑计清零，同时此位将自清零
    uint8_t electricity_counter_pause; //计量系统控制 0：正常工作模式 1：暂停工作
} axp_coulomb_counter_config_t;

//<maybe>
// 16 bit sub address?
// REG 300-REG 30F 待解密的数据
// REG 31x 解密后的密码数据

// REG BAH,BBH,C0H-CFH ????(OCV?)(AXP20X_RDC_H/L)
/*
OCV
#define AXP20X_RDC_H			0xba
#define AXP20X_RDC_L			0xbb
#define AXP20X_OCV(m)			(0xc0 + (m))
#define AXP20X_OCV_MAX			0xf
*/
/*====================RT APi====================*/
esp_err_t axp_pmic_poweroff(uint8_t allow_gpio_weakUp);

/*====================read only====================*/
esp_err_t axp_pmic_get_input_status(axp_input_status_t *dst);
esp_err_t axp_pmic_get_power_status(axp_power_status_t *dst);
esp_err_t axp_pmic_get_otg_vbus_status(axp_usb_otg_vbus_status_t *dst);
esp_err_t axp_pmic_get_counter_resault(axp_counter_resault_t *dst);
esp_err_t axp_pmic_get_adc_resault(axp_adc_resault_t *dst);
uint8_t axp_pmic_get_gpio_status(axp_gpio_channel_t ch);
/*====================read====================*/
esp_err_t axp_pmic_read_data_ram(axp_data_flash_t *dst);
/*====================irq====================*/
esp_err_t axp_pmic_irq_handle(axp_irq_status_t *dst);
/*====================write====================*/
esp_err_t axp_pmic_write_data_ram(axp_data_flash_t dat);
esp_err_t axp_pmic_set_power_output(axp_power_output_channel_t ch, axp_power_out_config_t cfg);
esp_err_t axp_pmic_set_VBUS_IPSOUT_channel_config(axp_VBUS_IPSOUT_channel_config_t cfg);
esp_err_t axp_pmic_set_VOFF_config(axp_VOFF_config_t cfg);
esp_err_t axp_pmic_set_power_off_config(axp_power_off_config_t cfg);
esp_err_t axp_pmic_set_charge_config(axp_charge_config_t cfg);
esp_err_t axp_pmic_set_PEK_key_config(axp_PEK_key_config_t cfg);
esp_err_t axp_pmic_set_DCDC_physics_config(axp_DCDC_physics_config_t cfg);
esp_err_t axp_pmic_set_IPSOUT_warning_level_config(axp_IPSOUT_warning_level_config_t cfg);
esp_err_t axp_pmic_set_adc_config(axp_adc_config_t cfg);
esp_err_t axp_pmic_set_ts_pin_config(axp_ts_pin_config_t cfg);
esp_err_t axp_pmic_set_timer_config(axp_timer_config_t cfg);
esp_err_t axp_pmic_set_srp_monitor_config(axp_srp_monitor_config_t cfg);
esp_err_t axp_pmic_set_over_temperature_poweroff_config(axp_over_temperature_poweroff_config_t cfg);
esp_err_t axp_pmic_set_GPIO_config(axp_gpio_channel_t ch, axp_gpio_config_t cfg);
esp_err_t axp_pmic_set_interrupt_config(axp_irq_config_t cfg);
esp_err_t axp_pmic_set_coulomb_counter_config(axp_coulomb_counter_config_t cfg);
/*====================read conf====================*/
esp_err_t axp_pmic_get_power_output(axp_power_output_channel_t ch, axp_power_out_config_t *dst);
esp_err_t axp_pmic_get_VBUS_IPSOUT_channel_config(axp_VBUS_IPSOUT_channel_config_t *dst);
esp_err_t axp_pmic_get_VOFF_config(axp_VOFF_config_t *dst);
esp_err_t axp_pmic_get_power_off_config(axp_power_off_config_t *dst);
esp_err_t axp_pmic_get_charge_config(axp_charge_config_t *dst);
esp_err_t axp_pmic_get_PEK_key_config(axp_PEK_key_config_t *dst);
esp_err_t axp_pmic_get_DCDC_physics_config(axp_DCDC_physics_config_t *dst);
esp_err_t axp_pmic_get_IPSOUT_warning_level_config(axp_IPSOUT_warning_level_config_t *dst);
esp_err_t axp_pmic_get_adc_config(axp_adc_config_t *dst);
esp_err_t axp_pmic_get_ts_pin_config(axp_ts_pin_config_t *dst);
esp_err_t axp_pmic_get_timer_config(axp_timer_config_t *dst);
esp_err_t axp_pmic_get_srp_monitor_config(axp_srp_monitor_config_t *dst);
esp_err_t axp_pmic_get_over_temperature_poweroff_config(axp_over_temperature_poweroff_config_t *dst);
esp_err_t axp_pmic_get_GPIO_config(axp_gpio_channel_t ch, axp_gpio_config_t *dst);
esp_err_t axp_pmic_get_interrupt_config(axp_irq_config_t *dst);
esp_err_t axp_pmic_get_coulomb_counter_config(axp_coulomb_counter_config_t *dst);

#endif