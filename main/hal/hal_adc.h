// 硬件抽象层：电池电压 ADC
// HAL: battery voltage ADC
//
// 替代 Arduino 的 analogReadMilliVolts()。
//
// ★ 精度关键：analogReadMilliVolts() 内部使用 eFuse 校准数据做曲线拟合，
//   直接读原始 ADC 值会有明显偏差（S3 上可达 ±5%），导致低电保护阈值失准。
//   因此这里同样走 adc_cali curve_fitting 方案。
//
// S3 的 ADC1 通道映射：GPIO1 -> ADC1_CH0 … GPIO10 -> ADC1_CH9

#pragma once

#include <stdint.h>

// 初始化电池 ADC。gpio 为板级宏 BOARD_VBAT_ADC_PIN。
void hal_battery_adc_init(int gpio);

// 读取电池电压（V），已按分压比换算。等价于原 readBatteryVoltage() 的返回值部分。
float hal_battery_read_voltage(int samples, float divider);
