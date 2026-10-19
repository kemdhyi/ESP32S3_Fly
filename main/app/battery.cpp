// 电池电压检测
//
// 原 Arduino 版依赖 analogReadMilliVolts()（内部含 eFuse 校准）。
// IDF 版改由 hal_battery_read_voltage() 完成，校准方案保持一致（曲线拟合）。

#include "app/battery.h"

#include "app/globals.h"
#include "hal/board.h"
#include "hal/hal_adc.h"

#define VBAT_ADC_PIN       BOARD_VBAT_ADC_PIN      // 电池电压采样引脚
#define VBAT_ADC_SAMPLES   BOARD_VBAT_ADC_SAMPLES  // ADC 采样次数
#define VBAT_DIVIDER       (43.0f / 33.0f)         // 分压比：上桥 10kΩ + 下桥 33kΩ

float batteryVoltage = 0.0f;  // 全局最新电压，由 updateBatteryVoltage() 定期刷新

void updateBatteryVoltage() {
	static float lastCheck = 0;
	if (t - lastCheck < 0.5f) return;  // 每 0.5 秒采样一次
	lastCheck = t;
	batteryVoltage = readBatteryVoltage();
}

float readBatteryVoltage() {
	return hal_battery_read_voltage(VBAT_ADC_SAMPLES, VBAT_DIVIDER);
}
