// 硬件抽象层：GPIO
// HAL: GPIO
//
// 替代 Arduino 的 pinMode() / digitalWrite() / digitalRead()。

#pragma once

#include <stdint.h>

enum HalPinMode {
	HAL_PIN_OUTPUT,
	HAL_PIN_INPUT,
	HAL_PIN_INPUT_PULLUP,
	HAL_PIN_INPUT_PULLDOWN,
};

void hal_gpio_init(int pin, HalPinMode mode);
void hal_gpio_write(int pin, int level);
int  hal_gpio_read(int pin);

// 电机引脚在初始化阶段需要先拉低防止误转，等价于 ledcDetach() + pinMode(OUTPUT) + digitalWrite(LOW)
void hal_gpio_release_pwm(int pin);
