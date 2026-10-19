// 硬件抽象层：GPIO 实现

#include "hal/hal_gpio.h"

#include "driver/gpio.h"
#include "esp_err.h"

void hal_gpio_init(int pin, HalPinMode mode) {
	if (pin < 0) return;

	gpio_config_t cfg = {};
	cfg.pin_bit_mask = (1ULL << pin);
	cfg.intr_type    = GPIO_INTR_DISABLE;

	switch (mode) {
		case HAL_PIN_OUTPUT:
			cfg.mode = GPIO_MODE_OUTPUT;
			break;
		case HAL_PIN_INPUT:
			cfg.mode = GPIO_MODE_INPUT;
			break;
		case HAL_PIN_INPUT_PULLUP:
			cfg.mode = GPIO_MODE_INPUT;
			cfg.pull_up_en = GPIO_PULLUP_ENABLE;
			break;
		case HAL_PIN_INPUT_PULLDOWN:
			cfg.mode = GPIO_MODE_INPUT;
			cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
			break;
	}
	ESP_ERROR_CHECK(gpio_config(&cfg));
}

void hal_gpio_write(int pin, int level) {
	if (pin < 0) return;
	gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
}

int hal_gpio_read(int pin) {
	if (pin < 0) return 0;
	return gpio_get_level((gpio_num_t)pin);
}

void hal_gpio_release_pwm(int pin) {
	// 解除 LEDC 绑定后把引脚拉低。LEDC 的 ledc_stop() 会把引脚复位，
	// 但为了与原 Arduino 版行为一致（先拉低再重配），这里显式配置为输出低。
	if (pin < 0) return;
	gpio_reset_pin((gpio_num_t)pin);
	hal_gpio_init(pin, HAL_PIN_OUTPUT);
	hal_gpio_write(pin, 0);
}
