// 硬件抽象层：时间基准实现

#include "hal/hal_time.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void hal_time_init() {
	// esp_timer 在系统启动时已初始化，这里只做一次自检
	(void)esp_timer_get_time();
}

void hal_delay_ms(uint32_t ms) {
	if (ms == 0) return;
	vTaskDelay(pdMS_TO_TICKS(ms));
}

void hal_delay_until_us(int64_t target_us) {
	int64_t now = esp_timer_get_time();
	if (target_us > now) {
		vTaskDelay(pdMS_TO_TICKS((uint32_t)((target_us - now) / 1000)));
	}
}
