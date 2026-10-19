// 硬件抽象层：电机 PWM 实现

#include "hal/hal_pwm.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/hal_gpio.h"

static const char* TAG = "hal_pwm";

static constexpr ledc_mode_t      MODE     = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t     TIMER    = LEDC_TIMER_0;
static constexpr int              MAX_CH   = 8;

static int      s_channelOfPin[MAX_CH] = {};   // 逻辑索引 -> 通道号
static int      s_pinCount = 0;
static uint32_t s_resBits = 10;

void hal_motor_pwm_detach(int pin) {
	if (pin < 0) return;
	// 找到该引脚所占通道并停止输出
	for (int ch = 0; ch < LEDC_CHANNEL_MAX; ch++) {
		ledc_stop(MODE, (ledc_channel_t)ch, 0);
	}
	hal_gpio_release_pwm(pin);
}

void hal_motor_pwm_init(const int* pins, int count, uint32_t freq_hz,
                        uint32_t resolutionBits, int channelBase) {
	if (pins == nullptr || count <= 0) return;
	if (count > MAX_CH) count = MAX_CH;

	s_pinCount = count;
	s_resBits = resolutionBits;

	// 1) 定时器：25 kHz @ 10 bit
	ledc_timer_config_t tcfg = {};
	tcfg.speed_mode      = MODE;
	tcfg.duty_resolution = (ledc_timer_bit_t)resolutionBits;
	tcfg.timer_num       = TIMER;
	tcfg.freq_hz         = freq_hz;
	tcfg.clk_cfg         = LEDC_AUTO_CLK;   // S3 上解析为 XTAL 40MHz，与 Arduino 默认一致

	esp_err_t err = ledc_timer_config(&tcfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
		return;
	}

	// 回读真实频率（硬件可能因分频比取整而略有偏差）
	int actualFreq = (int)ledc_get_freq(MODE, TIMER);
	ESP_LOGI(TAG, "LEDC timer ready: requested=%lu Hz actual=%d Hz res=%lu bit",
	         (unsigned long)freq_hz, actualFreq, (unsigned long)resolutionBits);

	// 2) 通道
	for (int i = 0; i < count; i++) {
		int pin = pins[i];
		int ch  = channelBase + i;

		// 先停掉旧配置，等价于 ledcDetach
		hal_gpio_release_pwm(pin);

		ledc_channel_config_t ccfg = {};
		ccfg.gpio_num   = pin;
		ccfg.speed_mode = MODE;
		ccfg.channel    = (ledc_channel_t)ch;
		ccfg.timer_sel  = TIMER;
		ccfg.intr_type  = LEDC_INTR_DISABLE;
		ccfg.duty       = 0;
		ccfg.hpoint     = 0;

		err = ledc_channel_config(&ccfg);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "ledc_channel_config(pin=%d ch=%d) failed: %s",
			         pin, ch, esp_err_to_name(err));
			s_channelOfPin[i] = -1;
			continue;
		}
		s_channelOfPin[i] = ch;
	}
}

int hal_motor_pwm_actual_freq() {
	return (int)ledc_get_freq(MODE, TIMER);
}

void hal_motor_pwm_write(int channel, uint32_t duty) {	uint32_t maxDuty = (1u << s_resBits) - 1u;
	if (duty > maxDuty) duty = maxDuty;

	ledc_set_duty(MODE, (ledc_channel_t)channel, duty);
	ledc_update_duty(MODE, (ledc_channel_t)channel);
}
