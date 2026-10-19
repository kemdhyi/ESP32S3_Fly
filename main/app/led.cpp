// 板载LED灯控制

#include "app/led.h"

#include "app/globals.h"
#include "app/imu.h"
#include "app/safety.h"
#include "hal/board.h"
#include "hal/hal_gpio.h"
#include "hal/hal_time.h"

#if WEB_RC_ENABLED
#include "comm/web_rc.h"
#endif

#if BOARD_LED_ENABLED

#define BLINK_PERIOD      500000  // 慢闪：500ms 半周期 → 1 Hz
#define BLINK_FAST_PERIOD  62500  // 快闪：62.5ms 半周期 → 8 Hz

void setupLED() {
	hal_gpio_init(BOARD_LED_PIN, HAL_PIN_OUTPUT);
	// 初始熄灭
	hal_gpio_write(BOARD_LED_PIN, BOARD_LED_INVERTED ? 1 : 0);
}

void setLED(bool on) {
	static bool state = false;
	if (on == state) {
		return; // 状态未变化时不写 GPIO
	}
	hal_gpio_write(BOARD_LED_PIN, (on ^ (BOARD_LED_INVERTED != 0)) ? 1 : 0);
	state = on;
}

void blinkLED() {
	setLED((hal_micros() / BLINK_PERIOD) % 2);
}

// 电池告警：按飞行状态选择阈值
// 飞行中（thrustTarget >= 0.15）→ L2（3.3V），L1 在飞行中不适用
// 未解锁 / 解锁怠速 → L1（3.5V）
bool batteryAlertActive() {
	if (batteryVoltage <= VBAT_ABSENT_THRESHOLD) return false;
	bool flying = armed && thrustTarget >= 0.15f;
	if (flying) return batteryVoltage < VBAT_LOW_THRESHOLD;    // L2：飞行中
	else        return batteryVoltage < VBAT_WARN_THRESHOLD;   // L1：未解锁/怠速
}

// 检测是否有任意告警（低电 / 遥控失联 / 倒置）
bool ledAlertActive() {
	// 倒置检测
	if (isInverted) return true;

	// 遥控失联检测（SBUS/CRSF，仅解锁后）
	if (controlTime != 0 && armed && (t - controlTime > rcLossTimeout)) return true;

#if WEB_RC_ENABLED
	// Web RC 失联检测：已激活但超时
	if (webRCEnabled && useWebRC && !isUsingWebRC()) return true;
#endif

	if (batteryAlertActive()) return true;

	return false;
}

// 主循环调用：根据飞行状态驱动 LED
void updateLED() {
	if (!armed) {
		if (batteryAlertActive()) {
			setLED((hal_micros() / BLINK_FAST_PERIOD) % 2); // 解锁前低电：快闪
		} else {
			setLED(false); // 正常待机：常灭
		}
		return;
	}
	if (ledAlertActive()) {
		setLED((hal_micros() / BLINK_FAST_PERIOD) % 2); // 任何告警：快闪 8Hz
	} else {
		setLED((hal_micros() / BLINK_PERIOD) % 2); // 正常飞行：慢闪 1Hz
	}
}

#else // BOARD_LED_ENABLED == 0

void setupLED() {}
void setLED(bool on) { (void)on; }
void blinkLED() {}
bool batteryAlertActive() { return false; }
bool ledAlertActive() { return false; }
void updateLED() {}

#endif // BOARD_LED_ENABLED
