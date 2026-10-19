// 时间相关函数
//
// 原 Arduino 版使用 micros()/1000000.0 作为时间基准。
// 这里改用 hal_micros64()（esp_timer_get_time()，64 位微秒，不回绕）。

#include "app/time.h"

#include "app/globals.h"
#include "hal/hal_time.h"

float t = NAN;      // 当前步进时间，单位：秒
float dt;           // 与上一步进的时间差，单位：秒
float loopRate;     // Hz

void step() {
	float now = (float)(hal_micros64() / 1000000.0);
	dt = now - t;
	t = now;

	if (!(dt > 0)) {
		dt = 0; // 首次步进与计数异常时置零
	}

	computeLoopRate();
}

void computeLoopRate() {
	static float windowStart = 0;
	static uint32_t rate = 0;
	rate++;
	if (t - windowStart >= 1) { // 1 秒窗口
		loopRate = (float)rate;
		windowStart = t;
		rate = 0;
	}
}
