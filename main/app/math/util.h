// 实用函数
// Utility functions
//
// ESP-IDF 原生版本：移除了 Arduino String / Print 依赖。
// splitString() 改为标准 C 字符串版本（见 hal/string_util），
// 打印相关能力统一由 app/cli.h 的 print() 提供。

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>

#include "esp_private/brownout.h"
#include "sdkconfig.h"

// 本工程仅面向 ESP32-S3
#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "CF-Drone IDF port targets ESP32-S3 only. Run 'idf.py set-target esp32s3'."
#endif

constexpr float ONE_G = 9.80665f;

// 主循环时间基准，定义于 app/time.cpp
extern float t;

// ---- 数学辅助（等价于 Arduino 的 constrain/radians/degrees）----
template <typename T>
static inline T constrain(T x, T lo, T hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}

static inline float radians(float deg) { return deg * (float)M_PI / 180.0f; }
static inline float degrees(float rad) { return rad * 180.0f / (float)M_PI; }

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline bool invalid(float x) { return !isfinite(x); }
static inline bool valid(float x) { return isfinite(x); }

// Wrap angle to [-PI, PI)
static inline float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * (float)M_PI);
	if (angle > (float)M_PI) {
		angle -= 2 * (float)M_PI;
	} else if (angle < -(float)M_PI) {
		angle += 2 * (float)M_PI;
	}
	return angle;
}

// 禁用 ESP32-S3 低压复位检测（电机启动瞬间电压跌落会误触发 brownout reset）
// 对应 Arduino 版的 disableBrownOut()
void disableBrownOut();

// ---- 频率限制器 ----
// 依赖全局 t（秒）。用法：static Rate r(10); if (r) { ... }
class Rate {
public:
	float rate;
	float last = 0;
	explicit Rate(float rate) : rate(rate) {}

	operator bool() {
		if (t - last >= 1 / rate) {
			last = t;
			return true;
		}
		return false;
	}
};

// ---- 布尔信号延迟滤波器 ----
// 保证信号在一段时间内持续为真后才返回 true
class Delay {
public:
	float delay;
	float start = NAN;
	explicit Delay(float delay) : delay(delay) {}

	bool update(bool on) {
		if (!on) {
			start = NAN;
			return false;
		} else if (isnan(start)) {
			start = t;
		}
		return t - start >= delay;
	}
};
