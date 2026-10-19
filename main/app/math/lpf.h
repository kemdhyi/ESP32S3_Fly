// 低通滤波器实现
// Low pass filter implementation
//
// ESP-IDF 原生版本：补全 include，算法与 Arduino 版逐行一致。

#pragma once

#include <math.h>

template <typename T> // 模板化以便同时用于标量与矢量
class LowPassFilter {
public:
	float alpha; // 平滑系数，1 表示禁用滤波
	T output;

	explicit LowPassFilter(float alpha) : alpha(alpha) {}

	T update(const T input) {
		if (alpha == 1) { // 滤波禁用
			return input;
		}

		if (!initialized) {
			output = input;
			initialized = true;
		}

		return output += alpha * (input - output);
	}

	void setCutOffFrequency(float cutOffFreq, float dt) {
		alpha = 1 - expf(-2 * (float)M_PI * cutOffFreq * dt);
	}

	void reset() {
		initialized = false;
	}

private:
	bool initialized = false;
};
