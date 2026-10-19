// PID控制器实现
// PID controller implementation
//
// ESP-IDF 原生版本：constrain 由 math/util.h 提供，算法与 Arduino 版逐行一致。

#pragma once

#include "app/math/lpf.h"
#include "app/math/util.h"

class PID {
public:
	float p, i, d;
	float windup;
	float dtMax;

	float derivative = 0;
	float integral = 0;

	LowPassFilter<float> lpf; // 微分项低通滤波器

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1f) :
		p(p), i(i), d(d), windup(windup), dtMax(dtMax), lpf(dAlpha) {}

	float update(float error) {
		float dt = t - prevTime;

		if (dt > 0 && dt < dtMax) {
			integral += error * dt;
			derivative = lpf.update((error - prevError) / dt); // 计算微分并低通滤波
		} else {
			integral = 0;
			derivative = 0;
		}

		prevError = error;
		prevTime = t;

		return p * error + constrain(i * integral, -windup, windup) + d * derivative; // PID
	}

	void reset() {
		prevError = NAN;
		prevTime = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN;
	float prevTime = NAN;
};
