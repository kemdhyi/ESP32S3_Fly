/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

// 移植说明（ESP-IDF）：
//   * 原文件通过 `#ifdef ESP32` 使用 Arduino-ESP32 的 hw_timer_t + attachInterruptArg
//     来产生 IMU 采样节拍。ESP-IDF 下这两个 API 都不存在。
//   * 本版改用 esp_timer 周期回调 + 二值信号量：
//       - esp_timer 回调运行在 esp_timer 任务上下文（非中断），
//         因此使用 xSemaphoreGive() 而非 xSemaphoreGiveFromISR()。
//       - waitForData() 中 xSemaphoreTake(portMAX_DELAY) 的语义与 Arduino 版一致。
//   * 如改用 IMU 的 DRDY 引脚中断，应走 gpio_install_isr_service() +
//     gpio_isr_handler_add()，那里才需要 FromISR 版本与 portYIELD_FROM_ISR()。

#pragma once

#include <math.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "drivers/logger.h"
#include "hal/hal_console.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_spi.h"

// IMU driver interface
class IMUInterface {
public:
	enum DLPF {
		DLPF_OFF,
		DLPF_MAX,
		DLPF_100HZ_APPROX,
		DLPF_50HZ_APPROX,
		DLPF_5HZ_APPROX,
		DLPF_MIN
	};
	enum AccelRange {
		ACCEL_RANGE_MIN,
		ACCEL_RANGE_2G,
		ACCEL_RANGE_4G,
		ACCEL_RANGE_8G,
		ACCEL_RANGE_16G,
		ACCEL_RANGE_MAX
	};
	enum GyroRange {
		GYRO_RANGE_MIN,
		GYRO_RANGE_250DPS,
		GYRO_RANGE_500DPS,
		GYRO_RANGE_1000DPS,
		GYRO_RANGE_2000DPS,
		GYRO_RANGE_MAX
	};
	enum Rate {
		RATE_MIN,
		RATE_50HZ_APPROX,
		RATE_1KHZ_APPROX,
		RATE_8KHZ_APPROX,
		RATE_MAX
	};
	virtual bool begin() = 0;
	virtual void reset() = 0;
	virtual int status() const = 0; // 0 - success, otherwise error
	virtual uint8_t whoAmI() = 0;
	virtual bool read() = 0;
	virtual void waitForData() = 0;
	virtual void getAccel(float& x, float& y, float& z) const = 0;
	virtual void getGyro(float& x, float& y, float& z) const = 0;
	virtual void getMag(float& x, float& y, float& z) const = 0;
	virtual float getTemp() = 0;
	virtual bool setRate(const Rate rate) = 0;
	virtual float getRate() = 0;
	virtual bool setAccelRange(const AccelRange range) = 0;
	virtual bool setGyroRange(const GyroRange range) = 0;
	virtual bool setDLPF(const DLPF dlpf) = 0;
	virtual const char* getModel() const = 0;
	virtual bool setupInterrupt() = 0;
};

// Base for all IMU drivers
class IMUBase : public IMUInterface, public Logger {
private:
	bool usingInterrupt = false;
	int interruptPin = -1;

	SemaphoreHandle_t interruptSemaphore = nullptr;
	esp_timer_handle_t timer = nullptr;

	// esp_timer 回调：任务上下文，只能用非 FromISR 版本
	static void timerCallback(void* arg) {
		xSemaphoreGive((SemaphoreHandle_t)arg);
	}

	bool setupInterruptTimer() {
		if (interruptSemaphore == nullptr) {
			interruptSemaphore = xSemaphoreCreateBinary();
			if (interruptSemaphore == nullptr) {
				log("Failed to create semaphore");
				return false;
			}
		}

		float rate = getRate();
		if (!(rate > 0)) {
			log("Invalid IMU rate");
			return false;
		}
		uint64_t period_us = (uint64_t)llround(1000000.0 / (double)rate);

		if (timer != nullptr) {
			esp_timer_stop(timer);
			esp_timer_delete(timer);
			timer = nullptr;
		}

		esp_timer_create_args_t args = {};
		args.callback = &IMUBase::timerCallback;
		args.arg = interruptSemaphore;
		args.dispatch_method = ESP_TIMER_TASK;
		args.name = "imu_tick";

		if (esp_timer_create(&args, &timer) != ESP_OK) {
			log("Failed to create timer");
			return false;
		}
		if (esp_timer_start_periodic(timer, period_us) != ESP_OK) {
			log("Failed to start timer");
			return false;
		}

		usingInterrupt = true;
		return true;
	}

	bool setupInterruptPin(uint8_t pin) {
		if (interruptSemaphore == nullptr) {
			interruptSemaphore = xSemaphoreCreateBinary();
		}
		hal_gpio_init(pin, HAL_PIN_INPUT_PULLUP);
		// 注：DRDY 引脚中断路径需额外接入 gpio_install_isr_service()，
		// 本工程默认走定时器节拍，此处仅保留接口。
		usingInterrupt = false;
		interruptPin = pin;
		return false;
	}

protected:
	// 中断源配置实现。原始 Bolder Flight 代码把它命名为 `setupInterrupt(int pin = -1)`，
	// 与基类纯虚 `setupInterrupt()` 构成同名重载并互相遮蔽（-Woverloaded-virtual）。
	// 这里改用独立名称，语义与调用点完全不变：
	//   setupInterruptSource()   -> 使用定时器节拍（本工程默认路径）
	//   setupInterruptSource(pin)-> 使用 DRDY 引脚中断
	bool setupInterruptSource(int pin = -1) {
		if (usingInterrupt) return true; // already set

		if (pin == -1) {
			return setupInterruptTimer();
		} else {
			return setupInterruptPin((uint8_t)pin);
		}
	}

public:
	void waitForData() override {
		if (this->status() && interruptPin != -1) return; // don't hang if error and interrupt pin is used

		if (usingInterrupt) {
			xSemaphoreTake(interruptSemaphore, portMAX_DELAY); // 等待节拍信号
			this->read();
		} else {
			while (!this->read()) {} // 轮询等待
		}
	}
};
