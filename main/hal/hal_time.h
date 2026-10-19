// 硬件抽象层：时间基准
// HAL: time base
//
// 替代 Arduino 的 micros() / millis() / delay()。
// esp_timer_get_time() 为 64 位微秒计数器，精度与 micros() 一致，且不会回绕。

#pragma once

#include <stdint.h>

#include "esp_timer.h"

// 微秒计数器（64 位，不会回绕）
static inline int64_t hal_micros64() { return esp_timer_get_time(); }

// 32 位截断版本，语义与 Arduino 的 micros() 完全一致（约 71 分钟回绕）
static inline uint32_t hal_micros() { return (uint32_t)esp_timer_get_time(); }

// 毫秒计数器（64 位）
static inline int64_t hal_millis64() { return esp_timer_get_time() / 1000; }

// 32 位截断版本，语义与 Arduino 的 millis() 一致
static inline uint32_t hal_millis() { return (uint32_t)(esp_timer_get_time() / 1000); }

// 阻塞延时（毫秒）。注意：飞控主循环内应避免调用。
void hal_delay_ms(uint32_t ms);

// 阻塞延时至指定绝对微秒时刻
void hal_delay_until_us(int64_t target_us);

void hal_time_init();
