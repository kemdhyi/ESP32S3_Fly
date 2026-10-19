// 硬件抽象层：电机 PWM（LEDC）
// HAL: motor PWM via LEDC
//
// 替代 Arduino-ESP32 3.x 的 ledcAttach(pin,...) / ledcWrite(pin,...) / ledcDetach(pin)。
// 与 Arduino 版的差异：
//   * Arduino 的 ledcAttach 以「引脚」为索引自动分配通道/定时器；
//     IDF 的 LEDC 以「通道 + 定时器」为索引，因此这里显式管理 1 个定时器 + N 个通道。
//   * 25 kHz @ 10 bit 在 ESP32-S3（XTAL 40 MHz）下分频比为 1.5625，硬件支持小数分频，
//     频率可精确输出；若硬件无法整除，ledc_timer_config 会自动取最近可行值，
//     此时必须用 hal_motor_pwm_actual_freq() 回读真实频率（与原代码 ledcChangeFrequency 回读一致）。

#pragma once

#include <stdint.h>

// 配置定时器 + 通道。channelBase 指定起始通道号。
// 若通道已在别处绑定，会先 stop 再重新配置，等价于 ledcDetach + ledcAttach。
void hal_motor_pwm_init(const int* pins, int count, uint32_t freq_hz,
                        uint32_t resolutionBits, int channelBase);

// 写入占空比（原始 LEDC 计数，0 .. 2^resolution-1）
void hal_motor_pwm_write(int channel, uint32_t duty);

// 查询定时器实际输出频率（硬件可能因分频比取整而略有偏差）
int hal_motor_pwm_actual_freq();

// 解除某个引脚的 PWM 绑定并拉低（等价于 ledcDetach + digitalWrite(LOW)）
void hal_motor_pwm_detach(int pin);
