// 硬件抽象层：RC 串口（CRSF / SBUS）
// HAL: RC UART
//
// 替代 Arduino 的 HardwareSerial。
//   * CRSF：420000 8N1，标准电平，无需反相
//   * SBUS：100000 8E2，信号反相（原 Arduino 版用 uart_->begin(baud, SERIAL_8E2, rx, tx, inv=true)）
//
// 与原代码 available()/read() 的映射：
//   available() -> hal_rc_uart_available()
//   read()      -> hal_rc_uart_read_byte()

#pragma once

#include <stddef.h>
#include <stdint.h>

// 初始化 RC 串口为 CRSF 模式（8N1，标准电平）
void hal_rc_uart_init_crsf(uint32_t baud, int rxPin, int txPin);

// 初始化 RC 串口为 SBUS 模式（8E2，RX/TX 反相）
void hal_rc_uart_init_sbus(uint32_t baud, int rxPin);

// 释放 RC 串口占用的资源（切换协议或重新配置引脚时调用）
void hal_rc_uart_deinit();

// 接收缓冲区中可读字节数
size_t hal_rc_uart_available();

// 读取一个字节，成功返回 true
bool hal_rc_uart_read_byte(uint8_t* out);

// 发送数据（遥测回传预留）
size_t hal_rc_uart_write(const uint8_t* data, size_t len);

// 清空接收缓冲区
void hal_rc_uart_flush_input();
