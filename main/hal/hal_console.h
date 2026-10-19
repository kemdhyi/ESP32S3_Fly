// 硬件抽象层：调试控制台串口
// HAL: console UART
//
// 替代 Arduino 的 Serial（UART0，115200）。用于 CLI 交互与 print() 输出。

#pragma once

#include <stddef.h>
#include <stdint.h>

void hal_console_init(uint32_t baud);

// 写出原始字节（不添加换行）
void hal_console_write(const char* data, size_t len);

// 写出以 NUL 结尾的字符串
void hal_console_write_str(const char* str);

// 缓冲区中可读字节数
size_t hal_console_available();

// 读取一个字节，成功返回 true
bool hal_console_read_byte(uint8_t* out);
