// 硬件抽象层：I2C（占位实现）
// HAL: I2C stub
//
// Bolder Flight 的 MPU9250 驱动同时支持 I2C 与 SPI 两种总线，构造函数与成员
// 均以 TwoWire* 形式引用 Arduino 的 Wire 库。
//
// 本工程只使用 SPI 连接 IMU：
//   * AK8963 磁力计挂在 MPU-9250 内部的 I2C 主控总线上，
//     由 MPU-9250 自己读取，主机侧仍只走 SPI。
//   * 扩展板 I2C 传感器（BMP388 等）尚未实现。
//
// 因此这里只提供满足编译期类型要求的占位类。由于 SPI 路径下不会实例化
// I2C 调用，不需要任何函数定义。

#pragma once

#include <stddef.h>
#include <stdint.h>

class TwoWire {
public:
	// 与原 Arduino Wire 签名对齐，仅用于类型匹配
	void begin() {}
	void begin(int sda, int scl) { (void)sda; (void)scl; }

	void beginTransmission(uint8_t address) { (void)address; }
	size_t write(uint8_t data) { (void)data; return 1; }
	size_t write(const uint8_t* data, size_t len) { (void)data; return len; }
	uint8_t endTransmission(bool stopBit = true) { (void)stopBit; return 0; }

	uint8_t requestFrom(uint8_t address, uint8_t len) { (void)address; (void)len; return 0; }
	int read() { return -1; }
	int available() { return 0; }
};

// 全局默认实例，供驱动构造函数取用（本工程不会实际使用）
extern TwoWire Wire;
