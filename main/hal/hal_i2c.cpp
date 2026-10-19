// 硬件抽象层：I2C 占位实现
//
// 仅提供全局 Wire 实例。本工程 IMU 走 SPI，不产生任何 I2C 总线访问。

#include "hal/hal_i2c.h"

TwoWire Wire;
