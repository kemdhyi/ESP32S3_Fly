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
//   * 删除 Teensy / IMXRT 专用分支（digitalWriteFast、delayNanoseconds）。
//   * digitalWrite() -> gpio_set_level()，delay() -> vTaskDelay()。
//   * SPIClass 由 hal/hal_spi.h 提供（底层 spi_master，软件片选、单事务内保持 CS 低）。

#include "invensense_imu.h"  // NOLINT

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void InvensenseImu::Config(TwoWire *i2c, const uint8_t addr) {
  i2c_ = i2c;
  dev_ = addr;
  iface_ = I2C;
}

void InvensenseImu::Config(SPIClass *spi, const uint8_t cs) {
  spi_ = spi;
  dev_ = cs;
  iface_ = SPI;
}

void InvensenseImu::Begin() {
  if (iface_ == SPI) {
    spi_->begin();
    if (dev_ == 255) {
      // 使用总线默认片选（IDF 版无此概念，取 SPI 实例记录的值）
      dev_ = (uint8_t)spi_->pinSS();
    }
    gpio_set_direction((gpio_num_t)dev_, GPIO_MODE_OUTPUT);
    /* 切换 CS 以锁定 SPI 模式 */
    gpio_set_level((gpio_num_t)dev_, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level((gpio_num_t)dev_, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool InvensenseImu::WriteRegister(const uint8_t reg, const uint8_t data,
                                  const int32_t spi_clock) {
  uint8_t ret_val;
  if (iface_ == I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(spi_clock, HAL_MSBFIRST, HAL_SPI_MODE3));
    spi_->transfer(reg);
    spi_->transfer(data);
    spi_->endTransaction();
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  ReadRegisters(reg, sizeof(ret_val), spi_clock, &ret_val);
  return data == ret_val;
}

bool InvensenseImu::ReadRegisters(const uint8_t reg, const uint8_t count,
                                  const int32_t spi_clock,
                                  uint8_t * const data) {
  if (!data) {return false;}
  if (iface_ == I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    bytes_rx_ = static_cast<uint8_t>(i2c_->requestFrom(static_cast<uint8_t>(dev_), count));
    if (bytes_rx_ == count) {
      for (size_t i = 0; i < count; i++) {
        data[i] = static_cast<uint8_t>(i2c_->read());
      }
      return true;
    } else {
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(spi_clock, HAL_MSBFIRST, HAL_SPI_MODE3));
    spi_->transfer(static_cast<uint8_t>(reg | SPI_READ_));
    spi_->transfer(data, count);
    spi_->endTransaction();
    return true;
  }
}

bool InvensenseImu::WriteRegister(const uint8_t reg, const uint8_t data) {
  if (iface_ == I2C) {
    return WriteRegister(reg, data, 0);
  } else {
    return false;
  }
}

bool InvensenseImu::ReadRegisters(const uint8_t reg, const uint8_t count,
                                  uint8_t * const data) {
  if (iface_ == I2C) {
    return ReadRegisters(reg, count, 0, data);
  } else {
    return false;
  }
}
