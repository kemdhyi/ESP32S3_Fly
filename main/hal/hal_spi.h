// 硬件抽象层：SPI
// HAL: SPI master
//
// 提供 Bolder Flight 驱动所期待的 SPIClass / SPISettings 语义，
// 底层使用 ESP-IDF 的 spi_master。
//
// 关键点：
//   * MPU-9250 要求 SPI 模式 3（CPOL=1, CPHA=1）、MSB first
//   * 片选由软件手动控制（spics_io_num = -1 + CS_KEEP_ACTIVE），
//     与原 Arduino 驱动"一次事务内多次 transfer、CS 保持低"的行为一致
//   * 按 spi_clock 缓存 spi_device_handle_t，避免每次传输都重新 add_device

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/spi_master.h"

// 占位类型：驱动用它表示"使用默认片选"。IDF 下无此概念，仅保留声明以通过编译。
extern const int SS;

static constexpr uint8_t HAL_MSBFIRST = 1;
static constexpr uint8_t HAL_LSBFIRST = 0;

static constexpr uint8_t HAL_SPI_MODE0 = 0;
static constexpr uint8_t HAL_SPI_MODE1 = 1;
static constexpr uint8_t HAL_SPI_MODE2 = 2;
static constexpr uint8_t HAL_SPI_MODE3 = 3;

struct SPISettings {
	int32_t clock;
	uint8_t bitOrder;
	uint8_t dataMode;

	SPISettings() : clock(1000000), bitOrder(HAL_MSBFIRST), dataMode(HAL_SPI_MODE3) {}
	SPISettings(int32_t clock, uint8_t bitOrder, uint8_t dataMode)
		: clock(clock), bitOrder(bitOrder), dataMode(dataMode) {}
};

class SpiDeviceIdf {
public:
	SpiDeviceIdf() = default;
	SpiDeviceIdf(int sck, int miso, int mosi, int cs)
		: sck_(sck), miso_(miso), mosi_(mosi), cs_(cs) {}

	// ---- 与原 Arduino SPIClass 对齐的接口 ----
	void begin();                     // 使用构造时的引脚
	void begin(int sck, int miso, int mosi, int cs);

	void beginTransaction(const SPISettings& settings);
	void endTransaction();

	uint8_t transfer(uint8_t data);
	void transfer(uint8_t* buf, size_t count);

	int pinSS() const { return cs_; }

	// 片选手动控制（驱动在事务外切换 CS 时需要）
	void csLow();
	void csHigh();

private:
	spi_device_handle_t deviceFor(int32_t clock);

	int sck_ = -1;
	int miso_ = -1;
	int mosi_ = -1;
	int cs_ = -1;

	bool busReady_ = false;
	// 一次事务内的状态
	spi_device_handle_t activeDev_ = nullptr;
	int32_t activeClock_ = 0;
	bool inTransaction_ = false;

	// 按时钟频率缓存设备句柄（最多 4 档：1MHz 配置 / 15MHz 读取 / 预留）
	static constexpr int MAX_DEVICES = 4;
	struct DevEntry {
		int32_t clock = 0;
		spi_device_handle_t handle = nullptr;
	};
	DevEntry devices_[MAX_DEVICES] = {};
	int deviceCount_ = 0;
};

// 与原 Arduino 代码中的类型名保持一致，方便驱动少改动
using SPIClass = SpiDeviceIdf;

extern SpiDeviceIdf SPI;   // 全局默认 SPI 实例（驱动按需 begin）
