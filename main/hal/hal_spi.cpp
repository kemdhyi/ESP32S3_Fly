// 硬件抽象层：SPI 实现

#include "hal/hal_spi.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/board.h"

static const char* TAG = "hal_spi";

const int SS = -1;

// 全局默认实例：引脚在 begin() 时确定
SpiDeviceIdf SPI;

void SpiDeviceIdf::begin() {
	begin(sck_, miso_, mosi_, cs_);
}

void SpiDeviceIdf::begin(int sck, int miso, int mosi, int cs) {
	sck_ = sck;
	miso_ = miso;
	mosi_ = mosi;
	cs_ = cs;

	if (busReady_) return;

	spi_bus_config_t bus = {};
	bus.mosi_io_num = mosi;
	bus.miso_io_num = miso;
	bus.sclk_io_num = sck;
	bus.quadwp_io_num = -1;
	bus.quadhd_io_num = -1;
	bus.max_transfer_sz = 64;

	esp_err_t err = spi_bus_initialize((spi_host_device_t)BOARD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
		return;
	}
	busReady_ = true;
	ESP_LOGI(TAG, "SPI bus ready (sck=%d miso=%d mosi=%d cs=%d)", sck, miso, mosi, cs);
}

spi_device_handle_t SpiDeviceIdf::deviceFor(int32_t clock) {
	for (int i = 0; i < deviceCount_; i++) {
		if (devices_[i].clock == clock) return devices_[i].handle;
	}
	if (deviceCount_ >= MAX_DEVICES) {
		ESP_LOGE(TAG, "too many SPI clock rates cached");
		return nullptr;
	}

	spi_device_interface_config_t dev = {};
	dev.clock_speed_hz = clock;
	dev.mode = 3;                    // CPOL=1, CPHA=1 —— MPU-9250 要求
	dev.spics_io_num = -1;           // 软件控制片选
	dev.queue_size = 1;
	dev.flags = SPI_DEVICE_NO_DUMMY;

	spi_device_handle_t handle = nullptr;
	esp_err_t err = spi_bus_add_device((spi_host_device_t)BOARD_SPI_HOST, &dev, &handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi_bus_add_device(%ld) failed: %s", (long)clock, esp_err_to_name(err));
		return nullptr;
	}
	devices_[deviceCount_].clock = clock;
	devices_[deviceCount_].handle = handle;
	deviceCount_++;
	return handle;
}

void SpiDeviceIdf::csLow() {
	if (cs_ >= 0) gpio_set_level((gpio_num_t)cs_, 0);
}

void SpiDeviceIdf::csHigh() {
	if (cs_ >= 0) gpio_set_level((gpio_num_t)cs_, 1);
}

void SpiDeviceIdf::beginTransaction(const SPISettings& settings) {
	activeClock_ = settings.clock;
	activeDev_ = deviceFor(settings.clock);
	inTransaction_ = true;
	csLow();
}

void SpiDeviceIdf::endTransaction() {
	csHigh();
	inTransaction_ = false;
	activeDev_ = nullptr;
}

uint8_t SpiDeviceIdf::transfer(uint8_t data) {
	if (activeDev_ == nullptr) return 0;

	spi_transaction_t trans = {};
	trans.length = 8;
	trans.tx_buffer = &data;
	trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;   // 一次事务内保持片选
	uint8_t rx = 0;
	trans.rx_buffer = &rx;

	esp_err_t err = spi_device_polling_transmit(activeDev_, &trans);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi transfer failed: %s", esp_err_to_name(err));
		return 0;
	}
	return rx;
}

void SpiDeviceIdf::transfer(uint8_t* buf, size_t count) {
	if (activeDev_ == nullptr || buf == nullptr || count == 0) return;

	spi_transaction_t trans = {};
	trans.length = count * 8;
	trans.rxlength = count * 8;
	trans.tx_buffer = buf;
	trans.rx_buffer = buf;                    // 原地读写（全双工）
	trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;

	esp_err_t err = spi_device_polling_transmit(activeDev_, &trans);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi bulk transfer failed: %s", esp_err_to_name(err));
	}
}
