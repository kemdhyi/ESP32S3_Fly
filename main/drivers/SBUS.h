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
//   * 删除 Teensy / STM32 / 通用 Arduino 的 HardwareSerial 分支，
//     改为调用本工程的 hal_rc_uart_* （底层为 ESP-IDF UART 驱动）。
//   * 原 ESP32 分支使用 uart_->begin(baud_, SERIAL_8E2, rxpin_, txpin_, inv_)；
//     IDF 版等价实现为 8E2 + uart_set_line_inverse(RXD_INV|TXD_INV)。
//   * 删除 SBUSTx（遥测回传未启用），仅保留接收端解析逻辑。
//     原始实现保留在工程根目录的 drivers/SBUS.cpp 中以便对比。

#ifndef SRC_SBUS_H_
#define SRC_SBUS_H_

#include <cstddef>
#include <cstdint>

struct SBUSData {
	bool lost_frame;
	bool failsafe;
	bool ch17, ch18;
	static constexpr int8_t NUM_CH = 16;
	int16_t ch[NUM_CH];
};

class SBUS {
 public:
	// rxpin < 0 表示使用板级默认；inv 为 SBUS 信号反相（标准 SBUS 需要）
	explicit SBUS(int rxpin = -1, bool inv = true, bool fast = false)
		: inv_(inv), fast_(fast), rxpin_(rxpin) {}

	void begin();
	void begin(int rxpin, bool inv = true, bool fast = false);
	bool read();
	inline SBUSData data() const { return data_; }

 private:
	/* Communication */
	bool inv_ = true;
	bool fast_ = false;
	int8_t rxpin_ = -1;
	int32_t baud_ = 100000;
	/* Message len */
	static constexpr int8_t PAYLOAD_LEN_ = 23;
	static constexpr int8_t HEADER_LEN_ = 1;
	static constexpr int8_t FOOTER_LEN_ = 1;
	/* SBUS message defs */
	static constexpr int8_t NUM_SBUS_CH_ = 16;
	static constexpr uint8_t HEADER_ = 0x0F;
	static constexpr uint8_t FOOTER_ = 0x00;
	static constexpr uint8_t FOOTER2_ = 0x04;
	static constexpr uint8_t CH17_MASK_ = 0x01;
	static constexpr uint8_t CH18_MASK_ = 0x02;
	static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
	static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
	/* Parsing state tracking */
	int8_t state_ = 0;
	uint8_t prev_byte_ = FOOTER_;
	uint8_t cur_byte_;
	/* Buffer for storing messages */
	uint8_t buf_[25];
	/* Data */
	bool new_data_;
	SBUSData data_;
	bool Parse();
};

#endif  // SRC_SBUS_H_
