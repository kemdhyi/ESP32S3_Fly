// 硬件抽象层：调试控制台串口实现

#include "hal/hal_console.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "hal/board.h"
#include "sdkconfig.h"

static constexpr uart_port_t CONSOLE_UART = (uart_port_t)CONFIG_CFDRONE_CONSOLE_UART_NUM;
static constexpr int CONSOLE_RX_BUF = 1024;
static constexpr int CONSOLE_TX_BUF = 2048;

void hal_console_init(uint32_t baud) {
	uart_config_t cfg = {};
	cfg.baud_rate  = (int)baud;
	cfg.data_bits  = UART_DATA_8_BITS;
	cfg.parity     = UART_PARITY_DISABLE;
	cfg.stop_bits  = UART_STOP_BITS_1;
	cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	// UART0 通常已被 ROM 打印初始化，这里重配为带驱动的模式
	esp_err_t err = uart_driver_install(CONSOLE_UART, CONSOLE_RX_BUF, CONSOLE_TX_BUF, 0, nullptr, 0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		return;
	}
	uart_param_config(CONSOLE_UART, &cfg);
	// 引脚保持 ROM 默认（UART0 的 TX/RX 由板级 USB 桥接固定），不调用 uart_set_pin
}

void hal_console_write(const char* data, size_t len) {
	if (data == nullptr || len == 0) return;
	uart_write_bytes(CONSOLE_UART, data, len);
}

void hal_console_write_str(const char* str) {
	if (str == nullptr) return;
	hal_console_write(str, strlen(str));
}

size_t hal_console_available() {
	size_t len = 0;
	if (uart_get_buffered_data_len(CONSOLE_UART, &len) != ESP_OK) return 0;
	return len;
}

bool hal_console_read_byte(uint8_t* out) {
	if (out == nullptr) return false;
	return uart_read_bytes(CONSOLE_UART, out, 1, 0) == 1;
}
