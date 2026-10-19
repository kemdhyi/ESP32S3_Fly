// 硬件抽象层：RC 串口实现

#include "hal/hal_uart.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/board.h"

static const char* TAG = "hal_uart";

static constexpr uart_port_t RC_UART = (uart_port_t)BOARD_RC_UART_NUM;
static constexpr int RC_RX_BUF = 1024;
static constexpr int RC_TX_BUF = 256;

static bool s_installed = false;

void hal_rc_uart_deinit() {
	if (!s_installed) return;
	uart_driver_delete(RC_UART);
	s_installed = false;
}

static bool installDriver() {
	if (s_installed) return true;
	esp_err_t err = uart_driver_install(RC_UART, RC_RX_BUF, RC_TX_BUF, 0, nullptr, 0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
		return false;
	}
	s_installed = true;
	return true;
}

static void configurePins(int rxPin, int txPin, bool invert) {
	if (invert) {
		uart_set_line_inverse(RC_UART, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
	} else {
		uart_set_line_inverse(RC_UART, UART_SIGNAL_INV_DISABLE);
	}
	// txPin < 0 表示不启用发送
	uart_set_pin(RC_UART,
	             txPin < 0 ? UART_PIN_NO_CHANGE : txPin,
	             rxPin < 0 ? UART_PIN_NO_CHANGE : rxPin,
	             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void hal_rc_uart_init_crsf(uint32_t baud, int rxPin, int txPin) {
	hal_rc_uart_deinit();

	if (!installDriver()) return;

	uart_config_t cfg = {};
	cfg.baud_rate  = (int)baud;
	cfg.data_bits  = UART_DATA_8_BITS;
	cfg.parity     = UART_PARITY_DISABLE;
	cfg.stop_bits  = UART_STOP_BITS_1;
	cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	esp_err_t err = uart_param_config(RC_UART, &cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config(CRSF) failed: %s", esp_err_to_name(err));
		return;
	}
	configurePins(rxPin, txPin, false);
	uart_flush_input(RC_UART);

	ESP_LOGI(TAG, "RC UART%d CRSF ready: %lu 8N1 rx=%d tx=%d",
	         (int)RC_UART, (unsigned long)baud, rxPin, txPin);
}

void hal_rc_uart_init_sbus(uint32_t baud, int rxPin) {
	hal_rc_uart_deinit();

	if (!installDriver()) return;

	uart_config_t cfg = {};
	cfg.baud_rate  = (int)baud;
	cfg.data_bits  = UART_DATA_8_BITS;
	cfg.parity     = UART_PARITY_EVEN;    // SBUS 为 8E2
	cfg.stop_bits  = UART_STOP_BITS_2;
	cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	esp_err_t err = uart_param_config(RC_UART, &cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config(SBUS) failed: %s", esp_err_to_name(err));
		return;
	}
	configurePins(rxPin, -1, true);   // SBUS 信号反相
	uart_flush_input(RC_UART);

	ESP_LOGI(TAG, "RC UART%d SBUS ready: %lu 8E2 inverted rx=%d",
	         (int)RC_UART, (unsigned long)baud, rxPin);
}

size_t hal_rc_uart_available() {
	if (!s_installed) return 0;
	size_t len = 0;
	if (uart_get_buffered_data_len(RC_UART, &len) != ESP_OK) return 0;
	return len;
}

bool hal_rc_uart_read_byte(uint8_t* out) {
	if (!s_installed || out == nullptr) return false;
	return uart_read_bytes(RC_UART, out, 1, 0) == 1;
}

size_t hal_rc_uart_write(const uint8_t* data, size_t len) {
	if (!s_installed || data == nullptr || len == 0) return 0;
	int written = uart_write_bytes(RC_UART, data, len);
	return written < 0 ? 0 : (size_t)written;
}

void hal_rc_uart_flush_input() {
	if (!s_installed) return;
	uart_flush_input(RC_UART);
}
