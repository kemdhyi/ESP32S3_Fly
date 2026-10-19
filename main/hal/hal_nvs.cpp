// 硬件抽象层：参数持久化实现

#include "hal/hal_nvs.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG = "hal_nvs";

bool NvsStore::begin(const char* name, bool readOnly) {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// 分区表变更 / NVS 数据版本不兼容 / 异常断电导致损坏
		// —— 对应原 Preferences.begin() 失败时的 nvs_flash_erase() + 重试路径
		ESP_LOGW(TAG, "nvs_flash_init -> %s, erasing partition and retrying",
		         esp_err_to_name(err));
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
		return false;
	}

	nvs_open_mode_t mode = readOnly ? NVS_READONLY : NVS_READWRITE;
	nvs_handle_t h = 0;
	err = nvs_open(name, mode, &h);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "nvs_open('%s') failed: %s", name, esp_err_to_name(err));
		return false;
	}
	handle_ = (void*)h;
	ESP_LOGI(TAG, "NVS namespace '%s' opened", name);
	return true;
}

void NvsStore::end() {
	if (handle_ != nullptr) {
		nvs_close((nvs_handle_t)handle_);
		handle_ = nullptr;
	}
}

bool NvsStore::isKey(const char* key) {
	if (handle_ == nullptr || key == nullptr) return false;
	size_t len = 0;
	return nvs_get_blob((nvs_handle_t)handle_, key, nullptr, &len) == ESP_OK ||
	       nvs_get_i32((nvs_handle_t)handle_, key, nullptr) == ESP_OK ||
	       nvs_get_u32((nvs_handle_t)handle_, key, nullptr) == ESP_OK ||
	       nvs_get_str((nvs_handle_t)handle_, key, nullptr, &len) == ESP_OK;
}

float NvsStore::getFloat(const char* key, float defaultValue) {
	if (handle_ == nullptr || key == nullptr) return defaultValue;
	uint32_t bits = 0;
	// 与原 Preferences::putFloat 一致：float 以 4 字节 blob 存储
	size_t len = sizeof(bits);
	esp_err_t err = nvs_get_blob((nvs_handle_t)handle_, key, &bits, &len);
	if (err != ESP_OK || len != sizeof(bits)) return defaultValue;

	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

size_t NvsStore::putFloat(const char* key, float value) {
	if (handle_ == nullptr || key == nullptr) return 0;
	uint32_t bits = 0;
	memcpy(&bits, &value, sizeof(bits));
	esp_err_t err = nvs_set_blob((nvs_handle_t)handle_, key, &bits, sizeof(bits));
	if (err != ESP_OK) return 0;
	// 与原 Preferences::putFloat 的返回值语义对齐：成功写入 sizeof(float) 字节
	return sizeof(float);
}

size_t NvsStore::putString(const char* key, const char* value) {
	if (handle_ == nullptr || key == nullptr || value == nullptr) return 0;
	esp_err_t err = nvs_set_str((nvs_handle_t)handle_, key, value);
	if (err != ESP_OK) return 0;
	return strlen(value);
}

size_t NvsStore::getString(const char* key, char* out, size_t maxLen) {
	if (out == nullptr || maxLen == 0) return 0;
	out[0] = '\0';
	if (handle_ == nullptr || key == nullptr) return 0;

	size_t len = maxLen;
	esp_err_t err = nvs_get_str((nvs_handle_t)handle_, key, out, &len);
	if (err != ESP_OK) {
		out[0] = '\0';
		return 0;
	}
	out[maxLen - 1] = '\0';
	return strlen(out);
}

const char* NvsStore::getString(const char* key, const char* defaultValue) {
	size_t n = getString(key, lastString_, sizeof(lastString_));
	if (n == 0) return defaultValue != nullptr ? defaultValue : "";
	return lastString_;
}

bool NvsStore::clear() {
	if (handle_ == nullptr) return false;
	esp_err_t err = nvs_erase_all((nvs_handle_t)handle_);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "nvs_erase_all failed: %s", esp_err_to_name(err));
		return false;
	}
	return nvs_commit((nvs_handle_t)handle_) == ESP_OK;
}
