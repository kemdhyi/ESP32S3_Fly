// 硬件抽象层：参数持久化（NVS）
// HAL: parameter storage on NVS
//
// 替代 Arduino 的 Preferences（其底层同样是 NVS，但命名空间与键的截断规则由
// Arduino 层实现）。这里提供与 Preferences 逐一对齐的接口，
// 使 parameters.cpp 的移植改动降到最低。
//
// ★ 兼容性关键：
//   * namespace 沿用原工程的 "flix"
//   * NVS 键长度上限为 15 字节（不含结尾 NUL），与 Preferences 的截断规则一致。
//     超长键会被截断，必须与原实现保持一致，否则升级固件后用户参数会丢失。

#pragma once

#include <stddef.h>
#include <stdint.h>

class NvsStore {
public:
	// 打开/创建命名空间。失败时按 IDF 标准做法擦除 NVS 分区后重试。
	// 等价于 Preferences::begin(name, false)
	bool begin(const char* name, bool readOnly = false);

	// 关闭句柄
	void end();

	bool isKey(const char* key);

	float  getFloat(const char* key, float defaultValue);
	size_t putFloat(const char* key, float value);

	// 变长字符串。返回实际写入长度（不含结尾 NUL）
	size_t putString(const char* key, const char* value);
	// 读入 out（最多 maxLen 字节，保证 NUL 结尾）
	size_t getString(const char* key, char* out, size_t maxLen);
	const char* getString(const char* key, const char* defaultValue);

	// 清空整个命名空间（等价于 Preferences::clear()）
	bool clear();

private:
	void* handle_ = nullptr;              // nvs_handle_t
	char  lastString_[256] = {};          // getString(key, const char*) 的返回缓冲
};

// NVS 键长度上限（与 NVS/Preferences 一致）
static constexpr size_t NVS_KEY_MAX_LEN = 15;
