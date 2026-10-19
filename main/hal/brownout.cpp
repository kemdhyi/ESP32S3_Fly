// 硬件抽象层：Brownout（低压复位）禁用
// 替代原 Arduino 工程 util.h 中的 disableBrownOut()
//
// 电机启动瞬间的电流冲击会拉低供电电压，触发 ESP32-S3 的 brownout reset。
// 飞控场景下宁可继续运行也不要在空中复位，因此显式关闭该检测。
//
// IDF 提供 esp_brownout_disable()（声明于 esp_private/brownout.h），
// 与原 util.h 中 ESP32-S3 分支的用法一致。

#include "esp_private/brownout.h"

#include "app/math/util.h"
#include "esp_log.h"

static const char* TAG = "brownout";

void disableBrownOut() {
	esp_brownout_disable();
	ESP_LOGI(TAG, "brownout reset detection disabled");
}
