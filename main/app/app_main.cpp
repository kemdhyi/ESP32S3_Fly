// app_main —— 飞控入口
//
// 由原 Arduino 工程的 CF-Drone.ino 改造而来：
//   setup() -> app_main() 中的初始化序列
//   loop()  -> fcTask 飞控主循环任务（默认绑定 core 1）
//
// 相对 Arduino 版的结构性改进：
//   * 飞控主循环是独立的 FreeRTOS 任务并固定在 core 1；
//     原先在 loop() 中轮询的 HTTP 服务器改为 core 0 的独立任务，
//     彻底隔离网络处理对姿态环时序的影响。
//   * app_main 初始化完成后自行删除，释放约 8KB 栈空间。

#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "app/battery.h"
#include "app/cli.h"
#include "app/control.h"
#include "app/estimate.h"
#include "app/globals.h"
#include "app/imu.h"
#include "app/led.h"
#include "app/log.h"
#include "app/math/util.h"
#include "app/motors.h"
#include "app/parameters.h"
#include "app/rc.h"
#include "app/safety.h"
#include "app/time.h"
#include "hal/board.h"
#include "hal/hal_adc.h"
#include "hal/hal_console.h"
#include "hal/hal_spi.h"
#include "hal/hal_time.h"

#if WIFI_ENABLED
#include "comm/wifi.h"
#endif
#if WEB_RC_ENABLED
#include "comm/web_rc.h"
#endif
#if CONFIG_CFDRONE_MAVLINK_ENABLED
void processMavlink();
#endif

static const char* TAG = "cf_drone";

// 品牌标识输出（LICENSE 附加条款第二条要求固件启动输出保留品牌名称）
static constexpr const char* BOOT_BANNER =
	"琛光科技 CF-Drone Flight Controller\n";

static void printBanner() {
	hal_console_write_str(BOOT_BANNER);
	hal_console_write_str("程序开始初始化！\n");
}

// ============================================================
// 飞控主循环（对应原 Arduino 的 loop()）
// ============================================================
static void fcTask(void* arg) {
	(void)arg;

	for (;;) {
		readIMU();               // 读取 IMU（阻塞等待采样节拍），完成校准与坐标旋转
		step();                  // 计算步进时间 t / dt 与循环频率
		readRC();                // 读取遥控接收机输入
#if WEB_RC_ENABLED
		readWebRC();             // 刷新 Web 遥控连接状态
		processConsoleCommandQueue(); // 执行网页投递的 CLI 命令（在飞控上下文中）
#endif
		estimate();              // 姿态与状态估计（互补滤波）
		updateBatteryVoltage();  // 电池电压采样与低电压保护判断
		control();               // 飞控核心：姿态环 PID 解算
		sendMotors();            // 输出电机控制量
		handleInput();           // 处理串口/Web 控制台输入命令
#if WIFI_ENABLED && CONFIG_CFDRONE_MAVLINK_ENABLED
		processMavlink();        // 处理 MAVLink 通信
#endif
		logData();               // 记录飞行日志
		syncParameters();        // 参数变更后延迟写入 Flash
		updateLED();             // 根据飞行状态刷新 LED

		// 让出 CPU 给同核的低优先级任务（本任务由 IMU 采样节拍天然限频）
		vTaskDelay(1);
	}
}

extern "C" void app_main(void) {
	// ---- 基础外设 ----
	hal_console_init(CONFIG_CFDRONE_CONSOLE_BAUD);
	hal_time_init();

	printBanner();
	ESP_LOGI(TAG, "CF-Drone ESP-IDF port, IDF %s, target %s",
	         esp_get_idf_version(), CONFIG_IDF_TARGET);

	// 禁用低压复位检测：电机启动瞬间电压跌落不应导致空中复位
	disableBrownOut();

	// ---- 参数（从 NVS 加载，未存储时使用默认值）----
	setupParameters();

	// ---- 板载硬件 ----
	setupLED();
	hal_battery_adc_init(BOARD_VBAT_ADC_PIN);
	setupMotors();
	setLED(true);                 // 点亮 LED，提示正在初始化

	// ---- 通信 ----
#if WIFI_ENABLED
	setupWiFi();
#endif
#if WEB_RC_ENABLED
	setupWebRC();
#endif

	// ---- 传感器与遥控 ----
	setupIMU();
	setupRC();

	setLED(false);                // 熄灭 LED，提示初始化完成
	hal_console_write_str("程序初始化完成！\n");
	hal_console_write_str("================================\n");

	// ---- 启动飞控主循环 ----
	BaseType_t ok = xTaskCreatePinnedToCore(
		fcTask, "fc_loop",
		CONFIG_CFDRONE_FC_TASK_STACK,
		nullptr,
		CONFIG_CFDRONE_FC_TASK_PRIORITY,
		nullptr,
		CONFIG_CFDRONE_FC_TASK_CORE);

	if (ok != pdPASS) {
		ESP_LOGE(TAG, "无法创建飞控主循环任务！");
		hal_console_write_str("错误：飞控主循环任务创建失败\n");
		return;
	}

	ESP_LOGI(TAG, "flight loop started on core %d (prio %d, stack %d)",
	         CONFIG_CFDRONE_FC_TASK_CORE, CONFIG_CFDRONE_FC_TASK_PRIORITY,
	         CONFIG_CFDRONE_FC_TASK_STACK);

	// app_main 完成使命，自行退出释放栈空间
	vTaskDelete(nullptr);
}
