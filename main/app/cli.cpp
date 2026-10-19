// Implementation of command line interface
//
// 移植说明（ESP-IDF）：
//   * Serial.print(buf)          -> hal_console_write()
//   * String command/arg0/arg1   -> 固定缓冲区 + strtok_r（消除堆分配与碎片）
//   * ESP.getChipModel()         -> esp_chip_info() + 型号表
//   * temperatureRead()          -> temperature_sensor_get_celsius()
//   * ESP.getFreeHeap()          -> esp_get_free_heap_size()
//   * ESP.restart()              -> esp_restart()
//   * uxTaskGetSystemState() 为 FreeRTOS 原生接口，原样可用。

#include "app/cli.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "app/control.h"
#include "app/globals.h"
#include "app/imu.h"
#include "app/log.h"
#include "app/math/lpf.h"
#include "app/math/pid.h"
#include "app/math/util.h"
#include "app/math/vector.h"
#include "app/motors.h"
#include "app/parameters.h"
#include "app/rc.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/board.h"
#include "hal/hal_console.h"
#include "hal/hal_time.h"

#include "driver/temperature_sensor.h"

#if WEB_RC_ENABLED
#include "comm/web_rc.h"
#endif

static const char* motd =
"CLI命令菜单，输入相应命令，回车后执行:\n"
"help - 帮助\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"p MOT_PIN_FL 14 - 参数设置示例，前左电机引脚为14\n"
"preset - 重置参数存储，设置参数后运行此命令\n"
"mfr, mfl, mrr, mrl - 测试马达 (马达不受算法影响运转，为了安全不要装桨叶！！！)\n"
"ca - 校准陀螺仪加速度计\n"
"ps - 显示pitch/roll/yaw姿态\n"
"cr - 校准RC遥控器\n"
"rc - 显示RC遥控数据\n"
"wifi - 显示WiFi信息\n"
"ap <ssid> <password> - 配置AP模式SSID和密码\n"
"sta <ssid> <password> - 配置STA客户端模式\n"
"raw/stab/acro/auto - 飞行模式设定\n"
"arm - 解锁无人机\n"
"disarm - 锁定无人机\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间信息\n"
"mot - 显示motor输出\n"
"sys - 显示系统info信息\n"
"log [dump] - 打印日志\n"
"reboot - 重启无人机\n"
"reset - 重置无人机\n";

// 命令解析结果（静态缓冲区，避免堆分配）
static char cmdLine[160];
static char* cmdName = nullptr;
static char* cmdArg0 = nullptr;
static char* cmdArg1 = nullptr;

void print(const char* format, ...) {
	// 固定栈缓冲 + vsnprintf 会静默截断超长内容（例如开机 motd），
	// 截断后连换行符都可能丢失，导致后续打印拼接到同一行。
	// 这里先用栈上缓冲区格式化，若所需长度超过缓冲，再按精确大小用堆内存重新格式化。
	char stackBuf[512];
	va_list args, argsCopy;
	va_start(args, format);
	va_copy(argsCopy, args);
	int needed = vsnprintf(stackBuf, sizeof(stackBuf), format, argsCopy);
	va_end(argsCopy);
	if (needed < 0) needed = 0; // 编码错误兜底

	char* buf = stackBuf;
	bool heapAllocated = false;
	if (needed >= (int)sizeof(stackBuf)) {
		buf = (char*)malloc(needed + 1);
		if (buf != nullptr) {
			vsnprintf(buf, needed + 1, format, args);
			heapAllocated = true;
		} else {
			buf = stackBuf; // 内存不足兜底：退回已截断的栈缓冲区内容
		}
	}
	va_end(args);

	hal_console_write_str(buf);
#if WIFI_ENABLED && CONFIG_CFDRONE_MAVLINK_ENABLED
	extern void mavlinkPrint(const char* str);
	mavlinkPrint(buf);
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf);
#endif
	if (heapAllocated) free(buf);
}

void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		// 长时间阻塞命令（ca/cr）期间持续刷新 IMU/姿态，否则打印信息会定格在旧值
		extern void readIMU();
		extern void step();
		extern void estimate();
		readIMU();
		step();
		estimate();
		handleInput();
#if WIFI_ENABLED && CONFIG_CFDRONE_MAVLINK_ENABLED
		extern void processMavlink();
		processMavlink();
#endif
#if WEB_RC_ENABLED
		readWebRC(); // 长时间命令期间保持 HTTP 服务器响应
#endif
		hal_delay_ms(50);
	}
}

// 简单按空格切分（等价于原 util.h 的 splitString，但无 String 堆分配）
static void splitCommand(char* str) {
	cmdName = cmdArg0 = cmdArg1 = nullptr;
	if (str == nullptr) return;

	while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
	if (*str == '\0') return;

	cmdName = str;

	char* p = str;
	while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
	if (*p) { *p = '\0'; p++; }

	while (*p == ' ' || *p == '\t') p++;
	if (*p == '\0') return;
	cmdArg0 = p;

	while (*p && *p != '\t' && *p != '\r' && *p != '\n') p++;
	if (*p) { *p = '\0'; p++; }

	while (*p == ' ' || *p == '\t') p++;
	if (*p == '\0') return;
	cmdArg1 = p;

	// 去掉末尾空白
	char* end = cmdArg1 + strlen(cmdArg1);
	while (end > cmdArg1 && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) *--end = '\0';
}

static const char* chipModelName(esp_chip_model_t model) {
	switch (model) {
		case CHIP_ESP32:   return "ESP32";
		case CHIP_ESP32S2: return "ESP32-S2";
		case CHIP_ESP32S3: return "ESP32-S3";
		case CHIP_ESP32C3: return "ESP32-C3";
		case CHIP_ESP32C2: return "ESP32-C2";
		case CHIP_ESP32C6: return "ESP32-C6";
		case CHIP_ESP32H2: return "ESP32-H2";
		default:           return "UNKNOWN";
	}
}

static void printSysInfo() {
	esp_chip_info_t info = {};
	esp_chip_info(&info);
	print("Chip: %s rev v%d.%d, %d core(s)\n",
	      chipModelName(info.model), info.revision / 100, info.revision % 100,
	      (int)info.cores);

	// 温度传感器：运行时安装探测，不支持的老芯片会失败并跳过
	{
		temperature_sensor_handle_t tsens = nullptr;
		temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
		if (temperature_sensor_install(&cfg, &tsens) == ESP_OK) {
			if (temperature_sensor_enable(tsens) == ESP_OK) {
				float celsius = 0;
				if (temperature_sensor_get_celsius(tsens, &celsius) == ESP_OK) {
					print("Temperature: %.1f °C\n", celsius);
				}
				temperature_sensor_disable(tsens);
			}
			temperature_sensor_uninstall(tsens);
		}
	}

	print("Free heap: %u\n", (unsigned)esp_get_free_heap_size());
	print("Uptime: %.1f s\n", (double)(esp_timer_get_time() / 1000000));

	// 任务表
	print("Num  Task                Stack  Prio  Core  CPU%%\n");
	int taskCount = (int)uxTaskGetNumberOfTasks();
	if (taskCount <= 0) return;

	TaskStatus_t* systemState = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * taskCount);
	if (systemState == nullptr) {
		print("(task list unavailable: out of memory)\n");
		return;
	}
	uint32_t totalRunTime = 0;
	taskCount = (int)uxTaskGetSystemState(systemState, (UBaseType_t)taskCount, &totalRunTime);
	for (int i = 0; i < taskCount; i++) {
		char core[16];   // 预留足够空间：xCoreID 最大为 "2147483647"（11 字符 + NUL）
#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
		if (systemState[i].xCoreID == tskNO_AFFINITY) {
			snprintf(core, sizeof(core), "*");
		} else {
			snprintf(core, sizeof(core), "%d", (int)systemState[i].xCoreID);
		}
#else
		snprintf(core, sizeof(core), "?");
#endif
		int cpuPercentage = 0;
		if (totalRunTime > 0) {
			cpuPercentage = (int)(systemState[i].ulRunTimeCounter / (totalRunTime / 100));
		}
		print("%-5d%-20s%-7d%-6d%-6s%d\n",
		      (int)systemState[i].xTaskNumber, systemState[i].pcTaskName,
		      (int)systemState[i].usStackHighWaterMark, (int)systemState[i].uxCurrentPriority,
		      core, cpuPercentage);
	}
	free(systemState);
}

void doCommand(const char* str, bool echo) {
	if (str == nullptr) return;

	// 拷贝到可写缓冲区后按空格切分
	strncpy(cmdLine, str, sizeof(cmdLine) - 1);
	cmdLine[sizeof(cmdLine) - 1] = '\0';
	splitCommand(cmdLine);

	if (cmdName == nullptr || cmdName[0] == '\0') return;

	// 回显
	if (echo) {
		print("> %s\n", str);
	}

	// 转小写
	for (char* p = cmdName; *p; p++) *p = (char)tolower((unsigned char)*p);

	const char* arg0 = cmdArg0 ? cmdArg0 : "";
	const char* arg1 = cmdArg1 ? cmdArg1 : "";
	const bool hasArg0 = cmdArg0 != nullptr && cmdArg0[0] != '\0';
	const bool hasArg1 = cmdArg1 != nullptr && cmdArg1[0] != '\0';

	// 执行命令
	if (strcmp(cmdName, "help") == 0 || strcmp(cmdName, "motd") == 0) {
		print("%s\n", motd);
	} else if (strcmp(cmdName, "p") == 0 && !hasArg0) {
		printParameters();
	} else if (strcmp(cmdName, "p") == 0 && hasArg0 && !hasArg1) {
		print("%s = %g\n", arg0, getParameter(arg0));
	} else if (strcmp(cmdName, "p") == 0) {
		bool success = setParameter(arg0, strtof(arg1, nullptr));
		if (success) {
			print("%s = %g\n", arg0, getParameter(arg0));
		} else {
			print("Parameter not found: %s\n", arg0);
		}
	} else if (strcmp(cmdName, "preset") == 0) {
		resetParameters();
	} else if (strcmp(cmdName, "time") == 0) {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (strcmp(cmdName, "ps") == 0) {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (strcmp(cmdName, "psq") == 0) {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (strcmp(cmdName, "imu") == 0) {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (strcmp(cmdName, "arm") == 0) {
		if (!imuOK) { print("IMU故障，禁止解锁！\n"); }
		else armed = true;
	} else if (strcmp(cmdName, "disarm") == 0) {
		armed = false;
	} else if (strcmp(cmdName, "raw") == 0) {
		mode = RAW;
	} else if (strcmp(cmdName, "stab") == 0) {
		mode = STAB;
	} else if (strcmp(cmdName, "acro") == 0) {
		mode = ACRO;
	} else if (strcmp(cmdName, "auto") == 0) {
		mode = AUTO;
	} else if (strcmp(cmdName, "rc") == 0) {
		print("channels: ");
		for (int i = 0; i < 16; i++) {
			print("%u ", channels[i]);
		}
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("time: %.1f\n", controlTime);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
	} else if (strcmp(cmdName, "wifi") == 0) {
#if WIFI_ENABLED
		extern void printWiFiInfo();
		printWiFiInfo();
#endif
	} else if (strcmp(cmdName, "ap") == 0) {
#if WIFI_ENABLED
		extern void configWiFi(bool ap, const char* ssid, const char* password);
		configWiFi(true, arg0, arg1);
#endif
	} else if (strcmp(cmdName, "sta") == 0) {
#if WIFI_ENABLED
		extern void configWiFi(bool ap, const char* ssid, const char* password);
		configWiFi(false, arg0, arg1);
#endif
	} else if (strcmp(cmdName, "mot") == 0) {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (strcmp(cmdName, "log") == 0) {
		printLogHeader();
		if (strcmp(arg0, "dump") == 0) printLogData();
	} else if (strcmp(cmdName, "cr") == 0) {
		calibrateRC();
	} else if (strcmp(cmdName, "ca") == 0) {
		calibrateAccel();
	} else if (strcmp(cmdName, "mfr") == 0) {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (strcmp(cmdName, "mfl") == 0) {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (strcmp(cmdName, "mrr") == 0) {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (strcmp(cmdName, "mrl") == 0) {
		testMotor(MOTOR_REAR_LEFT);
	} else if (strcmp(cmdName, "sys") == 0) {
		printSysInfo();
	} else if (strcmp(cmdName, "reset") == 0) {
		attitude = Quaternion();
		gyroBiasFilter.reset();
	} else if (strcmp(cmdName, "reboot") == 0) {
		esp_restart();
	} else {
		print("Invalid command: %s\n", cmdName);
	}
}

void handleInput() {
	static bool showMotd = true;
	static char input[160];
	static size_t inputLen = 0;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	uint8_t c;
	while (hal_console_available() && hal_console_read_byte(&c)) {
		if (c == '\n') {
			input[inputLen] = '\0';
			doCommand(input);
			inputLen = 0;
		} else if (c == '\r') {
			// 忽略 CR（兼容 CRLF 终端）
		} else if (inputLen < sizeof(input) - 1) {
			input[inputLen++] = (char)c;
		}
	}
}
