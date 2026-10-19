// 板级硬件配置 —— ESP32-S3 专用
// Board-level hardware configuration (ESP32-S3 only)
//
// 说明：本文件由原 Arduino 工程根目录的 board_config.h 精简而来。
// 原文件通过条件编译同时支持 ESP32 / ESP32-C3 / ESP32-S3；
// 本次移植目标芯片为 ESP32-S3，其余分支已删除。
//
// 功能开关（WiFi / Web RC / MAVLink）已迁移到 Kconfig，
// 见 main/Kconfig.projbuild，可通过 `idf.py menuconfig` 调整。
// 为保持原 app 代码中的 `#if WIFI_ENABLED` 写法可用，此处提供映射。

#pragma once

#include <sdkconfig.h>

// ============================================================
// ---- 电机引脚（数组顺序：RL, RR, FR, FL，与 motors.cpp 中常量定义一致）----
// GPIO4-7，远离 FSPI 区（GPIO10-13），无 Strapping 约束
// ============================================================
#define BOARD_MOTOR_PINS   {4, 5, 6, 7}    // RL=GPIO4, RR=GPIO5, FR=GPIO6, FL=GPIO7

// ============================================================
// ---- 电池 ADC ----
// GPIO1（ADC1_CH0，S3 的 ADC1 范围为 GPIO1-10）
// ============================================================
#define BOARD_VBAT_ADC_PIN 1

// ============================================================
// ---- RC 串口（CRSF/ELRS 默认；SBUS 可通过参数 RC_PROTOCOL=0 切换）----
// GPIO8，与 FSPI/UART0 无冲突
// ============================================================
#define BOARD_RC_UART_NUM  1               // 使用 UART1（UART0 留给控制台）
#define BOARD_RC_RX_PIN    8
#define BOARD_RC_TX_PIN    -1              // 遥测回传预留，-1=暂不启用
#define BOARD_RC_PROTOCOL  1               // 0=SBUS, 1=CRSF(ELRS)
#define BOARD_RC_BAUD      420000          // CRSF 标准波特率默认420000；SBUS为100000

// ============================================================
// ---- IMU SPI：使用 ESP32-S3 FSPI 默认管脚 ----
// SCK=12（默认），MISO=13（默认），MOSI=11（默认），CS=10（默认 SS）
// ============================================================
#define BOARD_SPI_HOST     0               // SPI2_HOST (FSPI)
#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10

// ============================================================
// ---- I2C（预留给扩展板传感器）----
// ============================================================
#define BOARD_I2C_SDA          35          // GPIO35
#define BOARD_I2C_SCL          36          // GPIO36

// ============================================================
// ---- LED：新 PCB 接 GPIO2 普通 LED，高电平点亮 ----
// ============================================================
#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      2
#define BOARD_LED_INVERTED 0

// ============================================================
// ---- 性能与资源配置（S3 标准）----
// ============================================================
#define BOARD_VBAT_ADC_SAMPLES       16   // ADC 采样次数
#define BOARD_LOG_DURATION           8    // 日志缓冲秒数
#define BOARD_CONSOLE_LINES          50   // 控制台行数
#define BOARD_CONSOLE_LINE_LEN       240  // 每行字符数
#define BOARD_MAVLINK_TELEM_FAST_HZ  10   // MAVLink 快速遥测频率

// ============================================================
// ---- 扩展板传感器 ----
// ============================================================
#define EXPANSION_BOARD_ENABLED      1    // 1 = 启用扩展板传感器探测（BMP388/VL53L1X/QMC5883L/PMW3901）
#define BOARD_VL53_XSHUT_PIN   37         // VL53L1X XSHUT
#define BOARD_PMW_CS_PIN       38         // PMW3901 CS

// ============================================================
// ---- 功能开关：Kconfig 优先，缺省打开 ----
// 这样无论通过 menuconfig 还是直接编译，`#if WIFI_ENABLED` 都能工作。
// ============================================================
#if defined(CONFIG_CFDRONE_WIFI_ENABLED)
#define WIFI_ENABLED     CONFIG_CFDRONE_WIFI_ENABLED
#else
#define WIFI_ENABLED     1
#endif

#if defined(CONFIG_CFDRONE_WEB_RC_ENABLED)
#define WEB_RC_ENABLED   CONFIG_CFDRONE_WEB_RC_ENABLED
#else
#define WEB_RC_ENABLED   1
#endif
