// 飞控全局状态
//
// Arduino 版通过各 .ino 文件顶部的 `extern` 声明共享全局变量（IDE 会把它们拼成
// 单个翻译单元）。改成标准 C++ 工程后，统一在本文件集中声明，各模块 include 即可。

#pragma once

#include <stdint.h>

#include "app/math/lpf.h"
#include "app/math/pid.h"
#include "app/math/quaternion.h"
#include "app/math/util.h"
#include "app/math/vector.h"
#include "hal/board.h"

// ============================================================
// 时间（app/time.cpp）
// ============================================================
extern float dt;         // 与上一步进的时间差（秒）
extern float loopRate;   // 主循环频率（Hz），1 秒窗口统计

// ============================================================
// 飞手输入指令（app/rc.cpp / app/control.cpp / comm/web_rc.cpp 共同写入）
// 范围 [-1, 1]；controlMode 为模式通道归一化值
// ============================================================
extern float controlRoll, controlPitch, controlYaw, controlThrottle;
extern float controlMode;

// ============================================================
// 传感器数据（app/imu.cpp）
// ============================================================
extern Vector gyro;              // 陀螺仪原始数据（rad/s，已扣偏置）
extern Vector acc;               // 加速度计数据（m/s²，已做 bias/scale 校正）
extern Vector rates;             // 滤波后的角速度（rad/s）
extern Quaternion attitude;      // 估计姿态（四元数）
extern bool landed;              // 是否落地静止
extern bool imuOK;               // IMU 初始化是否成功；false 时禁止解锁
extern Vector accBias;
extern Vector accScale;
extern Vector gyroBias;
extern Vector imuRotation;       // IMU 安装方向（欧拉角，rad）
extern LowPassFilter<Vector> gyroBiasFilter;

// ============================================================
// 飞行状态与控制（app/control.cpp）
// ============================================================
extern const int RAW, ACRO, STAB, ALTHOLD, AUTO;
extern int mode;
extern bool armed;
extern int flightModes[3];

extern PID rollRatePID, pitchRatePID, yawRatePID;
extern PID rollPID, pitchPID, yawPID;
extern Vector maxRate;
extern float tiltMax;

extern Quaternion attitudeTarget;
extern Vector ratesTarget;
extern Vector ratesExtra;
extern Vector torqueTarget;
extern float thrustTarget;

extern float motThrMin;
extern float motThrMax;
extern float trimRoll;
extern float trimPitch;

// ============================================================
// 电机输出（app/motors.cpp）
// ============================================================
extern float motors[4];          // 归一化推力 [0..1]
extern int motorPins[4];         // RL, RR, FR, FL
extern int pwmFrequency;
extern int pwmResolution;
extern int pwmStop;
extern int pwmMin;
extern int pwmMax;

extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;

// ============================================================
// 遥控接收机（app/rc.cpp）
// ============================================================
extern uint16_t channels[16];
extern float controlTime;        // 最后一次收到有效遥控数据的时间（秒），0=从未收到
extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int rcProtocol;           // 0=SBUS, 1=CRSF(ELRS)
extern int rcRxPin;
extern int rcTxPin;
extern int rcBaud;

// ============================================================
// 电池与失效保护（app/battery.cpp / app/safety.cpp）
// ============================================================
extern float batteryVoltage;
extern bool isInverted;
extern float rcLossTimeout;
extern float descendTime;

#define VBAT_WARN_THRESHOLD      3.5f   // L1：空载/怠速预警门限（V）
#define VBAT_LOW_THRESHOLD       3.3f   // L2：飞行中低电告警门限（V）
#define VBAT_CRITICAL_THRESHOLD  3.0f   // L3：飞行中危急门限，自动降落（V）
#define VBAT_ABSENT_THRESHOLD    0.5f   // 低于此值视为未接电池，忽略所有告警
#define BATTERY_FLYING_THRUST_MIN    0.15f
#define BATTERY_ACTION_DEBOUNCE_TIME 0.9f
#define ALTHOLD_HOVER_THRUST         0.5f
#define ARM_THROTTLE_LIMIT           0.05f

// ============================================================
// 姿态估计（app/estimate.cpp）
// ============================================================
extern float accWeight;
extern float levelWeight;
extern float levelMaxTilt;
extern float levelGateThreshold;
extern float levelBiasGain;
extern Vector levelGyroBias;
extern LowPassFilter<Vector> ratesFilter;

// ============================================================
// 通信配置（定义于 app/comm_config.cpp —— 始终参与编译，见该文件说明）
// ============================================================
extern int wifiMode;             // 0=关闭，1=AP，2=STA
extern int udpLocalPort;
extern int udpRemotePort;
extern int mavlinkSysId;
extern float telemetrySlowRate;
extern float telemetryFastRate;
extern bool mavlinkConnected;
extern Rate telemetrySlow;
extern Rate telemetryFast;

// ============================================================
// 日志（app/log.cpp）
// ============================================================
extern Vector attitudeEuler;
extern Vector attitudeTargetEuler;

// 日志缓冲区的编译期尺寸（与原 LOG_DURATION * LOG_RATE 一致）
static constexpr int LOG_RATE = 100;
static constexpr int LOG_SIZE = BOARD_LOG_DURATION * LOG_RATE;
static constexpr int LOG_COLUMNS = 14;

// 定义于 app/log.cpp，按行优先展开：logBuffer[i * LOG_COLUMNS + j]
extern float logBuffer[LOG_SIZE * LOG_COLUMNS];
