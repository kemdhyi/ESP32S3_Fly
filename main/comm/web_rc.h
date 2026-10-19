// 网页遥控服务端（接口）

#pragma once

#include <stdint.h>

// 必须先引入板级配置：WIFI_ENABLED / WEB_RC_ENABLED 由 hal/board.h 依 Kconfig 定义
#include "hal/board.h"

#if WEB_RC_ENABLED

// ==================== 连接状态标志 ====================
extern bool webRCEnabled;        // Web RC 当前有有效连接（由 readWebRC() 每帧更新）
extern bool useWebRC;            // 当前正在使用 Web RC 控制（与 webRCEnabled 保持同步）
extern bool webRCUpdated;        // 收到过至少一次摇杆数据
extern bool webConsoleEnabled;   // Web 调试控制台开关
extern char webRCWarnMsg[64];    // 待发送给前端的警告，发送一次后自动清空

// ==================== 摇杆暂存值（供状态端点读取）====================
// 单位：油门 0~100（%），姿态轴 ±30（°），按钮为按下=1/松开=0
extern float webRCRoll;
extern float webRCPitch;
extern float webRCYaw;
extern float webRCThrottle;
extern uint16_t webRCButtons;
extern unsigned long webRCLastUpdate;   // 最后一次收包的 millis() 时间戳

// ==================== 灵敏度缩放 ====================
extern float webRCThrottleScale;
extern float webRCStickScale;
extern float webRCYawScale;

// ==================== 死区 ====================
extern float stickDeadzone;
extern float throttleDeadzone;

void setupWebRC();
void readWebRC();                     // 刷新连接状态标志
void processConsoleCommandQueue();    // 主循环中执行网页投递的 CLI 命令
void webLog(const char* msg);         // 写入网页控制台环形缓冲
void setWebRCWarn(const char* msg);
void clearWebRCWarn();
bool isWebRCEnabled();
bool isUsingWebRC();

#else  // WEB_RC_ENABLED == 0

static inline void setupWebRC() {}
static inline void readWebRC() {}
static inline void processConsoleCommandQueue() {}
static inline void webLog(const char* msg) { (void)msg; }
static inline void setWebRCWarn(const char* msg) { (void)msg; }
static inline void clearWebRCWarn() {}
static inline bool isWebRCEnabled() { return false; }
static inline bool isUsingWebRC() { return false; }

#endif  // WEB_RC_ENABLED
