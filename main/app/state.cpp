// 全局状态定义
//
// 集中定义 app/globals.h 中声明为 extern 的控制输入变量。
//
// 原 Arduino 工程中这些变量定义在 CF-Drone.ino 顶部；
// 移植后按职责归类，把「飞手输入指令」放在本文件，
// 其余全局量分别由各组件的属主文件定义：
//   传感器量   -> app/imu.cpp
//   飞行与控制 -> app/control.cpp
//   遥控与时间 -> app/rc.cpp
//   电机输出   -> app/motors.cpp
//   日志量     -> app/log.cpp
//
// 使用方式完全一致：任何包含 app/globals.h 的模块都可直接读写这些变量，
// 与各协议适配层（SBUS / CRSF / MAVLink / Web RC）共用同一条控制路径。

#include "app/globals.h"

// 飞手输入指令，范围 [-1, 1]；controlMode 为模式通道归一化值
// 由 app/rc.cpp 的 normalizeRC()、comm/web_rc.cpp 的 setWebRCInput()、
// comm/mavlink.cpp 的 MANUAL_CONTROL 处理共同写入。
float controlRoll     = NAN;
float controlPitch    = NAN;
float controlYaw      = NAN;
float controlThrottle = NAN;
float controlMode     = NAN;
