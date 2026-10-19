// 命令行接口（接口）

#pragma once

// printf 风格输出。同时分发到：调试串口、MAVLink SERIAL_CONTROL、Web 控制台。
void print(const char* format, ...);

// 阻塞式等待 duration 秒，期间持续刷新 IMU/姿态/通信，
// 供 ca（加速度计标定）与 cr（遥控器标定）这类长流程使用。
void pause(float duration);

// 执行一条 CLI 命令（echo=true 时回显）
void doCommand(const char* str, bool echo = false);

// 主循环调用：从串口读取并解析 CLI 输入
void handleInput();
