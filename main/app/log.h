// 飞行日志（接口）

#pragma once

void prepareLogData();
void logData();
void printLogHeader();
void printLogData();

// 日志行数（mavlink 传输用）
int logRowCount();
