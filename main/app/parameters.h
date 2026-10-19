// 参数存储（接口）

#pragma once

#include <stddef.h>

#include "hal/hal_nvs.h"

// 全局参数存储句柄（命名空间 "flix"），定义于 parameters.cpp
extern NvsStore storage;

void setupParameters();
int  parametersCount();
const char* getParameterName(int index);
float getParameter(int index);
float getParameter(const char* name);
bool  setParameter(const char* name, float value);
void syncParameters();     // 参数变更后延迟写入 Flash
void printParameters();
void resetParameters();    // 清空存储并重启
