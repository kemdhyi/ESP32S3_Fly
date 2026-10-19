// 遥控接收机（接口）

#pragma once

#include <stdint.h>

void setupRC();
bool readRC();
void normalizeRC();
void calibrateRC();
void calibrateRCChannel(float* channel, uint16_t in[16], uint16_t out[16], const char* str);
void printRCCalibration();
