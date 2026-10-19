// 飞行控制（接口）

#pragma once

// 必须先引入板级配置：WEB_RC_ENABLED 由 hal/board.h 依 Kconfig 定义
#include "hal/board.h"

void control();
void interpretControls();
void controlAttitude();
void controlRates();
void controlTorque();
void desaturate(float& a, float& b, float& c, float& d);
const char* getModeName();

#if WEB_RC_ENABLED
void interpretWebRC();
#endif
