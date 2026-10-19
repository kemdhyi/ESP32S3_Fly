// 板载 LED（接口）

#pragma once

void setupLED();
void setLED(bool on);
void blinkLED();
bool batteryAlertActive();
bool ledAlertActive();
void updateLED();
