// Wi-Fi 功能支持（接口）
//
// 替代 Arduino 的 WiFi.h / WiFiAP.h / WiFiUdp.h。

#pragma once

#include <stddef.h>
#include <stdint.h>

void setupWiFi();
void printWiFiInfo();

// 通过 CLI 配置 AP / STA 凭据（写入 NVS，重启后生效）
void configWiFi(bool ap, const char* ssid, const char* password);

// UDP 收发（MAVLink 使用）
void sendWiFi(const uint8_t* buf, int len);
int  receiveWiFi(uint8_t* buf, int len);

// 当前是否为 AP 模式（决定内置网页的提示地址）
bool wifiIsApMode();
