// 通信配置的全局定义（始终参与编译）
//
// 为什么单独放一个文件：
//   app/parameters.cpp 的参数表无条件引用 wifiMode / udpLocalPort /
//   udpRemotePort / mavlinkSysId / telemetry*Rate / mavlinkConnected。
//   若把这些变量定义在 comm/wifi.cpp 或 comm/mavlink.cpp 内部，
//   一旦在 menuconfig 中关闭 WiFi / Web RC / MAVLink，
//   对应源文件整体被条件编译排除 → 参数表出现 undefined reference，
//   构建失败。
//
//   因此这里无条件定义它们，comm/wifi.cpp 与 comm/mavlink.cpp 改为 extern 引用。
//   这样任意功能开关组合都能正常构建——关闭某功能只是不再使用这些参数，
//   而不是让工程编译不过。

#include "app/globals.h"

#include "hal/board.h"

// ---- WiFi / UDP（原 wifi.ino 中的定义）----
// wifiMode 取值：0=关闭（W_DISABLED），1=AP（W_AP），2=STA（W_STA）
int wifiMode      = 1;
int udpLocalPort  = 14550;
int udpRemotePort = 14550;

// ---- MAVLink（原 mavlink.ino 中的定义）----
// mavlinkSysId 的初值取自 Kconfig；首次上电后由参数 MAV_SYS_ID 持久化覆盖。
int   mavlinkSysId        = CONFIG_CFDRONE_MAVLINK_SYS_ID;
float telemetrySlowRate   = 2.0f;
float telemetryFastRate   = BOARD_MAVLINK_TELEM_FAST_HZ;
bool  mavlinkConnected    = false;
// 遥测发送频率限制器（供 processMavlink 使用；由上面两个 rate 变量驱动）
Rate telemetrySlow(2.0f);
Rate telemetryFast(BOARD_MAVLINK_TELEM_FAST_HZ);
