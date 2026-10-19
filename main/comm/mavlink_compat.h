// MAVLink 统一入口（兼容头）
//
// 移植说明（ESP-IDF）：
//   原 Arduino 工程使用 `#include <MAVLink.h>`（Arduino 库自带的聚合头）。
//   改为 ESP-IDF 后，MAVLink 由组件 espressif/mavlink 提供
//   （内部即 mavlink/c_library_v2）。
//
//   注意：c_library_v2 的方言头（common/common.h、standard/standard.h 等）
//   **不允许被直接包含**，它们会检查 #ifndef MAVLINK_H 并报
//   "Wrong include order"。必须包含某个方言目录下的 mavlink.h，
//   由它按正确顺序定义 MAVLINK_STX / MAVLINK_ENDIAN 等宏后再引入方言头。
//
//   本工程使用 all/mavlink.h（all 方言囊括 common/standard/minimal 等全部消息，
//   与原 Arduino 库的 MAVLink.h 覆盖面一致）。
//
//   头文件搜索路径由 main/CMakeLists.txt 注入（指向 c_library_v2 目录）。

#pragma once

#include "all/mavlink.h"
