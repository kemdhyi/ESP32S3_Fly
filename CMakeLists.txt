cmake_minimum_required(VERSION 3.16)

# CF-Drone 飞控 —— 纯 ESP-IDF 版
# 原 Arduino 工程位于上一级目录，保持不变以便对比。

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(cf_drone)
