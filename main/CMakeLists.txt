# CF-Drone 飞控主组件
#
# 源文件采用显式清单（而非 GLOB），便于控制编译顺序与排查问题。
# 新增文件时请同步更新本文件。

set(CFDRONE_APP_SRCS
    "app/app_main.cpp"
    "app/state.cpp"
    "app/comm_config.cpp"
    "app/time.cpp"
    "app/parameters.cpp"
    "app/control.cpp"
    "app/estimate.cpp"
    "app/safety.cpp"
    "app/imu.cpp"
    "app/motors.cpp"
    "app/battery.cpp"
    "app/led.cpp"
    "app/rc.cpp"
    "app/log.cpp"
    "app/cli.cpp"
)

set(CFDRONE_HAL_SRCS
    "hal/hal_time.cpp"
    "hal/hal_gpio.cpp"
    "hal/hal_pwm.cpp"
    "hal/hal_adc.cpp"
    "hal/hal_spi.cpp"
    "hal/hal_uart.cpp"
    "hal/hal_console.cpp"
    "hal/hal_i2c.cpp"
    "hal/hal_nvs.cpp"
    "hal/brownout.cpp"
)

set(CFDRONE_DRIVER_SRCS
    "drivers/invensense_imu.cpp"
    "drivers/MPU9250.cpp"
    "drivers/SBUS.cpp"
)

set(CFDRONE_COMM_SRCS
    "comm/wifi.cpp"
    "comm/http_server.cpp"
    "comm/web_rc.cpp"
    "comm/mavlink.cpp"
)

set(CFDRONE_SRCS
    ${CFDRONE_APP_SRCS}
    ${CFDRONE_HAL_SRCS}
    ${CFDRONE_DRIVER_SRCS}
    ${CFDRONE_COMM_SRCS}
)

# 过滤掉尚未创建的文件，便于分阶段开发
set(CFDRONE_EXISTING_SRCS "")
foreach(src ${CFDRONE_SRCS})
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${src}")
        list(APPEND CFDRONE_EXISTING_SRCS "${src}")
    else()
        message(STATUS "CF-Drone: skipping missing source ${src}")
    endif()
endforeach()

# MAVLink 由 ESP 组件管理器提供（见 main/idf_component.yml：
# espressif/mavlink，内部即 mavlink/c_library_v2），随构建自动下载到
# managed_components/。若目录尚未就绪则自动跳过 MAVLink 源文件，
# 保证飞控核心始终可以构建。
set(CFDRONE_EXTRA_REQUIRES "")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__mavlink/CMakeLists.txt")
    list(APPEND CFDRONE_EXTRA_REQUIRES mavlink)
    message(STATUS "CF-Drone: MAVLink component found, enabling telemetry")
else()
    message(STATUS "CF-Drone: MAVLink component not found, telemetry disabled")
endif()

idf_component_register(
    SRCS ${CFDRONE_EXISTING_SRCS}
    INCLUDE_DIRS
        "."
        "app"
        "app/math"
        "hal"
        "comm"
        "drivers"
    REQUIRES
        driver
        esp_timer
        esp_wifi
        esp_netif
        esp_adc
        nvs_flash
        ${CFDRONE_EXTRA_REQUIRES}
)

# ---- 编译选项 ----
# 注：-Wno-overloaded-virtual 用于 Bolder Flight IMU 驱动的既有设计
#     （IMUBase::setupInterrupt(int) 与虚函数 setupInterrupt() 同名重载），
#     原 Arduino 工程同样存在该模式，此处不做行为改动。
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -Wno-sign-compare
    -Wno-overloaded-virtual
)

# ---- 预处理宏 ----
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MAVLINK_COMM_NUM_BUFFERS=2
)

# ---- MAVLink 头文件路径 ----
# espressif/mavlink 组件把 mavlink/c_library_v2 放在组件的子目录 c_library_v2 下，
# 而组件本身只把组件根目录加入 include 路径。这里显式补上子目录，
# 使 `#include "mavlink_helpers.h"` 与 `#include "common/common.h"` 均可解析。
set(CFDRONE_MAVLINK_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__mavlink/c_library_v2")
if(EXISTS "${CFDRONE_MAVLINK_DIR}/mavlink_helpers.h")
    target_include_directories(${COMPONENT_LIB} PRIVATE "${CFDRONE_MAVLINK_DIR}")
    message(STATUS "CF-Drone: MAVLink headers at ${CFDRONE_MAVLINK_DIR}")
else()
    message(STATUS "CF-Drone: MAVLink headers not found, MAVLink sources will fail to compile")
endif()
