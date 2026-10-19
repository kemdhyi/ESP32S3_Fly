# CF-Drone → 纯 ESP-IDF 迁移说明

本目录是把工作区根目录的原 Arduino 工程（`.ino` / `drivers/`）完整移植到
**原生 ESP-IDF（不依赖 Arduino 框架）** 后的新工程。

- 目标芯片：**ESP32-S3**（原工程支持 ESP32 / ESP32-C3 / ESP32-S3，本次只保留 S3）
- ESP-IDF 版本：**v5.4.4**（ESP-IDF v5.4.4 @ `296b6eab`）
- 原 Arduino 工程**未做任何改动**，仍在 `../`（根目录），便于逐文件对照

## 构建状态（已验证）

```
$ idf.py build
...
Successfully created esp32s3 image.
Generated .../build/cf_drone.bin
Project build complete.
```

| 指标 | 数值 |
|---|---|
| 编译告警（本工程源码，排除第三方组件） | **0** |
| `cf_drone.bin` | 901,152 字节（0xDC020） |
| factory 分区占用 | 28.7%（0x300000 = 3 MB 中已用 0xDC020） |
| Flash Code | 617,632 字节 |
| Flash Data | 163,956 字节 |
| DIRAM | 187,663 / 341,760 字节（54.9%） |
| **IRAM** | 16,383 / 16,384 字节（**99.99%**） |

> ⚠️ **IRAM 已几乎用满（仅剩 1 字节）**。当前构建通过，但后续若新增
> 需要 IRAM 的代码（如 `IRAM_ATTR` 函数、ISR、更多 WiFi 相关组件），
> 可能出现链接失败。届时可通过 menuconfig 的
> `Compiler options → Optimization Level` 或减少组件来释放空间。

---

## 1. 目录结构

```
idf/
├── CMakeLists.txt              顶层工程定义
├── sdkconfig.defaults          默认配置（对应原 board_config.h 的板级常量）
├── partitions.csv              分区表（NVS + factory，不预留 OTA）
├── idf_env.ps1 / idf_env.bat   本机 ESP-IDF 环境激活包装脚本（见 §5）
├── main/
│   ├── CMakeLists.txt          源文件清单 + 编译选项 + MAVLink 头路径
│   ├── Kconfig.projbuild       menuconfig 选项（WiFi / Web RC / MAVLink / 任务亲和性）
│   ├── idf_component.yml       组件依赖：espressif/mavlink
│   ├── app/                    应用层（飞控逻辑，与原 .ino 一一对应）
│   │   ├── math/               vector / quaternion / pid / lpf / util（纯头文件）
│   │   ├── app_main.cpp        ← CF-Drone.ino（setup/loop → app_main + fcTask）
│   │   ├── state.cpp           飞手输入指令的全局定义
│   │   ├── comm_config.cpp     通信配置的全局定义（始终编译，见 §4.5）
│   │   ├── time.cpp            ← time.ino
│   │   ├── parameters.cpp      ← parameters.ino
│   │   ├── control.cpp         ← control.ino
│   │   ├── estimate.cpp        ← estimate.ino
│   │   ├── safety.cpp          ← safety.ino
│   │   ├── imu.cpp             ← imu.ino
│   │   ├── motors.cpp          ← motors.ino
│   │   ├── battery.cpp         ← battery.ino
│   │   ├── led.cpp             ← led.ino
│   │   ├── rc.cpp              ← rc.ino
│   │   ├── log.cpp             ← log.ino
│   │   ├── cli.cpp             ← cli.ino
│   │   └── globals.h           原各 .ino 顶部 extern 声明的集中化
│   ├── hal/                    硬件抽象层（新增，替代 Arduino API）
│   │   ├── board.h             ← board_config.h（精简为 S3）
│   │   ├── hal_time / hal_gpio / hal_pwm / hal_adc
│   │   ├── hal_spi / hal_i2c / hal_uart / hal_console / hal_nvs
│   │   └── brownout.cpp        ← util.h 的 disableBrownOut()
│   ├── drivers/                Bolder Flight 驱动（MIT），已改接 ESP-IDF HAL
│   │   ├── IMU.h               hw_timer_t → esp_timer
│   │   ├── MPU9250.cpp/.h
│   │   ├── invensense_imu.cpp/.h
│   │   ├── SBUS.cpp/.h
│   │   └── logger.h            Print → 控制台串口
│   └── comm/                   通信层
│       ├── wifi.cpp            ← wifi.ino
│       ├── http_server.cpp/.h  替代 Arduino WebServer（BSD socket + 独立任务）
│       ├── web_rc.cpp          ← web_rc.ino
│       ├── web_rc_html.h       ← web_rc_html.h（仅去掉 PROGMEM）
│       ├── mavlink.cpp         ← mavlink.ino
│       └── mavlink_compat.h    MAVLink 头文件聚合入口
└── doc/
    └── MIGRATION.md            本文档
```

---

## 2. Arduino API → ESP-IDF API 映射

### 2.1 时间

| Arduino | ESP-IDF | 位置 |
|---|---|---|
| `micros()` | `hal_micros()` / `hal_micros64()` → `esp_timer_get_time()` | `hal/hal_time.h` |
| `millis()` | `hal_millis()` → `esp_timer_get_time()/1000` | 同上 |
| `delay(ms)` | `hal_delay_ms()` → `vTaskDelay()` | `hal/hal_time.cpp` |

> 注意：Arduino 的 `micros()` 是 32 位、约 71 分钟回绕。
> 本工程 `app/time.cpp` 使用 64 位 `esp_timer_get_time()`，
> `t` 为 `float`，长期运行（>4.6 小时）后 float 精度会开始影响 `dt`，
> 与原工程行为一致，如需长时间飞行请改用 `double`。

### 2.2 GPIO / LEDC / ADC

| Arduino | ESP-IDF | 位置 |
|---|---|---|
| `pinMode` / `digitalWrite` | `hal_gpio_init()` / `hal_gpio_write()` → `gpio_config` / `gpio_set_level` | `hal/hal_gpio.cpp` |
| `ledcAttach(pin,freq,res)` | `hal_motor_pwm_init()` → `ledc_timer_config` + `ledc_channel_config` | `hal/hal_pwm.cpp` |
| `ledcWrite(pin,duty)` | `hal_motor_pwm_write(ch,duty)` | 同上 |
| `ledcDetach(pin)` | `hal_motor_pwm_detach(pin)` | 同上 |
| `ledcChangeFrequency()` | `ledc_get_freq()` | 同上 |
| `analogReadMilliVolts()` | `hal_battery_read_voltage()` → `adc_oneshot` + **curve_fitting 校准** | `hal/hal_adc.cpp` |

> **`analogReadMilliVolts()` 的等价性是本次移植最关键的精度点。**
> Arduino 版内部做 eFuse 校准 + 曲线拟合；IDF 版必须显式走
> `adc_cali_create_scheme_curve_fitting()`，否则电压读数偏差会直接
> 导致低电保护阈值（3.5/3.3/3.0 V）失准。
> 若 eFuse 无校准数据，代码会退化为原始值线性换算并打警告。

> **LEDC 时钟**：Arduino 的 `ledcAttach` 用 `LEDC_AUTO_CLK`（S3 上解析为 XTAL 40 MHz）。
> 本工程同样使用 `LEDC_AUTO_CLK`，25 kHz @ 10 bit 的分频比 1.5625 可被硬件小数分频精确表示。
> `setupMotors()` 仍保留「回读实际频率并反写 `pwmFrequency`」的逻辑，
> 保证 ESC 模式下 `getDutyCycle()` 的脉宽换算正确。

### 2.3 SPI / I2C / UART

| Arduino | ESP-IDF | 位置 |
|---|---|---|
| `SPIClass` / `SPISettings` | `SpiDeviceIdf`（`SPIClass` 为其别名） | `hal/hal_spi.h/.cpp` |
| `TwoWire` | 占位类（本工程 IMU 走 SPI） | `hal/hal_i2c.h` |
| `HardwareSerial` | `hal_rc_uart_*` → `uart_driver_install` / `uart_param_config` | `hal/hal_uart.cpp` |
| `Serial`（控制台） | `hal_console_*` → UART0 | `hal/hal_console.cpp` |

SPI 关键实现细节：
- MPU-9250 要求 **SPI 模式 3**（CPOL=1, CPHA=1），已在 `spi_device_interface_config_t` 中固定。
- **片选由软件控制**（`spics_io_num = -1` + `SPI_TRANS_CS_KEEP_ACTIVE`），
  而不是让 IDF 自动控制——原 Arduino 驱动依赖「一次事务内多次 `transfer()` 保持 CS 低」的语义，
  若交给硬件自动控制会在每次 `transfer()` 之间抬高 CS，导致寄存器读取错位。
- 按 `spi_clock` 缓存 `spi_device_handle_t`（1 MHz 配置 / 15 MHz 读取），
  避免每次传输都 `spi_bus_add_device()`。

CRSF / SBUS 参数：

| 协议 | 波特率 | 帧格式 | 反相 |
|---|---|---|---|
| CRSF/ELRS | 420000 | 8N1 | 否 |
| SBUS | 100000 | 8E2 | 是（`uart_set_line_inverse(RXD_INV｜TXD_INV)`） |

### 2.4 NVS（参数持久化）

| Arduino `Preferences` | 本工程 `NvsStore` | ESP-IDF 底层 |
|---|---|---|
| `begin(ns, false)` | `begin(ns, false)` | `nvs_open` |
| `isKey` | `isKey` | `nvs_get_*` 探测 |
| `getFloat` / `putFloat` | 同名 | `nvs_get_blob` / `nvs_set_blob`（4 字节，与原实现一致） |
| `getString` / `putString` | 同名 | `nvs_get_str` / `nvs_set_str` |
| `clear()` | `clear()` | `nvs_erase_all` + `nvs_commit` |

> **兼容性**：命名空间沿用 **`flix`**，全部参数名逐字保持原样，
> 唯一例外见 §4「已知行为差异」。
> NVS 键长度上限 15 字节，`setupParameters()` 会对超长参数名打警告。

### 2.5 Wi-Fi / UDP / HTTP

| Arduino | ESP-IDF | 位置 |
|---|---|---|
| `WiFi.setSleep(false)` | `esp_wifi_set_ps(WIFI_PS_NONE)` | `comm/wifi.cpp` |
| `WiFi.softAP()` | `esp_netif_create_default_wifi_ap()` + `esp_wifi_set_config()` | 同上 |
| `WiFi.begin()` | `esp_netif_create_default_wifi_sta()` + `esp_wifi_connect()` | 同上 |
| `WiFi.softAPgetStationNum()` | `WIFI_EVENT_AP_STACONNECTED/DISCONNECTED` 事件计数 | 同上 |
| `WiFiUDP` | 非阻塞 BSD socket（`socket`/`bind`/`recvfrom`/`sendto`） | 同上 |
| `WebServer` | `comm/http_server`（BSD socket + 独立任务） | `comm/http_server.cpp` |
| `WiFiServer`（8080 重定向） | 第二个监听 socket + 独立任务 | 同上 |

### 2.6 其他

| Arduino | ESP-IDF | 位置 |
|---|---|---|
| `String` | 定长 `char[]` + `snprintf` + `strtok_r` | `app/cli.cpp`、`comm/web_rc.cpp` |
| `ESP.getChipModel()` | `esp_chip_info()` + 型号表 | `app/cli.cpp` |
| `temperatureRead()` | `temperature_sensor_get_celsius()` | 同上 |
| `ESP.getFreeHeap()` | `esp_get_free_heap_size()` | 同上 |
| `ESP.restart()` | `esp_restart()` | `app/parameters.cpp`、`app/cli.cpp` |
| `hw_timer_t` + ISR | `esp_timer` 周期回调 + 二值信号量 | `drivers/IMU.h` |
| `attachInterruptArg` | 预留 `gpio_isr_service`（本工程走定时器节拍） | 同上 |
| `Printable` / `printTo()` | 已移除（无调用者） | `app/math/vector.h`、`quaternion.h` |
| `constrain` / `radians` / `degrees` / `mapf` | 同名内联函数 | `app/math/util.h` |
| `MAVLink.h` | `comm/mavlink_compat.h` → `common/common.h` + `mavlink_helpers.h` | `comm/mavlink_compat.h` |

---

## 3. 结构化改进（与原 Arduino 版的差异）

1. **飞控主循环与网络处理彻底分离**
   原版在 `loop()` 里调用 `webRCServer.handleClient()`，HTTP 处理直接占用飞控时间片。
   现在：飞控跑在 `fcTask`（默认 core 1，优先级 10），
   HTTP 服务器跑在独立任务（默认 core 0）。
   可用 menuconfig 调整：`CF-Drone 飞控配置 → 飞控主循环绑定的 CPU 核心 / HTTP 服务器任务绑定的 CPU 核心`。

2. **消除堆分配**
   - `String`（原 `cli.ino`/`web_rc.ino`/`mavlink.ino` 大量使用）→ 定长缓冲，
     同时消除了长时间运行下的堆碎片风险。
   - MAVLink 的 `mavlinkPrintBuffer`（原 `String`）→ 1024 字节环形缓冲。
   - `cli.ino` 的 `print()` 保留原有的「超长内容转堆分配」策略（行为一致），
     但任务表打印改用 `malloc/free` 而非 `new[]`。

3. **跨核访问加锁**
   Web RC 的 HTTP 处理器运行在 core 0，飞控在 core 1，
   因此对 `webRCLastUpdate`、`webRCWarnMsg`、控制台环形缓冲与命令队列的访问
   都使用 `portENTER_CRITICAL` 保护（原 Arduino 版单核串行，无需加锁）。

4. **`globals.h` 集中声明**
   原工程依赖 Arduino IDE 把多个 `.ino` 拼成单个翻译单元，靠散落的 `extern` 共享全局变量。
   现在统一在 `app/globals.h` 声明，各模块 include 即可。

5. **`gcc -Werror=all`**
   ESP-IDF 默认把警告当错误。移植过程中已修掉：
   - `PID` 与 `Vector` 成员初始化顺序（`-Werror=reorder`）
   - 整数/浮点隐式转换（`-Wno-sign-compare` 已关闭该噪声）
   - Bolder Flight 驱动的 `setupInterrupt` 重载遮蔽（`-Wno-overloaded-virtual`，
     属既有设计，未改行为）

---

## 4. 已知行为差异与注意事项

### 4.1 首次刷入后必须重新标定
原 Arduino 版使用默认分区表，本工程使用自定义分区表，
**NVS 分区偏移与大小不同，旧参数数据不可用**。
首次刷入后需要：

```
preset        # 清空参数存储并重启
ca            # 六面加速度计标定
cr            # 遥控器通道标定
```

### 4.2 保留的原工程问题（未做「修正」，以免改变已标定机体的行为）

- **`calibrateAccelOnce()` 的校正式语义**
  原实现为 `acc = (acc - accBias) / accScale`（先减后除），
  而 `parameters.cpp` 注释写的是 `(acc_raw - bias) / scale` —— 二者一致，
  但变量名为 `scale` 却做了除法，属于命名误导。代码逐行保留。
- **`calibrateAccelOnce()` 不修改全局 `acc`**
  函数末尾 `acc = acc / samples` 之后没有恢复或写回，
  即该函数会污染全局 `acc`（标定结束后由下一次 `readIMU()` 覆盖）。
  逐行保留。
- **`calibrateAccel()` 内部 `pause(8)` 会阻塞主循环**
  移植后 `pause()` 仍在飞控任务上下文中执行，
  期间电机保持最后状态。**标定务必拆桨进行。**
- **`batteryAlertActive()` 的 L1/L2 阈值**
  原 `led.ino` 注释写 L2=2.8V、L1=3.4V，但 `battery.ino` 实际定义
  `VBAT_WARN_THRESHOLD=3.5f`、`VBAT_LOW_THRESHOLD=3.3f`。
  本工程按**代码实际值**（3.5 / 3.3 / 3.0）统一到 `app/globals.h`，
  并修正了 `batteryFailsafe()` 的注释。

### 4.3 移植过程中的两个具体技术坑（供后续维护参考）

**（1）特性宏必须在包含头文件之前可见**

`main/hal/board.h` 依据 Kconfig 定义 `WIFI_ENABLED` / `WEB_RC_ENABLED`。
任何使用 `#if WEB_RC_ENABLED` 的头文件（`app/control.h`、`comm/web_rc.h`）
或源文件（`comm/web_rc.cpp`、`comm/mavlink.cpp`）都**必须先包含 `hal/board.h`**，
否则宏未定义 → `#if` 判为假 → 整段代码被预处理器丢弃 →
表现为「编译通过但链接时大量 undefined reference，且对应 .obj 为空」。

这一坑在实践中很容易被误判为链接器问题，排查方法：

```powershell
# 若某目标文件没有任何符号，基本就是被 #if 整段丢弃了
xtensa-esp32s3-elf-nm --defined-only `
  build/esp-idf/main/CMakeFiles/__idf_main.dir/comm/web_rc.cpp.obj
```

**（2）只被数据表引用的 `static` 函数会被 GCC 丢弃**

`comm/web_rc.cpp` 的 HTTP 路由表 `webRcRoutes[]` 由 `comm/http_server.cpp`
以 `extern` 引用。原先 7 个路由处理函数都声明为 `static`，
其唯一引用来自同文件的 `webRcRoutes[]`；GCC 在 `-ffunction-sections` 下
判定它们「本翻译单元内未被使用」并整体丢弃，连带 `webRcRoutes[]` 也被移除，
最终链接报 `undefined reference to 'webRcRoutes'`。

> 有趣的是字符串字面量（`"/console/cmd"` 等）仍留在 `.rodata` 中，
> 所以只看「字符串是否存在」会误判为已编译。

修复方式：给路由处理函数与路由表加 `extern "C"`（外部链接），
使其无法被编译器丢弃。这是标准 C++ 工程相对 Arduino 单翻译单元的
一个真实差异——Arduino IDE 把所有 `.ino` 合成一个 TU，不会触发该优化。

### 4.4 新增的 Kconfig 选项

原 `board_config.h` 里的 `BOARD_WIFI_ENABLED` / `BOARD_WEB_RC_ENABLED`
宏已迁移到 Kconfig（`main/Kconfig.projbuild`），
同时保持宏映射（`WIFI_ENABLED` / `WEB_RC_ENABLED`），
使 `#if WEB_RC_ENABLED` 写法继续可用。

`idf.py menuconfig` → **CF-Drone 飞控配置**：

| 选项 | 默认 | 说明 |
|---|---|---|
| 启用 WiFi | y | 关闭可释放约 40 KB RAM |
| 启用 Web 遥控器 | y | 依赖 WiFi |
| 启用 MAVLink 遥测 | y | 依赖 WiFi |
| 调试串口控制器编号 / 波特率 | 0 / 115200 | |
| 飞控主循环绑定的 CPU 核心 | 1 | |
| 飞控主循环任务优先级 / 栈 | 10 / 8192 | |
| HTTP 服务器任务绑定的 CPU 核心 | 0 | |
| Web 遥控器 HTTP 端口 | 80 | |
| 旧 PCB 地址兼容重定向端口 | 8080 | 设 0 禁用 |
| MAVLink 系统 ID / UDP 端口 | 1 / 14550 | |

### 4.5 功能开关组合下的可构建性

`app/parameters.cpp` 的参数表**无条件**引用了一批通信配置变量：

```
WIFI_MODE → wifiMode        WIFI_LOC_PORT → udpLocalPort
WIFI_REM_PORT → udpRemotePort
MAV_SYS_ID → mavlinkSysId   MAV_RATE_SLOW → telemetrySlowRate
MAV_RATE_FAST → telemetryFastRate
```

如果把这些变量定义在 `comm/wifi.cpp` / `comm/mavlink.cpp` 内部，
一旦在 menuconfig 中关闭 WiFi / Web RC / MAVLink，对应源文件整体被
条件编译排除，参数表就会 `undefined reference`，**工程无法构建**。

因此它们统一挪到 **`app/comm_config.cpp`**（无条件参与编译），
`comm/wifi.cpp` 与 `comm/mavlink.cpp` 改为 `extern` 引用。
同理，`main/Kconfig.projbuild` 中的 `CFDRONE_HTTP_*` 三项
**故意不设置** `depends on CFDRONE_WEB_RC_ENABLED` —— 因为
`comm/http_server.cpp` 无条件参与编译且引用了这些宏。

**已实测的组合**：

| 配置 | 结果 |
|---|---|
| WiFi + Web RC + MAVLink 全开（默认） | ✅ 构建通过，`cf_drone.bin` = 901,152 B |
| WiFi 关闭（Web RC / MAVLink 随之关闭） | ✅ 构建通过，`cf_drone.bin` = 886,640 B |

> 关闭 WiFi 可省约 14 KB flash；运行时还能再省约 40 KB RAM。

### 4.6 分区表与 Flash 大小
`partitions.csv` 按 **4 MB flash**（S3 模组最常见配置）设计，
不预留 OTA 槽位（原 Arduino 工程也无 OTA 能力）。
若模组是 8/16 MB，请在 menuconfig 中调大
`Serial Flasher Config → Flash size`，并按需扩展 `factory` 分区。

### 4.7 MAVLink 组件
使用乐鑫官方组件 `espressif/mavlink`（内部即 `mavlink/c_library_v2`），
由 `main/idf_component.yml` 声明，首次构建自动下载到 `managed_components/`。
该组件把头文件放在子目录 `c_library_v2/` 下，
`main/CMakeLists.txt` 已显式注入该路径。

**若需完全离线构建**：把 `c_library_v2` 整个目录拷到 `components/mavlink/`，
并改写 `main/CMakeLists.txt` 中的 `CFDRONE_MAVLINK_DIR` 指向它。

---

## 5. 构建与烧录

### 5.1 本机环境说明（重要）

本机 ESP-IDF 安装布局比较特殊，`export.ps1` 默认会失败：

| 项目 | 实际路径 |
|---|---|
| IDF 源码 | `D:\esp\v5.4.4\esp-idf` |
| 工具链 | `D:\Espressif\tools` |
| Python venv | `D:\Espressif\tools\python\v5.4.4\venv` |

`export.ps1` 默认去找 `D:\Espressif\python_env\idf5.4_py3.9_env`，该路径不存在，
因此需要：
1. 设置 `IDF_PYTHON_ENV_PATH` 指向真实的 venv；
2. `D:\Espressif\espidf.constraints.v5.4.txt` 需存在（从 `D:\Espressif\tools\` 复制）；
3. 临时 PowerShell 执行策略需为 `Bypass`（`export.ps1` 生成的激活脚本未签名）。

工程内已提供包装脚本处理以上三点。

### 5.2 使用包装脚本（推荐）

**PowerShell：**
```powershell
cd idf
.\idf_env.ps1 idf.py build
.\idf_env.ps1 idf.py -p COM5 flash monitor
```
或先激活再直接调用：
```powershell
.\idf_env.ps1
idf.py build
```

**cmd.exe：**
```bat
cd idf
idf_env.bat idf.py build
```

### 5.3 手动激活

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
$env:IDF_TOOLS_PATH        = 'D:\Espressif'
$env:IDF_PYTHON_ENV_PATH   = 'D:\Espressif\tools\python\v5.4.4\venv'
. D:\esp\v5.4.4\esp-idf\export.ps1
cd idf
idf.py set-target esp32s3    # 首次
idf.py build
idf.py -p COM5 flash monitor
```

### 5.4 常用命令

```bash
idf.py build                 # 构建
idf.py -p COM5 flash         # 烧录
idf.py -p COM5 monitor       # 串口监视（Ctrl+] 退出）
idf.py menuconfig            # 配置（CF-Drone 飞控配置）
idf.py size-components       # 各组件的 flash 占用
idf.py fullclean             # 彻底清理
```

---

## 6. 移植后的验证清单

建议按此顺序验证（**务必拆桨**）：

| # | 步骤 | 预期 |
|---|---|---|
| 1 | 上电，串口 115200 | 输出 `琛光科技 CF-Drone Flight Controller` 与 CLI motd |
| 2 | `help` | 打印命令菜单 |
| 3 | `sys` | 显示 `Chip: ESP32-S3`、温度、空闲堆、任务表（含 Core 列） |
| 4 | `time` | `Loop rate` 接近 1000（IMU 1 kHz 节拍） |
| 5 | `imu` | `status: OK`、`model: MPU-6500/9250`、whoAmI 正确 |
| 6 | `ps` | 静止时 roll/pitch ≈ 0，yaw 稳定 |
| 7 | `rc` | 拨动摇杆时 `channels` 数值变化；CRSF 默认 172~1811 |
| 8 | `mot` | 显示四路电机输出 |
| 9 | `mfl` / `mfr` / `mrl` / `mrr` | 对应电机低速转动 3 秒（确认桨叶已拆） |
| 10 | `p CTL_TRIM_ROLL 0.01` → 重启 → `p CTL_TRIM_ROLL` | 参数持久化生效 |
| 11 | 电池接入，`/web_rc/status` 或观察 LED | 电压读数与万用表偏差应 < 0.1 V |
| 12 | 手机连 `Drone_WiFi` / `12345678`，浏览器开 `http://192.168.4.1` | 网页遥控器界面加载，摇杆可用 |
| 13 | 网页控制台 `enable` 后执行命令 | 能收到固件输出 |
| 14 | QGroundControl 连 UDP 14550 | 能收到心跳、读参数、`MANUAL_CONTROL` 生效 |
| 15 | 与原 Arduino 版对比 `log` 数据（同 PID 参数） | 姿态曲线应一致；**若不一致需重新整定 PID** |

> **第 15 项很重要**：`micros()` → `esp_timer_get_time()` 的精度特征不同，
> `pid.h` 中 `dt` 的抖动特性会有细微差别，PID 可能需要微调。

---

## 7. 尚待完成 / 未实现的部分

| 项 | 状态 | 说明 |
|---|---|---|
| 扩展板传感器（BMP388 / VL53L1X / QMC5883L / PMW3901） | **未实现** | 原工程只在 `board_config.h` 中定义了引脚与开关宏，`EXPANSION_BOARD_ENABLED` 无对应实现代码，未参与编译。本工程保留宏定义。 |
| ALTHOLD 定高模式 | 未实现（与原版一致） | 原版同样为 stub，`ALTHOLD_HOVER_THRUST` 为占位常量 |
| SBUS 遥测回传（`SBUSTx`） | 已移除 | 原 `rc.ino` 只使用接收端；`SBUSTx` 未参与编译，为减小体积已从 `drivers/SBUS.cpp` 删除 |
| IMU DRDY 引脚中断 | 保留接口但未启用 | 原工程默认走定时器节拍；`IMUBase::setupInterruptPin()` 已改为返回 false 并注释说明接入方式 |
| 硬件实测 | **未进行** | 本次交付为源码级移植 + 编译验证，未在真实硬件上跑过 |
