// 参数存储在闪存（NVS）
//
// 移植说明（ESP-IDF）：
//   * Arduino 的 Preferences 换成 hal/hal_nvs.h 的 NvsStore（接口逐一对齐，
//     底层同样操作 NVS，命名空间沿用 "flix"）。
//   * 原 begin() 失败时调用 nvs_flash_erase() + nvs_flash_init() 重试的逻辑，
//     已内聚到 NvsStore::begin() 中。
//   * ESP.restart() -> esp_restart()。
//   ★ 参数名必须保持不变，否则升级固件后用户标定数据会全部丢失。

#include "app/parameters.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "app/cli.h"
#include "app/globals.h"
#include "app/log.h"
#include "app/math/util.h"
#include "app/motors.h"
#include "app/rc.h"
#include "esp_system.h"
#include "hal/board.h"

NvsStore storage;

struct Parameter {
	const char *name; // 最长 15 字符（NVS 键限制）
	bool integer;
	union { float *f; int *i; };
	float cache; // 当前已持久化的值
	void (*callback)(); // 参数变更后的回调
	Parameter(const char *name, float *variable, void (*callback)() = nullptr)
		: name(name), integer(false), f(variable), cache(0), callback(callback) {}
	Parameter(const char *name, int *variable, void (*callback)() = nullptr)
		: name(name), integer(true), i(variable), cache(0), callback(callback) {}
	float getValue() const { return integer ? (float)*i : *f; }
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; }
};

Parameter parameters[] = {
	// ===== 控制（角速率内环 PID）=====
	// 角速率环直接控制机身旋转速度，是最内层控制回路。
	// P：比例增益，值越大响应越快，过大会振荡；
	// I：积分增益，消除稳态误差（如电机差异），过大会产生积分饱和；
	// D：微分增益，抑制超调，过大会放大高频噪声；
	// WU（windup）：积分饱和限制幅值，防止 I 项累积过大。
	{"CTL_R_RATE_P",  &rollRatePID.p},      // 横滚角速率 P 增益（rad/s → 力矩）
	{"CTL_R_RATE_I",  &rollRatePID.i},      // 横滚角速率 I 增益
	{"CTL_R_RATE_D",  &rollRatePID.d},      // 横滚角速率 D 增益
	{"CTL_R_RATE_WU", &rollRatePID.windup}, // 横滚角速率积分限幅（±windup）
	{"CTL_P_RATE_P",  &pitchRatePID.p},     // 俯仰角速率 P 增益
	{"CTL_P_RATE_I",  &pitchRatePID.i},     // 俯仰角速率 I 增益
	{"CTL_P_RATE_D",  &pitchRatePID.d},     // 俯仰角速率 D 增益
	{"CTL_P_RATE_WU", &pitchRatePID.windup},// 俯仰角速率积分限幅
	{"CTL_Y_RATE_P",  &yawRatePID.p},       // 偏航角速率 P 增益（偏航惯量小，通常需更大值）
	{"CTL_Y_RATE_I",  &yawRatePID.i},       // 偏航角速率 I 增益
	{"CTL_Y_RATE_D",  &yawRatePID.d},       // 偏航角速率 D 增益

	// ===== 控制（角度外环 PID）=====
	// 角度环将目标姿态角误差转换为角速率指令，输出给内环。
	// I 项用于自动补偿物理不对称引起的固定悬停偏差，需同时配置 WU 积分限幅才能生效。
	{"CTL_R_P",  &rollPID.p},      // 横滚角度 P 增益（角度误差 rad → 角速率指令 rad/s）
	{"CTL_R_I",  &rollPID.i},      // 横滚角度 I 增益
	{"CTL_R_D",  &rollPID.d},      // 横滚角度 D 增益（一般为 0）
	{"CTL_R_WU", &rollPID.windup}, // 横滚角度积分限幅（rad/s）
	{"CTL_P_P",  &pitchPID.p},     // 俯仰角度 P 增益
	{"CTL_P_I",  &pitchPID.i},     // 俯仰角度 I 增益
	{"CTL_P_D",  &pitchPID.d},     // 俯仰角度 D 增益
	{"CTL_P_WU", &pitchPID.windup},// 俯仰角度积分限幅（rad/s）
	{"CTL_Y_P",  &yawPID.p},       // 偏航角度 P 增益

	// ===== 控制（限制值）=====
	{"CTL_P_RATE_MAX", &maxRate.y}, // 俯仰最大角速率限制（rad/s）
	{"CTL_R_RATE_MAX", &maxRate.x}, // 横滚最大角速率限制（rad/s）
	{"CTL_Y_RATE_MAX", &maxRate.z}, // 偏航最大角速率限制（rad/s）
	{"CTL_TILT_MAX",   &tiltMax},   // 稳定模式最大允许倾斜角（rad）

	// ===== 控制（软件配平）=====
	// 用于补偿飞机重心偏移或电机推力不一致导致的悬停偏移，无需物理调整。
	{"CTL_TRIM_ROLL",  &trimRoll},  // 横滚配平角（rad），正值向右滚
	{"CTL_TRIM_PITCH", &trimPitch}, // 俯仰配平角（rad），正值向后仰

	// ===== 控制（飞行模式）=====
	// 遥控器模式通道将摇杆三挡位映射到对应飞行模式枚举值：
	// 0=RAW（直通），1=ACRO（特技），2=STAB（自稳），3=ALTHOLD（气压计定高），4=AUTO
	{"CTL_FLT_MODE_0", &flightModes[0]}, // 模式通道第 0 挡对应的飞行模式
	{"CTL_FLT_MODE_1", &flightModes[1]}, // 模式通道第 1 挡对应的飞行模式
	{"CTL_FLT_MODE_2", &flightModes[2]}, // 模式通道第 2 挡对应的飞行模式

	// ===== IMU（传感器安装旋转补偿）=====
	// 若 IMU 安装方向与机体坐标系不一致，在此设置旋转角（rad）进行软件补偿。
	{"IMU_ROT_ROLL",  &imuRotation.x}, // IMU 绕机体 X 轴（横滚轴）的安装偏转角（rad）
	{"IMU_ROT_PITCH", &imuRotation.y}, // IMU 绕机体 Y 轴（俯仰轴）的安装偏转角（rad）
	{"IMU_ROT_YAW",   &imuRotation.z}, // IMU 绕机体 Z 轴（偏航轴）的安装偏转角（rad）

	// ===== IMU（陀螺仪偏置滤波）=====
	// 陀螺仪静态偏置通过低通滤波器（指数移动平均）在上电初始化时估计。
	// alpha 越小滤波越强（收敛越慢），越大收敛越快但对振动噪声更敏感。
	{"IMU_GYRO_BIAS_A", &gyroBiasFilter.alpha}, // 陀螺偏置低通滤波器系数 alpha（0~1）

	// ===== IMU（加速度计标定）=====
	// 出厂或重新安装后需对加速度计进行六面标定，结果存入以下参数。
	// 标定公式：acc_corrected = (acc_raw - bias) / scale
	{"IMU_ACC_BIAS_X", &accBias.x},   // 加速度计 X 轴零偏（m/s²）
	{"IMU_ACC_BIAS_Y", &accBias.y},   // 加速度计 Y 轴零偏（m/s²）
	{"IMU_ACC_BIAS_Z", &accBias.z},   // 加速度计 Z 轴零偏（m/s²）
	{"IMU_ACC_SCALE_X", &accScale.x}, // 加速度计 X 轴增益校正因子（理想值 1.0）
	{"IMU_ACC_SCALE_Y", &accScale.y}, // 加速度计 Y 轴增益校正因子
	{"IMU_ACC_SCALE_Z", &accScale.z}, // 加速度计 Z 轴增益校正因子

	// ===== 姿态估计 =====
	// 采用互补滤波：陀螺仪积分提供高频动态，加速度计提供低频重力修正。
	{"EST_ACC_WEIGHT",   &accWeight},          // 加速度计对姿态的修正权重（0~1）
	{"EST_RATES_LPF_A",  &ratesFilter.alpha},  // 角速率低通滤波器系数 alpha（0~1）
	{"EST_LVL_GATE_THR", &levelGateThreshold}, // 摇杆门控阈值（0~1）
	{"EST_LVL_BIAS_GAIN",&levelBiasGain},      // Mahony I 项增益

	// ===== 电机（引脚配置）=====
	// 修改后立即调用 setupMotors() 重新初始化 LEDC 通道，无需重启。
	{"MOT_PIN_FL", &motorPins[MOTOR_FRONT_LEFT],  setupMotors}, // 左前电机 GPIO 引脚号
	{"MOT_PIN_FR", &motorPins[MOTOR_FRONT_RIGHT], setupMotors}, // 右前电机 GPIO 引脚号
	{"MOT_PIN_RL", &motorPins[MOTOR_REAR_LEFT],   setupMotors}, // 左后电机 GPIO 引脚号
	{"MOT_PIN_RR", &motorPins[MOTOR_REAR_RIGHT],  setupMotors}, // 右后电机 GPIO 引脚号

	// ===== 电机（PWM 配置）=====
	// 直驱 MOSFET 模式：pwmMax=-1，pwmStop/pwmMin=0，使用纯占空比控制。
	// ESC 模式：pwmMax 设为脉宽上限（μs），pwmMin 设为脉宽下限，pwmFrequency 降至 400Hz。
	{"MOT_PWM_FREQ", &pwmFrequency, setupMotors}, // PWM 频率（Hz）
	{"MOT_PWM_RES",  &pwmResolution, setupMotors},// PWM 分辨率（bit）
	{"MOT_PWM_STOP", &pwmStop},                   // 电机停转时的 PWM 值
	{"MOT_PWM_MIN",  &pwmMin},                    // 电机最小 PWM 值
	{"MOT_PWM_MAX",  &pwmMax},                    // -1=纯占空比模式；正整数=ESC 脉宽上限（μs）

	// ===== 电机（推力映射）=====
	// 将归一化油门（0~1）线性映射到电机推力输出范围，保留余量供姿态修正使用。
	{"MOT_THR_MIN", &motThrMin}, // 推力下限（0~1），建议≥0.05 防失速
	{"MOT_THR_MAX", &motThrMax}, // 推力上限（0~1），建议≤0.9 保留姿态修正余量

	// ===== 遥控（通道校准）=====
	// 原始 RC 信号经校准转换为归一化值。校准公式：normalized = (raw - zero) / max
	{"RC_ZERO_0", &channelZero[0]}, // 通道 0 中立点（校准用）
	{"RC_ZERO_1", &channelZero[1]}, // 通道 1 中立点
	{"RC_ZERO_2", &channelZero[2]}, // 通道 2 中立点
	{"RC_ZERO_3", &channelZero[3]}, // 通道 3 中立点
	{"RC_ZERO_4", &channelZero[4]}, // 通道 4 中立点
	{"RC_ZERO_5", &channelZero[5]}, // 通道 5 中立点
	{"RC_ZERO_6", &channelZero[6]}, // 通道 6 中立点
	{"RC_ZERO_7", &channelZero[7]}, // 通道 7 中立点
	{"RC_MAX_0",  &channelMax[0]},  // 通道 0 行程幅值（校准用）
	{"RC_MAX_1",  &channelMax[1]},  // 通道 1 行程幅值
	{"RC_MAX_2",  &channelMax[2]},  // 通道 2 行程幅值
	{"RC_MAX_3",  &channelMax[3]},  // 通道 3 行程幅值
	{"RC_MAX_4",  &channelMax[4]},  // 通道 4 行程幅值
	{"RC_MAX_5",  &channelMax[5]},  // 通道 5 行程幅值
	{"RC_MAX_6",  &channelMax[6]},  // 通道 6 行程幅值
	{"RC_MAX_7",  &channelMax[7]},  // 通道 7 行程幅值

	// ===== 遥控（通道功能映射）=====
	// 指定各功能轴使用的物理通道编号（0-based 整数）。
	{"RC_ROLL",     &rollChannel},     // 横滚通道编号
	{"RC_PITCH",    &pitchChannel},    // 俯仰通道编号
	{"RC_THROTTLE", &throttleChannel}, // 油门通道编号
	{"RC_YAW",      &yawChannel},      // 偏航通道编号
	{"RC_MODE",     &modeChannel},     // 飞行模式切换通道编号（三挡拨杆）

	// ===== 遥控（串口硬件配置）=====
	// 四个参数均绑定 setupRC 回调：修改后立即重新初始化 RC 串口，无需重启即可生效。
	{"RC_RX_PIN",   &rcRxPin,    setupRC}, // 遥控接收器数据输入 GPIO 引脚（UART RX）
	{"RC_PROTOCOL", &rcProtocol, setupRC}, // 遥控协议：0=SBUS（电平反相），1=CRSF/ELRS
	{"RC_TX_PIN",   &rcTxPin,    setupRC}, // 遥测回传 GPIO 引脚；-1=不启用回传
	{"RC_BAUD",     &rcBaud,     setupRC}, // 遥控串口波特率；SBUS=100000，CRSF=420000

	// ===== WiFi =====
	// 注意：WiFi 相关变量定义于 comm/wifi.cpp。
	{"WIFI_MODE",     &wifiMode},      // WiFi 工作模式：0=关闭，1=STA，2=AP
	{"WIFI_LOC_PORT", &udpLocalPort},  // 本地 UDP 监听端口
	{"WIFI_REM_PORT", &udpRemotePort}, // 远端 UDP 目标端口

	// ===== MAVLink 遥测 =====
	{"MAV_SYS_ID",    &mavlinkSysId},       // MAVLink 系统 ID（1~254）
	{"MAV_RATE_SLOW", &telemetrySlowRate},  // 慢速遥测发送频率（Hz）
	{"MAV_RATE_FAST", &telemetryFastRate},  // 快速遥测发送频率（Hz）

	// ===== 故障保护 =====
	{"SF_RC_LOSS_TIME",  &rcLossTimeout}, // RC 信号丢失超时阈值（秒）
	{"SF_DESCEND_TIME",  &descendTime},   // 自动下降至油门归零所需时间（秒）
};

static constexpr int PARAM_COUNT = (int)(sizeof(parameters) / sizeof(parameters[0]));

void setupParameters() {
	print("Setup parameters\n");
	if (!storage.begin("flix", false)) {
		print("[NVS] 参数命名空间打开失败，参数将无法持久化，请检查分区表/硬件！\n");
	}

	for (int i = 0; i < PARAM_COUNT; i++) {
		Parameter &parameter = parameters[i];

		// 键长度检查：NVS 上限 15 字节
		size_t nameLen = strlen(parameter.name);
		if (nameLen > NVS_KEY_MAX_LEN) {
			print("[NVS] 警告：参数名 %s 长度 %u 超过 %u 字节上限，会被截断\n",
			      parameter.name, (unsigned)nameLen, (unsigned)NVS_KEY_MAX_LEN);
		}

		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, parameter.getValue()); // store default value
		}
		float stored = storage.getFloat(parameter.name, parameter.getValue());
		// 整型参数若读到 NaN（flash 损坏），回退到代码默认值并重写
		if (parameter.integer && !isfinite(stored)) {
			stored = parameter.getValue();
			storage.putFloat(parameter.name, stored);
		}
		parameter.setValue(stored);
		parameter.cache = parameter.getValue();
	}
}

int parametersCount() {
	return PARAM_COUNT;
}

const char *getParameterName(int index) {
	if (index < 0 || index >= PARAM_COUNT) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= PARAM_COUNT) return NAN;
	return parameters[index].getValue();
}

float getParameter(const char *name) {
	for (int i = 0; i < PARAM_COUNT; i++) {
		if (strcasecmp(parameters[i].name, name) == 0) {
			return parameters[i].getValue();
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (int i = 0; i < PARAM_COUNT; i++) {
		if (strcasecmp(parameters[i].name, name) == 0) {
			if (parameters[i].integer && !isfinite(value)) return false; // 整型不允许 NaN/Inf
			parameters[i].setValue(value);
			if (parameters[i].callback) parameters[i].callback();
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // 每秒同步一次
	if (motorsActive()) return; // 飞行中不写 flash，避免引入延时

	for (int i = 0; i < PARAM_COUNT; i++) {
		Parameter &parameter = parameters[i];
		if (parameter.getValue() == parameter.cache) continue;
		if (isnan(parameter.getValue()) && isnan(parameter.cache)) continue; // handle NAN != NAN
		size_t written = storage.putFloat(parameter.name, parameter.getValue());
		if (written != sizeof(float)) continue; // 写入失败时保留旧 cache，下一轮自动重试
		parameter.cache = parameter.getValue();
	}
}

void printParameters() {
	for (int i = 0; i < PARAM_COUNT; i++) {
		print("%s = %g\n", parameters[i].name, parameters[i].getValue());
	}
}

void resetParameters() {
	storage.clear();
	esp_restart();
}
