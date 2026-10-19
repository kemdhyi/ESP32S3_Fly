// 陀螺仪加速度计相关设置
//
// 移植说明（ESP-IDF）：
//   * SPI.begin(sck,miso,mosi,cs) 由 hal/hal_spi 提供（底层 spi_master，
//     软件片选 + 模式 3，与原 Arduino 行为一致）。
//   * delay(1) -> hal_delay_ms(1)（FreeRTOS 延时）。
//   * IMU 采样节拍由 drivers/IMU.h 的 esp_timer 实现（替代 hw_timer_t）。
//
// ★ 注意（保留原实现行为，未做修正）：
//   readIMU() 中的校正式为 `acc = (acc - accBias) / accScale`
//   （与 parameters.cpp 中部注释所写的公式一致），
//   而 calibrateAccelOnce() 末尾的 `acc = acc / samples` 之后再无写回，
//   即六面标定只更新 accBias/accScale 参数，不改变已采集的 acc。
//   移植时逐行保留，避免改变现场已标定机体的行为。

#include "app/imu.h"

#include <math.h>
#include <stdio.h>

#include "app/cli.h"
#include "app/globals.h"
#include "app/math/lpf.h"
#include "app/math/quaternion.h"
#include "app/math/util.h"
#include "app/math/vector.h"
#include "drivers/MPU9250.h"
#include "hal/board.h"
#include "hal/hal_spi.h"
#include "hal/hal_time.h"

// 全局 SPI 实例：引脚在 setupIMU() 中传入
static SpiDeviceIdf imuSpi(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI, BOARD_SPI_CS);

MPU9250 imu(imuSpi, BOARD_SPI_CS);

bool imuOK = false; // IMU 初始化是否成功；false 时禁止解锁，readIMU() 跳过等待

// ---- 全局传感器状态（声明见 app/globals.h）----
Vector gyro;             // 陀螺仪数据（rad/s，已扣偏置）
Vector acc;              // 加速度计数据（m/s²，已做 bias/scale 校正）
Vector rates;            // 滤波后的角速度（rad/s）
Quaternion attitude;     // 估计姿态（四元数）
bool landed = false;     // 是否落地静止

// IMU 安装方向（欧拉角，单位 rad）。默认值 (0, 0, -PI/2) 对应本 PCB 的安装方式：
// 芯片正面朝上，X 丝印→飞行器右侧，Y 丝印→飞行器前方 → 转换公式 Vector(data.y, -data.x, data.z)
// 如需适配其他安装方向，修改此值并执行 `preset` 重置参数存储。
Vector imuRotation(0, 0, -(float)M_PI / 2);

Vector accBias;
Vector accScale(1, 1, 1);
Vector gyroBias;
LowPassFilter<Vector> gyroBiasFilter(0.001); // 陀螺仪偏置低通估计滤波器

void setupIMU() {
	print("Setup IMU\n");
	imuSpi.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI, BOARD_SPI_CS);
	if (!imu.begin()) {
		print("⚠ IMU初始化失败！请检查 IMU 硬件连接！\n");
		return; // 不调用 configureIMU()，不启动定时器中断，imuOK 保持 false
	}
	imuOK = true;
	configureIMU();
}

void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_MAX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
	imu.setupInterrupt();
}

void readIMU() {
	if (!imuOK) return; // IMU 故障时跳过，gyro/acc 保持零值，主循环继续运行
	imu.waitForData();
	imu.getGyro(gyro.x, gyro.y, gyro.z);
	imu.getAccel(acc.x, acc.y, acc.z);
	calibrateGyroOnce();
	// apply scale and bias
	acc = (acc - accBias) / accScale;
	gyro = gyro - gyroBias;
	// 按 imuRotation 欧拉角旋转到机体坐标系
	// 旋转四元数缓存：imuRotation 为运行时常量（仅参数变更时改变），
	// 缓存后每帧仅做 3 次浮点比较，消除原先每帧 6 次 sinf/cosf 调用。
	static Quaternion _imuRotQuat;
	static float _cachedRotX = NAN, _cachedRotY = NAN, _cachedRotZ = NAN;
	if (imuRotation.x != _cachedRotX || imuRotation.y != _cachedRotY || imuRotation.z != _cachedRotZ) {
		_imuRotQuat = Quaternion::fromEuler(imuRotation).inversed();
		_cachedRotX = imuRotation.x;
		_cachedRotY = imuRotation.y;
		_cachedRotZ = imuRotation.z;
	}
	acc  = Quaternion::rotateVector(acc,  _imuRotQuat);
	gyro = Quaternion::rotateVector(gyro, _imuRotQuat);
}

void calibrateGyroOnce() {
	static Delay landedDelay(2);
	if (!landedDelay.update(landed)) return; // 仅在确认静止时校准

	gyroBias = gyroBiasFilter.update(gyro);
}

void calibrateAccel() {
	print("校准陀螺仪加速计 Calibrating accelerometer\n");
	imu.setAccelRange(imu.ACCEL_RANGE_2G); // the most sensitive mode

	print("1/6 水平放置：机头朝前（正常飞行方向），底部朝下水平放置在平坦表面，确保完全水平无倾斜。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("水平校准完成。请继续。\n");
	pause(1);
	print("2/6 机头朝上：保持机头朝前,将机头指向天空，尾部朝下并接触支撑面，与地面垂直。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("机头向上校准完成。请继续。\n");
	pause(1);
	print("3/6 机头朝下：保持机头朝前,将机头指向地面并接触支撑面，尾部朝上，与地面垂直。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("机头向下校准完成。请继续。\n");
	pause(1);
	print("4/6 右侧朝下：保持机头朝前,将飞行器右侧朝下并接触支撑面，左侧机臂朝上，与地面垂直。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("右侧向下校准完成。请继续。\n");
	pause(1);
	print("5/6 左侧朝下：保持机头朝前,将飞行器左侧朝下并接触支撑面，右侧机臂朝上，与地面垂直。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("左侧向下校准完成。请继续。\n");
	pause(1);
	print("6/6 倒置放置：保持机头朝前,将飞行器完全翻转，顶部朝下、底部朝上，整体呈水平倒置状态。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();

	printIMUCalibration();
	print("✓全部校准完成！将机身放正，执行ps命令查看校准结果。\n");
	configureIMU();
}

void calibrateAccelOnce() {
	const int samples = 1000;
	static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
	static Vector accMin(INFINITY, INFINITY, INFINITY);

	// 计算加速度计读数的平均值
	acc = Vector(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		imu.waitForData();
		Vector sample;
		imu.getAccel(sample.x, sample.y, sample.z);
		acc = acc + sample;

		// 每 10 帧主动让出 1ms（对应原 ESP32-C3 分支的 WiFi 让出逻辑，
		// 在 S3 双核上飞控独占 core 1，仍保留以维持行为一致）
		if (i % 10 == 0) hal_delay_ms(1);
	}
	acc = acc / samples;

	// 更新最大最小值
	if (acc.x > accMax.x) accMax.x = acc.x;
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;
	// 计算 scale 与 bias
	accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}

void printIMUCalibration() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");
	print("model: %s\n", imu.getModel());
	print("who am I: 0x%02X\n", imu.whoAmI());
	print("rate: %.0f\n", loopRate);
	print("gyro: %f %f %f\n", gyro.x, gyro.y, gyro.z);
	print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}
