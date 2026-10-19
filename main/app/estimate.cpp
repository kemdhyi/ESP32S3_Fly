// 基于陀螺仪和加速度计的姿态计算

#include "app/estimate.h"

#include <math.h>

#include "app/globals.h"
#include "app/math/lpf.h"
#include "app/math/quaternion.h"
#include "app/math/util.h"
#include "app/math/vector.h"
#include "app/motors.h"

float accWeight = 0.003f;

// ============== 水平修正 P 项 ==============
float levelWeight = 0;  // 水平修正 P 项权重（关闭）
float levelMaxTilt = radians(30); // rad，水平修正在此倾角处衰减到零（与 TILT_MAX 一致）
// 摇杆感知门控阈值：飞手摇杆偏转量超过此阈值时修正权重线性衰减至 0。
// 参数名：EST_LVL_GATE_THR，推荐范围 0.2（灵敏）~ 0.4（宽松）
float levelGateThreshold = 0.2f;

// ============== Mahony 风格水平修正 I 项 ==============
// levelBiasGain：重力误差向虚拟陀螺偏置的积分增益（I 项）。
// 积分量在 applyGyro() 中从陀螺读数中减去，不直接修改 attitude 四元数，
// 因此 PID 控制回路感知不到修正过程。参数名：EST_LVL_BIAS_GAIN
float levelBiasGain = 0;
Vector levelGyroBias(0, 0, 0);

LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
}

void applyGyro() {
	// Mahony I 项：从陀螺读数中减去估计的虚拟偏置，再滤波积分
	rates = ratesFilter.update(gyro - levelGyroBias);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// 判断是否应施加加速度计重力修正
	float accNorm = acc.norm();
	landed = !motorsActive() && fabsf(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// 计算加速度计修正量
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// 施加修正
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// 水平修正（Mahony 风格：P 项直接修正 attitude，I 项积分进虚拟陀螺偏置）
//   - P 项（levelWeight）：对 attitude 施加小修正，提供快速响应
//   - I 项（levelBiasGain）：把重力参考误差积分到 levelGyroBias，
//     在 applyGyro() 中从陀螺读数减去，绕开 PID 误差输入
void applyLevel() {
	if (landed) {
		levelGyroBias = Vector(0, 0, 0); // 落地后清零偏置，下次起飞重新学习
		return;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acosf(constrain(up.z, -1.0f, 1.0f));

	// 极近水平时跳过：tilt→0 时叉积方向对振动噪声极敏感
	if (tilt < radians(0.1f)) return;

	// P 项权重：倾角越接近 levelMaxTilt，权重越小
	float dynamicWeight = levelWeight * constrain(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);

	// ---- 摇杆感知门控 ----
	float stickDeflection = fmaxf(fabsf(controlRoll), fabsf(controlPitch));
	float stickGate = constrain(1.0f - stickDeflection / levelGateThreshold, 0.0f, 1.0f);
	dynamicWeight *= stickGate;
	// ------------------------------------

	// 重力参考误差：机体 Z 轴与世界 Z 轴之间的旋转向量
	Vector error = Vector::rotationVectorBetween(Vector(0, 0, 1), up);

	// P 项：直接修正 attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(error * dynamicWeight));

	// I 项：积分进虚拟陀螺偏置（打杆期间 stickGate=0，暂停积分）
	levelGyroBias += error * (levelBiasGain * stickGate);

	// 限幅：等效最大补偿 3 deg/s，防止偏置发散
	float biasNorm = levelGyroBias.norm();
	const float biasLimit = radians(3.0f);
	if (biasNorm > biasLimit) levelGyroBias = levelGyroBias * (biasLimit / biasNorm);
}
