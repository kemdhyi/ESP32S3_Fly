// 使用MOSFET的电机输出控制
// 如果使用ESC，将pwmStop、pwmMin和pwmMax更改为适当的μs值，将pwmFrequency减小到400
//
// Arduino 版使用 ledcAttach(pin,...)/ledcWrite(pin,...)/ledcDetach(pin)（按引脚索引）。
// IDF 版改用 hal_motor_pwm_* （按通道索引），行为保持一致：
//   * 初始化时先解绑并拉低引脚，防止上电误转
//   * 回读 LEDC 实际频率并反写 pwmFrequency，保证 ESC 模式下脉宽换算正确

#include "app/motors.h"

#include <math.h>
#include <string.h>

#include "app/cli.h"
#include "app/globals.h"
#include "app/math/util.h"
#include "hal/board.h"
#include "hal/hal_pwm.h"
#include "hal/hal_time.h"

float motors[4]; // normalized motor thrusts in range [0..1]

// 电机引脚（对应 MOTOR_REAR_LEFT=0, MOTOR_REAR_RIGHT=1, MOTOR_FRONT_RIGHT=2, MOTOR_FRONT_LEFT=3）
int motorPins[4] = BOARD_MOTOR_PINS; // RL, RR, FR, FL

int pwmFrequency = 25000; // ESP32-S3 LEDC 时钟 40MHz（XTAL），10-bit 上限 39062Hz；25kHz 可精确输出
int pwmResolution = 10;
int pwmStop = 0;
int pwmMin = 0;
int pwmMax = -1; // -1 表示纯占空比模式（接 MOSFET 直驱）；接 ESC 时设为实际 PWM 最大值（μs）

// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

static constexpr int MOTOR_CHANNEL_BASE = 0;   // LEDC 通道 0..3

void setupMotors() {
	print("Setup Motors\n");

	// 先解绑所有引脚（重复调用时清理旧 LEDC 通道），再拉低防止误转
	for (int i = 0; i < 4; i++) {
		hal_motor_pwm_detach(motorPins[i]);
	}

	hal_motor_pwm_init(motorPins, 4, (uint32_t)pwmFrequency, (uint32_t)pwmResolution,
	                   MOTOR_CHANNEL_BASE);

	// 回读实际频率（对应 Arduino 版 ledcChangeFrequency 的返回值处理）
	int actual = hal_motor_pwm_actual_freq();
	if (actual > 0) pwmFrequency = actual;

	for (int i = 0; i < 4; i++) {
		print("  motor%d pin=%d freq=%dHz\n", i, motorPins[i], pwmFrequency);
	}

	sendMotors();
	print("Motors initialized\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0.0f, 1.0f);
	if (pwmMax >= 0) { // pwm 时间模式（接 ESC 用）
		float pwm = mapf(value, 0, 1, (float)pwmMin, (float)pwmMax);
		if (value == 0) pwm = (float)pwmStop;
		float duty = mapf(pwm, 0, 1000000.0f / (float)pwmFrequency, 0, (float)((1 << pwmResolution) - 1));
		return (int)roundf(duty);
	} else { // 纯占空比模式（接 MOSFET 直驱用）
		return (int)roundf(value * (float)((1 << pwmResolution) - 1));
	}
}

void sendMotors() {
	for (int i = 0; i < 4; i++) {
		hal_motor_pwm_write(MOTOR_CHANNEL_BASE + i, (uint32_t)getDutyCycle(motors[i]));
	}
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n) {
	print("Testing motor %d\n", n);
	motors[n] = 0.3f;
	hal_delay_ms(50); // 等待当前 PWM 周期结束再改占空比
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}
