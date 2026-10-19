// MAVLink通讯
//
// 移植说明（ESP-IDF）：
//   * #include <MAVLink.h>（Arduino 库）-> <mavlink_helpers.h> + common 方言头，
//     由 components/mavlink 组件提供（用 pymavlink mavgen 生成）。
//   * String mavlinkPrintBuffer -> 定长环形缓冲（避免堆分配与碎片）。
//   * 其余打包/解包逻辑与 Arduino 版逐行一致（MAVLink 是纯 C 库）。

#include "hal/board.h"      // 先定义 WIFI_ENABLED / WEB_RC_ENABLED

#if WIFI_ENABLED && CONFIG_CFDRONE_MAVLINK_ENABLED

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "app/cli.h"
#include "app/globals.h"
#include "app/log.h"
#include "app/math/util.h"
#include "app/parameters.h"
#include "app/rc.h"
#include "comm/wifi.h"

extern "C" {
#include "comm/mavlink_compat.h"
}

// 前向声明：MAVLink 相关函数在下方定义，processMavlink() 先引用它们
// （原 .ino 由 Arduino IDE 自动生成前置声明，标准 C++ 需显式给出）
void sendMavlink();
void receiveMavlink();
void sendMavlinkPrint();
void sendMessage(const void* msg);
void handleMavlink(const void* msg);

// MAVLink 配置变量（mavlinkSysId / telemetry*Rate / mavlinkConnected）
// 与遥测限速器（telemetrySlow / telemetryFast）均定义于 app/comm_config.cpp，
// 以便在关闭 MAVLink 时参数表仍可正常引用。

// ============================================================
// 串口转发缓冲（替代 Arduino String，定长环形缓冲）
// ============================================================
static constexpr size_t PRINT_BUF_SIZE = 1024;
static char printBuf[PRINT_BUF_SIZE];
static size_t printLen = 0;

static void printBufAppend(const char* str) {
	size_t n = strlen(str);
	if (n > PRINT_BUF_SIZE - 1 - printLen) {
		n = (printLen < PRINT_BUF_SIZE - 1) ? (PRINT_BUF_SIZE - 1 - printLen) : 0;
	}
	if (n > 0) {
		memcpy(printBuf + printLen, str, n);
		printLen += n;
		printBuf[printLen] = '\0';
	}
}

static void printBufClear() {
	printLen = 0;
	printBuf[0] = '\0';
}

void processMavlink() {
	// 参数表里的频率可在运行时被修改，这里同步到 Rate 对象
	telemetrySlow.rate = telemetrySlowRate;
	telemetryFast.rate = telemetryFastRate;

	sendMavlink();
	receiveMavlink();
}

void sendMavlink() {
	sendMavlinkPrint();

	mavlink_message_t msg;
	uint32_t time = (uint32_t)(t * 1000);

	if (telemetrySlow) {
		mavlink_msg_heartbeat_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
			(armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
			((mode == STAB) ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0) |
			((mode == AUTO) ? MAV_MODE_FLAG_AUTO_ENABLED : MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
			mode, MAV_STATE_STANDBY);
		sendMessage(&msg);

		if (!mavlinkConnected) return; // send only heartbeat until connected

		mavlink_msg_extended_sys_state_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			MAV_VTOL_STATE_UNDEFINED, landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR);
		sendMessage(&msg);
	}

	if (telemetryFast && mavlinkConnected) {
		const float offset[] = {0, 0, 0, 0};
		mavlink_msg_attitude_quaternion_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			time, attitude.w, attitude.x, -attitude.y, -attitude.z, rates.x, -rates.y, -rates.z, offset); // convert to frd
		sendMessage(&msg);

		mavlink_msg_rc_channels_raw_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			(uint32_t)(controlTime * 1000), 0,
			channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6], channels[7], UINT8_MAX);
		if (channels[0] != 0) sendMessage(&msg); // 0 means no RC input

		float controls[8] = {0};
		memcpy(controls, motors, sizeof(float) * 4);
		mavlink_msg_actuator_control_target_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time, 0, controls);
		sendMessage(&msg);

		mavlink_msg_scaled_imu_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time,
			acc.x / ONE_G * 1000, -acc.y / ONE_G * 1000, -acc.z / ONE_G * 1000, // convert to frd
			gyro.x * 1000, -gyro.y * 1000, -gyro.z * 1000,
			0, 0, 0, 0);
		sendMessage(&msg);
	}
}

void sendMessage(const void *msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buf, (mavlink_message_t *)msg);
	sendWiFi(buf, len);
}

void receiveMavlink() {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = receiveWiFi(buf, MAVLINK_MAX_PACKET_LEN);
	if (len) mavlinkConnected = true;

	// New packet, parse it
	mavlink_message_t msg;
	mavlink_status_t status;
	for (int i = 0; i < len; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
			handleMavlink(&msg);
		}
	}
}

void handleMavlink(const void *_msg) {
	const mavlink_message_t& msg = *(mavlink_message_t *)_msg;

	if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
		mavlink_manual_control_t m;
		mavlink_msg_manual_control_decode(&msg, &m);
		if (m.target && m.target != mavlinkSysId) return; // 0 is broadcast

		controlThrottle = m.z / 1000.0f;
		controlPitch = m.x / 1000.0f;
		controlRoll = m.y / 1000.0f;
		controlYaw = m.r / 1000.0f;
		controlMode = NAN;
		controlTime = t;
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
		mavlink_param_request_list_t m;
		mavlink_msg_param_request_list_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		mavlink_message_t out;
		for (int i = 0; i < parametersCount(); i++) {
			mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
				getParameterName(i), getParameter(i), MAV_PARAM_TYPE_REAL32, parametersCount(), i);
			sendMessage(&out);
		}
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
		mavlink_param_request_read_t m;
		mavlink_msg_param_request_read_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char name[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN + 1];
		strlcpy(name, m.param_id, sizeof(name)); // param_id might be not null-terminated
		float value = strlen(name) == 0 ? getParameter(m.param_index) : getParameter(name);
		if (m.param_index != -1) {
			memcpy(name, getParameterName(m.param_index), MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
			name[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = '\0';
		}
		mavlink_message_t out;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
			name, value, MAV_PARAM_TYPE_REAL32, parametersCount(), m.param_index);
		sendMessage(&out);
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
		mavlink_param_set_t m;
		mavlink_msg_param_set_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1];
		strlcpy(name, m.param_id, sizeof(name)); // param_id might be not null-terminated
		bool success = setParameter(name, m.param_value);
		if (!success) return;
		// send ack
		mavlink_message_t out;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
			m.param_id, getParameter(name), MAV_PARAM_TYPE_REAL32, parametersCount(), 0); // index is unknown
		sendMessage(&out);
	}

	if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) { // handle to make qgc happy
		mavlink_mission_request_list_t m;
		mavlink_msg_mission_request_list_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		mavlink_message_t out;
		mavlink_msg_mission_count_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
			0, 0, 0, (uint8_t)MAV_MISSION_TYPE_MISSION);
		sendMessage(&out);
	}

	if (msg.msgid == MAVLINK_MSG_ID_SERIAL_CONTROL) {
		mavlink_serial_control_t m;
		mavlink_msg_serial_control_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char data[MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN + 1];
		size_t copy = m.count;
		if (copy > MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN) copy = MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN;
		memcpy(data, m.data, copy);
		data[copy] = '\0';
		doCommand(data, true);
	}

	if (msg.msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET) {
		if (mode != AUTO) return;

		mavlink_set_attitude_target_t m;
		mavlink_msg_set_attitude_target_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		// copy attitude, rates and thrust targets
		ratesTarget.x = m.body_roll_rate;
		ratesTarget.y = -m.body_pitch_rate; // convert to flu
		ratesTarget.z = -m.body_yaw_rate;
		attitudeTarget.w = m.q[0];
		attitudeTarget.x = m.q[1];
		attitudeTarget.y = -m.q[2];
		attitudeTarget.z = -m.q[3];
		thrustTarget = m.thrust;
		ratesExtra = Vector(0, 0, 0);

		if (m.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE) attitudeTarget.invalidate();
		armed = m.thrust > 0;
	}

	if (msg.msgid == MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET) {
		if (mode != AUTO) return;

		mavlink_set_actuator_control_target_t m;
		mavlink_msg_set_actuator_control_target_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		attitudeTarget.invalidate();
		ratesTarget.invalidate();
		torqueTarget.invalidate();
		memcpy(motors, m.controls, sizeof(float) * 4); // copy motor thrusts
		armed = motors[0] > 0 || motors[1] > 0 || motors[2] > 0 || motors[3] > 0;
	}

	if (msg.msgid == MAVLINK_MSG_ID_LOG_REQUEST_DATA) {
		mavlink_log_request_data_t m;
		mavlink_msg_log_request_data_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		// Send all log records
		const int rows = logRowCount();
		for (int i = 0; i < rows; i++) {
			mavlink_message_t out;
			mavlink_msg_log_data_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out, 0, i,
				sizeof(float) * LOG_COLUMNS,
				(const uint8_t *)(logBuffer + (size_t)i * LOG_COLUMNS));
			sendMessage(&out);
		}
	}

	// Handle commands
	if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
		mavlink_command_long_t m;
		mavlink_msg_command_long_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		mavlink_message_t response;
		bool accepted = false;

		if (m.command == MAV_CMD_REQUEST_MESSAGE && m.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
			accepted = true;
			mavlink_msg_autopilot_version_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &response,
				MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT | MAV_PROTOCOL_CAPABILITY_MAVLINK2,
				1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
			sendMessage(&response);
		}

		if (m.command == MAV_CMD_COMPONENT_ARM_DISARM) {
			if (m.param1 && controlThrottle > 0.05) return; // don't arm if throttle is not low
			accepted = true;
			armed = m.param1 == 1;
		}

		if (m.command == MAV_CMD_DO_SET_MODE) {
			if (m.param2 < 0 || m.param2 > AUTO) return; // incorrect mode
			accepted = true;
			mode = (int)m.param2;
		}

		// send command ack
		mavlink_message_t ack;
		mavlink_msg_command_ack_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &ack, m.command,
			accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, msg.sysid, msg.compid);
		sendMessage(&ack);
	}
}

// 将 CLI 输出转发到地面站（SERIAL_CONTROL）
void mavlinkPrint(const char* str) {
	printBufAppend(str);
}

void sendMavlinkPrint() {
	if (printLen == 0) return;

	// 分块发送
	const char* str = printBuf;
	const size_t total = printLen;
	for (size_t i = 0; i < total; i += MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN) {
		char data[MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN + 1];
		size_t copy = total - i;
		if (copy > MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN) copy = MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN;
		memcpy(data, str + i, copy);
		data[copy] = '\0';

		mavlink_message_t msg;
		mavlink_msg_serial_control_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			SERIAL_CONTROL_DEV_SHELL,
			(i + MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN < total) ? SERIAL_CONTROL_FLAG_MULTI : 0,
			0, 0, (uint8_t)copy, (const uint8_t *)data, 0, 0);
		sendMessage(&msg);
	}
	printBufClear();
}

#endif  // WIFI_ENABLED && CONFIG_CFDRONE_MAVLINK_ENABLED
