// RAM日志记录
//
// 移植说明（ESP-IDF）：
//   * logBuffer 由 logBuffer[LOG_SIZE][logColumns] 改为行优先展开的一维数组
//     （定义见 app/globals.h），供 mavlink 以连续内存块发送。
//   * 其余逻辑逐行保持不变。

#include "app/log.h"

#include "app/cli.h"
#include "app/globals.h"
#include "app/math/util.h"
#include "app/math/vector.h"

Vector attitudeEuler;
Vector attitudeTargetEuler;   // 定义见下方 logEntries 的引用

struct LogEntry {
	const char *name;
	float *value;
};

static LogEntry logEntries[] = {
	{"t", &t},
	{"rates.x", &rates.x},
	{"rates.y", &rates.y},
	{"rates.z", &rates.z},
	{"ratesTarget.x", &ratesTarget.x},
	{"ratesTarget.y", &ratesTarget.y},
	{"ratesTarget.z", &ratesTarget.z},
	{"attitude.x", &attitudeEuler.x},
	{"attitude.y", &attitudeEuler.y},
	{"attitude.z", &attitudeEuler.z},
	{"attitudeTarget.x", &attitudeTargetEuler.x},
	{"attitudeTarget.y", &attitudeTargetEuler.y},
	{"attitudeTarget.z", &attitudeTargetEuler.z},
	{"thrustTarget", &thrustTarget}
};

static_assert(sizeof(logEntries) / sizeof(logEntries[0]) == LOG_COLUMNS,
              "LOG_COLUMNS 必须与 logEntries 项数一致");

float logBuffer[LOG_SIZE * LOG_COLUMNS];

static int logPointer = 0;
static int logFilled = 0;   // 已写入的行数（最多 LOG_SIZE）

int logRowCount() {
	return logFilled;
}

void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}

void logData() {
	if (!armed) return;
	static Rate period(LOG_RATE);
	if (!period) return;

	prepareLogData();

	for (int i = 0; i < LOG_COLUMNS; i++) {
		logBuffer[logPointer * LOG_COLUMNS + i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) {
		logPointer = 0;
	}
	if (logFilled < LOG_SIZE) logFilled++;
}

void printLogHeader() {
	for (int i = 0; i < LOG_COLUMNS; i++) {
		print("%s%s", logEntries[i].name, i < LOG_COLUMNS - 1 ? "," : "\n");
	}
}

void printLogData() {
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i * LOG_COLUMNS] == 0) continue; // 跳过空记录
		for (int j = 0; j < LOG_COLUMNS; j++) {
			print("%g%s", logBuffer[i * LOG_COLUMNS + j], j < LOG_COLUMNS - 1 ? "," : "\n");
		}
	}
}
