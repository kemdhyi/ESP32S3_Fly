// 网页遥控服务端
//
// 移植说明（ESP-IDF）：
//   * Arduino WebServer -> comm/http_server.h（BSD socket + 独立任务）
//   * String -> 定长 char 缓冲（消除堆分配与碎片）
//   * millis() -> hal_millis()
//   * 路由处理函数改为自由函数 + HttpRequest 参数
//   * ★ 线程安全：HTTP 处理器运行在 core 0，飞控主循环在 core 1，
//     因此对 webRCLastUpdate / webRCWarnMsg / 控制台队列的访问都加了临界区保护。
//   * readWebRC() 不再泵 HTTP（服务器已有独立任务），只刷新连接状态标志。

#include "hal/board.h"      // 先定义 WIFI_ENABLED / WEB_RC_ENABLED
#include "comm/web_rc.h"

#if WEB_RC_ENABLED

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "app/battery.h"
#include "app/cli.h"
#include "app/globals.h"
#include "app/math/util.h"
#include "comm/http_server.h"
#include "comm/web_rc_html.h"
#include "hal/hal_time.h"

// ==================== 配置常量 ====================
#define WEB_RC_TIMEOUT_MS    10000          // 连接超时：最后一次收包超过此时间(ms)视为断连

// ==================== 连接状态标志 ====================
bool webRCEnabled    = false;
bool useWebRC        = false;
bool webRCUpdated    = false;
bool webConsoleEnabled = false;
char webRCWarnMsg[64] = "";

// 跨核访问保护（spinlock，临界区极短）
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ==================== 最后处理的请求上下文 ====================
static int lastProcType        = 0;   // 1=摇杆 2=按钮 4=心跳
static int lastProcButtonIdx   = -1;
static int lastProcButtonState = -1;

// ==================== 摇杆暂存值 ====================
float webRCRoll     = 0.0f;
float webRCPitch    = 0.0f;
float webRCYaw      = 0.0f;
float webRCThrottle = 0.0f;
uint16_t webRCButtons    = 0;
unsigned long webRCLastUpdate = 0;  // millis() 时间戳

// ==================== 灵敏度缩放 ====================
float webRCThrottleScale = 1.0f;
float webRCStickScale    = 0.85f;
float webRCYawScale      = 0.68f;

// ==================== 死区 ====================
float stickDeadzone    = 0.06f;
float throttleDeadzone = 0.06f;

// ==================== 输入值域（勿随意修改，需与前端保持一致）====================
static const float THROTTLE_MIN = 0.0f;
static const float THROTTLE_MAX = 100.0f;
static const float STICK_MAX    = 30.0f;
static const float RAW_MAX      = 100.0f;

// ==================== 内部状态 ====================
static float lastValidThrottle = 0.0f;
static float lastValidRoll     = 0.0f;
static float lastValidPitch    = 0.0f;
static float lastValidYaw      = 0.0f;
static unsigned long lastDataErrorTime = 0;

// ==================== 控制台日志缓冲区 ====================
#define CONSOLE_LINES    BOARD_CONSOLE_LINES
#define CONSOLE_LINE_LEN BOARD_CONSOLE_LINE_LEN
static char consoleBuf[CONSOLE_LINES][CONSOLE_LINE_LEN];
static int  consoleTail   = 0;
static int  consoleFilled = 0;
static int  consoleTotal  = 0;

#define CONSOLE_CMD_QUEUE_SIZE 4
#define CONSOLE_CMD_LEN 64
static char consoleCmdQueue[CONSOLE_CMD_QUEUE_SIZE][CONSOLE_CMD_LEN];
static int  consoleCmdHead  = 0;
static int  consoleCmdTail  = 0;
static int  consoleCmdCount = 0;

bool enqueueConsoleCmd(const char* cmd) {
	if (cmd == nullptr || *cmd == '\0') return false;

	bool ok = false;
	portENTER_CRITICAL(&s_mux);
	if (consoleCmdCount < CONSOLE_CMD_QUEUE_SIZE) {
		strncpy(consoleCmdQueue[consoleCmdTail], cmd, CONSOLE_CMD_LEN - 1);
		consoleCmdQueue[consoleCmdTail][CONSOLE_CMD_LEN - 1] = '\0';
		consoleCmdTail = (consoleCmdTail + 1) % CONSOLE_CMD_QUEUE_SIZE;
		consoleCmdCount++;
		ok = true;
	}
	portEXIT_CRITICAL(&s_mux);
	return ok;
}

void processConsoleCommandQueue() {
	char cmd[CONSOLE_CMD_LEN];

	portENTER_CRITICAL(&s_mux);
	if (consoleCmdCount <= 0) {
		portEXIT_CRITICAL(&s_mux);
		return;
	}
	strncpy(cmd, consoleCmdQueue[consoleCmdHead], CONSOLE_CMD_LEN - 1);
	cmd[CONSOLE_CMD_LEN - 1] = '\0';
	consoleCmdHead = (consoleCmdHead + 1) % CONSOLE_CMD_QUEUE_SIZE;
	consoleCmdCount--;
	portEXIT_CRITICAL(&s_mux);

	doCommand(cmd, false);
}

void webLog(const char* msg) {
	if (msg == nullptr) return;
	// 按 \n 拆分写入，避免换行符污染 JSON
	const char* start = msg;
	while (*start != '\0') {
		const char* end = strchr(start, '\n');
		int len = end ? (int)(end - start) : (int)strlen(start);
		if (len > 0) {
			int copy = (len < CONSOLE_LINE_LEN - 1) ? len : (CONSOLE_LINE_LEN - 1);
			portENTER_CRITICAL(&s_mux);
			int slot = consoleTail;
			portEXIT_CRITICAL(&s_mux);

			strncpy(consoleBuf[slot], start, (size_t)copy);
			consoleBuf[slot][copy] = '\0';

			portENTER_CRITICAL(&s_mux);
			consoleTail = (consoleTail + 1) % CONSOLE_LINES;
			if (consoleFilled < CONSOLE_LINES) consoleFilled++;
			consoleTotal++;
			portEXIT_CRITICAL(&s_mux);
		}
		if (end == nullptr) break;
		start = end + 1;
	}
}

// ==================== 摇杆处理 ====================

float applyDeadzone(float norm, float deadzone) {
	if (fabsf(norm) < deadzone) return 0.0f;
	float sign = (norm > 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(norm) - deadzone) / (1.0f - deadzone);
}

float processAxis(float raw, float& lastValid) {
	if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) {
		if (hal_millis() - lastDataErrorTime > 2000) lastDataErrorTime = hal_millis();
		return lastValid;
	}
	float norm = constrain(raw, -RAW_MAX, RAW_MAX) / RAW_MAX;
	norm = applyDeadzone(norm, stickDeadzone);
	lastValid = norm * STICK_MAX;
	return lastValid;
}

float processThrottle(float raw) {
	if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) return lastValidThrottle;
	raw = constrain(raw, -RAW_MAX, RAW_MAX);
	float pct = (raw + RAW_MAX) / (2.0f * RAW_MAX) * THROTTLE_MAX;
	if (pct < throttleDeadzone * THROTTLE_MAX) pct = 0.0f;
	pct = constrain(pct * webRCThrottleScale, THROTTLE_MIN, THROTTLE_MAX);
	lastValidThrottle = pct;
	return pct;
}

// ==================== 核心数据处理 ====================

void setWebRCInput(float roll, float pitch, float yaw, float throttle, uint16_t buttons) {
	float pThrottle = processThrottle(throttle);
	float pYaw      = processAxis(yaw,   lastValidYaw);
	float pPitch    = processAxis(pitch, lastValidPitch);
	float pRoll     = processAxis(roll,  lastValidRoll);

	// 暂存处理后的值（供状态端点读取）
	webRCThrottle = pThrottle;
	webRCYaw      = pYaw;
	webRCPitch    = pPitch;
	webRCRoll     = pRoll;
	webRCButtons  = buttons;
	webRCLastUpdate = hal_millis();
	webRCUpdated  = true;

	// 写入统一控制变量（与 SBUS/MAVLink 同路径）
	controlRoll     = constrain(pRoll  * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
	controlPitch    = constrain(pPitch * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
	controlYaw      = constrain(pYaw   * webRCYawScale   / STICK_MAX, -1.0f, 1.0f);
	controlThrottle = pThrottle / THROTTLE_MAX;
	controlMode     = NAN;
	controlTime     = t;

	static float lastPrintedThrottle = -1.0f;
	if (fabsf(pThrottle - lastPrintedThrottle) > 5.0f) {
		print("WebRC T=%.0f%% R=%.1f P=%.1f Y=%.1f Btn=0x%04X\n",
		      pThrottle, pRoll, pPitch, pYaw, buttons);
		lastPrintedThrottle = pThrottle;
	}
}

// ==================== JSON 协议处理器 ====================

const char* findJsonValue(const char* json, const char* key) {
	const char* p = strstr(json, key);
	if (p == nullptr) return nullptr;
	p = strchr(p, ':');
	if (p == nullptr) return nullptr;
	p++;
	while (*p == ' ' || *p == '"') p++;
	return p;
}

bool handleJSONProtocol(const char* body) {
	if (body == nullptr || body[0] == '\0' || strchr(body, '{') == nullptr) return false;
	const char* json = body;
	const char* typePos = strstr(json, "\"t\":");
	if (typePos == nullptr) return false;
	int type = atoi(typePos + 4);
	const char* v;

	lastProcType        = type;
	lastProcButtonIdx   = -1;
	lastProcButtonState = -1;

	switch (type) {
		case 1: { // 摇杆数据
			float th = 0, r = 0, p = 0, y = 0;
			if ((v = findJsonValue(json, "\"th\""))) th = strtof(v, nullptr);
			if ((v = findJsonValue(json, "\"r\"")))  r  = strtof(v, nullptr);
			if ((v = findJsonValue(json, "\"p\"")))  p  = strtof(v, nullptr);
			if ((v = findJsonValue(json, "\"y\"")))  y  = strtof(v, nullptr);
			setWebRCInput(r, p, y, th, webRCButtons);
			break;
		}
		case 2: { // 按钮事件
			int idx = 0, state = 0;
			if ((v = findJsonValue(json, "\"b\"")))  idx   = atoi(v);
			if ((v = findJsonValue(json, "\"s\"")))  state = atoi(v);
			if (idx >= 0 && idx < 16) {
				if (state) webRCButtons |=  (uint16_t)(1 << idx);
				else       webRCButtons &= (uint16_t)~(1 << idx);
				webRCLastUpdate = hal_millis();
				webRCUpdated    = true;
				lastProcButtonIdx   = idx;
				lastProcButtonState = state;
			}
			break;
		}
		case 4: // 心跳
			webRCLastUpdate = hal_millis();
			webRCUpdated    = true;
			break;
		default:
			break;
	}
	return true;
}

void setWebRCWarn(const char* msg) {
	portENTER_CRITICAL(&s_mux);
	strncpy(webRCWarnMsg, msg, sizeof(webRCWarnMsg) - 1);
	webRCWarnMsg[sizeof(webRCWarnMsg) - 1] = '\0';
	portEXIT_CRITICAL(&s_mux);
}

void clearWebRCWarn() {
	portENTER_CRITICAL(&s_mux);
	webRCWarnMsg[0] = '\0';
	portEXIT_CRITICAL(&s_mux);
}

// ==================== 连接状态 ====================

bool isWebRCEnabled() {
	return webRCUpdated && (hal_millis() - webRCLastUpdate < WEB_RC_TIMEOUT_MS);
}

bool isUsingWebRC() {
	return useWebRC && isWebRCEnabled();
}

// ==================== HTTP 路由处理器 ====================

// GET / —— 内置网页遥控器界面
extern "C" void routeIndex(const HttpRequest& req) {
	httpSend(req.fd, 200, "text/html; charset=utf-8", webRCIndexHtml);
}

// POST /web_rc, POST /web_rc/heartbeat
extern "C" void routeWebRC(const HttpRequest& req) {
	if (req.bodyLen == 0) {
		httpSend(req.fd, 400, "application/json", "{\"e\":\"no data\"}");
		return;
	}

	if (!handleJSONProtocol(req.body)) {
		httpSend(req.fd, 400, "application/json", "{\"e\":\"parse failed\"}");
		return;
	}

	char resp[320];
	// warn 在按钮事件（rt=2）和心跳包（rt=4）响应中携带并清除：
	//   rt=2：用户主动操作（解锁被拒等），需要即时反馈
	//   rt=4：系统自动触发（低电自动上锁等），心跳周期≈2s，延迟可接受
	// 摇杆包（rt=1）不消费 warn，避免抢在 state=0 响应之前把 warn 清掉
	bool deliverWarn = (lastProcType == 2 || lastProcType == 4) && webRCWarnMsg[0] != '\0';

	if (deliverWarn) {
		if (lastProcButtonIdx >= 0) {
			snprintf(resp, sizeof(resp),
				"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d,\"warn\":\"%s\"}",
				mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState, webRCWarnMsg);
		} else {
			snprintf(resp, sizeof(resp),
				"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"warn\":\"%s\"}",
				mode, (int)armed, lastProcType, webRCWarnMsg);
		}
		clearWebRCWarn(); // 发送后立即清空
	} else {
		if (lastProcButtonIdx >= 0) {
			snprintf(resp, sizeof(resp),
				"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d}",
				mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState);
		} else {
			snprintf(resp, sizeof(resp),
				"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d}",
				mode, (int)armed, lastProcType);
		}
	}
	httpSend(req.fd, 200, "application/json", resp);
}

// GET /web_rc/status
extern "C" void routeStatus(const HttpRequest& req) {
	float vbat = readBatteryVoltage();
	if (isnan(vbat) || vbat < 0.0f) vbat = 0.0f;
	char json[384];
	snprintf(json, sizeof(json),
		"{\"enabled\":%s,\"active\":%s,"
		"\"voltage\":%.2f,"
		"\"throttle\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
		isWebRCEnabled() ? "true" : "false",
		isUsingWebRC()   ? "true" : "false",
		vbat,
		webRCThrottle, webRCRoll, webRCPitch, webRCYaw);
	httpSend(req.fd, 200, "application/json", json);
}

// 按字节写出一个 JSON 转义后的字符串（替代 String::replace 链，避免堆分配）
// 返回写入后的新位置
static char* appendJsonEscaped(char* out, char* end, const char* src) {
	while (*src != '\0' && out < end - 7) {
		char c = *src++;
		switch (c) {
			case '\\': *out++ = '\\'; *out++ = '\\'; break;
			case '"':  *out++ = '\\'; *out++ = '"';  break;
			case '\n': *out++ = '\\'; *out++ = 'n';  break;
			case '\r': *out++ = '\\'; *out++ = 'r';  break;
			case '\t': *out++ = '\\'; *out++ = 't';  break;
			default:   *out++ = c; break;
		}
	}
	*out = '\0';
	return out;
}

// GET /console?since=&limit=
extern "C" void routeConsole(const HttpRequest& req) {
	int since = -1;
	int limit = 20;
	httpArgInt(req, "since", &since);
	httpArgInt(req, "limit", &limit);
	if (limit <= 0) limit = 20;
	if (limit > CONSOLE_LINES) limit = CONSOLE_LINES;

	int filled, total;
	portENTER_CRITICAL(&s_mux);
	filled = consoleFilled;
	total  = consoleTotal;
	portEXIT_CRITICAL(&s_mux);

	// 与 Arduino 版一致：环形缓冲以「绝对行号 % CONSOLE_LINES」索引
	int availableFrom = total - filled;
	if (availableFrom < 0) availableFrom = 0;
	int sendFrom = (since >= 0) ? since : availableFrom;
	if (sendFrom < availableFrom) sendFrom = availableFrom;
	if (sendFrom > total) sendFrom = total;
	int sendTo = total;
	if (sendTo > sendFrom + limit) sendTo = sendFrom + limit;

	static char json[HTTP_MAX_RESPONSE];
	char* out = json;
	char* end = json + sizeof(json) - 4;

	out += snprintf(out, (size_t)(end - out),
	                "{\"total\":%d,\"next\":%d,\"has_more\":%s,\"lines\":[",
	                total, sendTo, (sendTo < total) ? "true" : "false");

	bool first = true;
	for (int i = sendFrom; i < sendTo && out < end - 8; i++) {
		int idx = i % CONSOLE_LINES;
		if (idx < 0) idx += CONSOLE_LINES;

		if (!first) *out++ = ',';
		first = false;
		*out++ = '"';
		*out = '\0';
		out = appendJsonEscaped(out, end, consoleBuf[idx]);
		if (out < end - 2) *out++ = '"';
	}
	out += snprintf(out, (size_t)(end - out), "]}");
	if (out < json + sizeof(json) - 1) *out = '\0';
	else json[sizeof(json) - 1] = '\0';

	httpSend(req.fd, 200, "application/json", json);
}

// POST /console/cmd
extern "C" void routeConsoleCmd(const HttpRequest& req) {
	// 复制并去除首尾空白
	char cmd[CONSOLE_CMD_LEN];
	strncpy(cmd, req.body, sizeof(cmd) - 1);
	cmd[sizeof(cmd) - 1] = '\0';

	char* p = cmd;
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
	char* q = p + strlen(p);
	while (q > p && (q[-1] == ' ' || q[-1] == '\t' || q[-1] == '\r' || q[-1] == '\n')) *--q = '\0';

	if (*p == '\0') {
		httpSend(req.fd, 400, "application/json", "{\"ok\":0,\"e\":\"empty command\"}");
		return;
	}

	char echo[CONSOLE_LINE_LEN];
	snprintf(echo, sizeof(echo), "> %s", p);
	webLog(echo);

	if (!enqueueConsoleCmd(p)) {
		webLog("! command queue is full");
		httpSend(req.fd, 503, "application/json", "{\"ok\":0,\"e\":\"queue full\"}");
		return;
	}

	httpSend(req.fd, 200, "application/json", "{\"ok\":1,\"queued\":1}");
}

// POST /console/enable
extern "C" void routeConsoleEnable(const HttpRequest& req) {
	webConsoleEnabled = true;
	httpSend(req.fd, 200, "application/json", "{\"ok\":1}");
}

// POST /console/disable
extern "C" void routeConsoleDisable(const HttpRequest& req) {
	webConsoleEnabled = false;
	httpSend(req.fd, 200, "application/json", "{\"ok\":1}");
}

// 路由表（供 comm/http_server.cpp 引用）
extern "C" const HttpRoute webRcRoutes[] = {
	{"/",                 "GET",  routeIndex},
	{"/web_rc",           "POST", routeWebRC},
	{"/web_rc/heartbeat", "POST", routeWebRC},
	{"/web_rc/status",    "GET",  routeStatus},
	{"/console",          "GET",  routeConsole},
	{"/console/cmd",      "POST", routeConsoleCmd},
	{"/console/enable",   "POST", routeConsoleEnable},
	{"/console/disable",  "POST", routeConsoleDisable},
};
extern "C" const size_t webRcRouteCount = sizeof(webRcRoutes) / sizeof(webRcRoutes[0]);

// ==================== 主设置函数 ====================

void setupWebRC() {
	print("Setup WEB RC\n");
	lastValidThrottle = THROTTLE_MIN;
	lastValidRoll = lastValidPitch = lastValidYaw = 0.0f;

	// HTTP 服务器在独立任务中运行（默认 core 0）
	httpServerStart(CONFIG_CFDRONE_HTTP_PORT);

	// 旧 PCB 丝印指向 :8080，提供 301 跳转
	httpRedirectStart(CONFIG_CFDRONE_HTTP_REDIRECT_PORT, CONFIG_CFDRONE_HTTP_PORT);

	print("✓ Web RC 已启动，访问地址: http://192.168.4.1\n");
	print("  死区 摇杆=%.0f%% 油门=%.0f%% | 缩放 摇杆=%.2f 偏航=%.2f\n",
	      stickDeadzone * 100.0f, throttleDeadzone * 100.0f,
	      webRCStickScale, webRCYawScale);
}

// ==================== 主循环函数 ====================

void readWebRC() {
	// HTTP 服务器已由独立任务处理，这里只刷新连接状态标志
	if (isWebRCEnabled()) {
		webRCEnabled = useWebRC = true;
	} else {
		webRCEnabled = useWebRC = false;
	}
}

#endif  // WEB_RC_ENABLED
