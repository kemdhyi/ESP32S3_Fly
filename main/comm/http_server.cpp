// 极简 HTTP 服务器实现

#include "comm/http_server.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "sdkconfig.h"

static const char* TAG = "http";

// 由 comm/web_rc.cpp 提供的路由表
extern const HttpRoute webRcRoutes[];
extern const size_t webRcRouteCount;

static constexpr int LISTEN_BACKLOG = 4;
static constexpr int RECV_TIMEOUT_MS = 2000;

// 读取一行（以 \n 结尾），去除末尾的 \r。返回读到的字节数，超时/关闭返回 -1。
static int readLine(int fd, char* buf, size_t cap) {
	size_t n = 0;
	while (n + 1 < cap) {
		char c;
		int r = (int)recv(fd, &c, 1, 0);
		if (r <= 0) return -1;
		if (c == '\n') {
			if (n > 0 && buf[n - 1] == '\r') n--;
			buf[n] = '\0';
			return (int)n;
		}
		buf[n++] = c;
	}
	buf[cap - 1] = '\0';
	return (int)(cap - 1);
}

// 解析请求行："GET /console?since=5 HTTP/1.1"
static bool parseRequestLine(const char* line, HttpRequest& req) {
	const char* sp1 = strchr(line, ' ');
	if (sp1 == nullptr) return false;
	size_t methodLen = (size_t)(sp1 - line);
	if (methodLen == 0 || methodLen >= sizeof(req.method)) return false;
	memcpy(req.method, line, methodLen);
	req.method[methodLen] = '\0';

	const char* target = sp1 + 1;
	const char* sp2 = strchr(target, ' ');
	size_t targetLen = sp2 ? (size_t)(sp2 - target) : strlen(target);
	if (targetLen >= sizeof(req.path) + sizeof(req.query)) return false;

	// 拆分 path 与 query
	const char* q = (const char*)memchr(target, '?', targetLen);
	if (q != nullptr) {
		size_t pathLen = (size_t)(q - target);
		if (pathLen >= sizeof(req.path)) pathLen = sizeof(req.path) - 1;
		memcpy(req.path, target, pathLen);
		req.path[pathLen] = '\0';

		size_t queryLen = targetLen - pathLen - 1;
		if (queryLen >= sizeof(req.query)) queryLen = sizeof(req.query) - 1;
		memcpy(req.query, q + 1, queryLen);
		req.query[queryLen] = '\0';
	} else {
		size_t pathLen = targetLen;
		if (pathLen >= sizeof(req.path)) pathLen = sizeof(req.path) - 1;
		memcpy(req.path, target, pathLen);
		req.path[pathLen] = '\0';
		req.query[0] = '\0';
	}
	return true;
}

static void handleConnection(int fd) {
	HttpRequest req = {};
	req.fd = fd;

	// ---- 请求行 ----
	char line[HTTP_MAX_LINE];
	if (readLine(fd, line, sizeof(line)) <= 0) return;
	if (!parseRequestLine(line, req)) {
		httpSend(fd, 400, "application/json", "{\"e\":\"bad request line\"}");
		return;
	}

	// ---- 请求头 ----
	size_t contentLength = 0;
	for (int i = 0; i < 32; i++) {
		int n = readLine(fd, line, sizeof(line));
		if (n <= 0) break;
		if (n == 0) break; // 空行 = 头部结束

		if (strncasecmp(line, "Content-Length:", 15) == 0) {
			contentLength = (size_t)strtoul(line + 15, nullptr, 10);
		} else if (strncasecmp(line, "Content-Type:", 13) == 0) {
			const char* v = line + 13;
			while (*v == ' ') v++;
			strlcpy(req.contentType, v, sizeof(req.contentType));
		}
	}

	// ---- 请求体 ----
	if (contentLength > 0) {
		size_t want = contentLength;
		if (want > sizeof(req.body) - 1) want = sizeof(req.body) - 1;
		size_t got = 0;
		while (got < want) {
			int r = (int)recv(fd, req.body + got, want - got, 0);
			if (r <= 0) break;
			got += (size_t)r;
		}
		// 丢弃多余部分，避免连接数据残留
		while (got < contentLength) {
			char discard[64];
			size_t chunk = contentLength - got;
			if (chunk > sizeof(discard)) chunk = sizeof(discard);
			int r = (int)recv(fd, discard, chunk, 0);
			if (r <= 0) break;
			got += (size_t)r;
		}
		req.bodyLen = (got < want) ? got : want;
		req.body[req.bodyLen] = '\0';
	} else {
		req.body[0] = '\0';
	}

	// ---- 路由匹配 ----
	for (size_t i = 0; i < webRcRouteCount; i++) {
		const HttpRoute& route = webRcRoutes[i];
		if (strcmp(route.path, req.path) == 0 &&
		    strcasecmp(route.method, req.method) == 0) {
			route.handler(req);
			return;
		}
	}

	httpSend(fd, 404, "application/json", "{\"e\":\"not found\"}");
}

void httpSend(int fd, int status, const char* contentType, const char* body) {
	if (fd < 0) return;

	const char* reason = "OK";
	switch (status) {
		case 200: reason = "OK"; break;
		case 301: reason = "Moved Permanently"; break;
		case 400: reason = "Bad Request"; break;
		case 404: reason = "Not Found"; break;
		case 503: reason = "Service Unavailable"; break;
		default:  reason = "OK"; break;
	}

	const size_t bodyLen = (body != nullptr) ? strlen(body) : 0;

	char header[256];
	int headerLen = snprintf(header, sizeof(header),
		"HTTP/1.1 %d %s\r\n"
		"Content-Type: %s\r\n"
		"Content-Length: %u\r\n"
		"Cache-Control: no-store\r\n"
		"Connection: close\r\n"
		"\r\n",
		status, reason,
		contentType != nullptr ? contentType : "text/plain",
		(unsigned)bodyLen);

	if (headerLen > 0) send(fd, header, (size_t)headerLen, 0);
	if (bodyLen > 0) send(fd, body, bodyLen, 0);
}

bool httpArgInt(const HttpRequest& req, const char* name, int* out) {
	if (name == nullptr || out == nullptr) return false;
	const size_t nameLen = strlen(name);

	const char* p = req.query;
	while (*p != '\0') {
		// 定位 "name=" 起始
		if (strncmp(p, name, nameLen) == 0 && p[nameLen] == '=') {
			*out = (int)strtol(p + nameLen + 1, nullptr, 10);
			return true;
		}
		// 跳到下一个 '&'
		const char* amp = strchr(p, '&');
		if (amp == nullptr) break;
		p = amp + 1;
	}
	return false;
}

// HTTP 处理任务
static void httpTask(void* arg) {
	int listenFd = (int)(intptr_t)arg;

	for (;;) {
		struct sockaddr_in client = {};
		socklen_t clientLen = sizeof(client);
		int fd = accept(listenFd, (struct sockaddr*)&client, &clientLen);
		if (fd < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				vTaskDelay(pdMS_TO_TICKS(5));
				continue;
			}
			ESP_LOGW(TAG, "accept failed: errno=%d", errno);
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}

		// 设置接收超时，避免半开连接永久占用任务
		struct timeval tv = {};
		tv.tv_sec = RECV_TIMEOUT_MS / 1000;
		tv.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

		handleConnection(fd);
		close(fd);

		// 让出 CPU，避免饥饿同核任务（WiFi 协议栈在 core 0）
		taskYIELD();
	}
}

static int createListener(uint16_t port) {
	int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
		return -1;
	}

	int reuse = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	struct sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		ESP_LOGE(TAG, "bind(%u) failed: errno=%d", (unsigned)port, errno);
		close(fd);
		return -1;
	}
	if (listen(fd, LISTEN_BACKLOG) < 0) {
		ESP_LOGE(TAG, "listen(%u) failed: errno=%d", (unsigned)port, errno);
		close(fd);
		return -1;
	}
	return fd;
}

void httpServerStart(uint16_t port) {
	if (port == 0) port = CONFIG_CFDRONE_HTTP_PORT;

	int listenFd = createListener(port);
	if (listenFd < 0) {
		ESP_LOGE(TAG, "HTTP server failed to start on port %u", (unsigned)port);
		return;
	}

	BaseType_t ok = xTaskCreatePinnedToCore(
		httpTask, "http_srv", 6144, (void*)(intptr_t)listenFd,
		5, nullptr, CONFIG_CFDRONE_HTTP_TASK_CORE);
	if (ok != pdPASS) {
		ESP_LOGE(TAG, "failed to create HTTP task");
		close(listenFd);
		return;
	}
	ESP_LOGI(TAG, "HTTP server listening on port %u (core %d)",
	         (unsigned)port, CONFIG_CFDRONE_HTTP_TASK_CORE);
}

// ---- 旧 PCB 丝印 :8080 -> 主端口 301 跳转 ----

static uint16_t s_redirectMainPort = 80;

static void redirectTask(void* arg) {
	int listenFd = (int)(intptr_t)arg;

	for (;;) {
		struct sockaddr_in client = {};
		socklen_t clientLen = sizeof(client);
		int fd = accept(listenFd, (struct sockaddr*)&client, &clientLen);
		if (fd < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				vTaskDelay(pdMS_TO_TICKS(20));
				continue;
			}
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}

		struct timeval tv = {};
		tv.tv_sec = 0;
		tv.tv_usec = 200000;   // 200ms，与原实现的自旋等待时长一致
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

		char line[HTTP_MAX_LINE];
		char path[96] = "/";
		char host[64] = "";

		if (readLine(fd, line, sizeof(line)) > 0) {
			// 解析 "GET /path HTTP/1.1"
			const char* sp1 = strchr(line, ' ');
			if (sp1 != nullptr) {
				const char* sp2 = strchr(sp1 + 1, ' ');
				size_t pathLen = sp2 ? (size_t)(sp2 - sp1 - 1) : strlen(sp1 + 1);
				if (pathLen >= sizeof(path)) pathLen = sizeof(path) - 1;
				memcpy(path, sp1 + 1, pathLen);
				path[pathLen] = '\0';
			}
			// 读取头部，提取 Host（去掉端口部分）
			for (int i = 0; i < 32; i++) {
				int n = readLine(fd, line, sizeof(line));
				if (n <= 0) break;
				if (n == 0) break;
				if (strncasecmp(line, "Host:", 5) == 0) {
					const char* v = line + 5;
					while (*v == ' ') v++;
					strlcpy(host, v, sizeof(host));
					char* colon = strchr(host, ':');
					if (colon != nullptr) *colon = '\0';
				}
			}
		}

		char body[192];
		int n = snprintf(body, sizeof(body),
			"HTTP/1.1 301 Moved Permanently\r\n"
			"Location: http://%s%s\r\n"
			"Connection: close\r\n"
			"Content-Length: 0\r\n"
			"\r\n",
			host, path);
		if (n > 0) send(fd, body, (size_t)n, 0);

		close(fd);
		taskYIELD();
	}
}

void httpRedirectStart(uint16_t port, uint16_t mainPort) {
	if (port == 0) return;
	if (port == mainPort) {
		ESP_LOGW(TAG, "redirect port equals main port, skipping");
		return;
	}

	int listenFd = createListener(port);
	if (listenFd < 0) {
		ESP_LOGW(TAG, "redirect server failed to start on port %u", (unsigned)port);
		return;
	}

	s_redirectMainPort = mainPort;

	BaseType_t ok = xTaskCreatePinnedToCore(
		redirectTask, "http_301", 3072, (void*)(intptr_t)listenFd,
		4, nullptr, CONFIG_CFDRONE_HTTP_TASK_CORE);
	if (ok != pdPASS) {
		ESP_LOGE(TAG, "failed to create redirect task");
		close(listenFd);
		return;
	}
	ESP_LOGI(TAG, "HTTP redirect %u -> %u enabled", (unsigned)port, (unsigned)mainPort);
}
