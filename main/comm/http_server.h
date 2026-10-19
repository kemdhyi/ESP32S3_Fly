// 极简 HTTP 服务器（替代 Arduino 的 WebServer.h）
//
// 设计要点：
//   * 运行在独立的 FreeRTOS 任务（默认 core 0），与飞控主循环（core 1）分离。
//     这是相对 Arduino 版的重要改进：原版在 loop() 里调用 webRCServer.handleClient()，
//     HTTP 处理会直接占用飞控时间片。
//   * 不支持长连接/分块传输/keep-alive 复用；每次请求一个连接、发完即关。
//     对本项目的 20Hz 摇杆包 + 低频轮询完全够用。
//   * 不使用 String/堆分配，全部定长缓冲 + snprintf。

#pragma once

#include <stddef.h>
#include <stdint.h>

// 单个连接的最大请求体
static constexpr size_t HTTP_MAX_BODY = 512;
// 请求行的最大长度
static constexpr size_t HTTP_MAX_LINE = 256;
// 单个响应的最大长度（/console 需要拼装多行 JSON，故大于请求体上限）
static constexpr size_t HTTP_MAX_RESPONSE = 6144;

struct HttpRequest {
	char method[8];                 // "GET" / "POST"
	char path[96];                  // 不含查询串
	char query[192];                // 查询串原始内容（不含 '?'）
	char contentType[64];           // Content-Type 头（可选）
	char body[HTTP_MAX_BODY];       // POST body（等价 WebServer::arg("plain")）
	size_t bodyLen;
	int fd;                         // 连接 fd，用于 send()
};

// 路由处理器：在任务中同步执行，可直接调用 httpSend* 写回响应
typedef void (*HttpHandler)(const HttpRequest& req);

struct HttpRoute {
	const char* path;
	const char* method;             // "GET" / "POST"
	HttpHandler handler;
};

// 发送响应（body 为 NUL 结尾字符串；为 nullptr 时发送 Content-Length: 0）
void httpSend(int fd, int status, const char* contentType, const char* body);

// 查询参数读取（等价 WebServer::arg("since")）
// 找到返回 true 并写入 *out，否则返回 false
bool httpArgInt(const HttpRequest& req, const char* name, int* out);

// 启动主 HTTP 服务器。会创建监听 socket 与处理任务。
// port=0 时使用 Kconfig 中的 CONFIG_CFDRONE_HTTP_PORT。
void httpServerStart(uint16_t port);

// 启动兼容旧 PCB 丝印的 301 跳转服务（:8080 -> 主端口）。port=0 表示禁用。
void httpRedirectStart(uint16_t port, uint16_t mainPort);
