/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

// 移植说明：原文件依赖 Arduino 的 Print 类与 log_printf()。
// ESP-IDF 版改为直接写入控制台串口（hal_console），
// setLogOutput() 保留为兼容接口（本工程不使用）。

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hal/hal_console.h"

// 前向声明：保留 setLogOutput 签名以便驱动无需改动（本工程不启用）
class Print;

class Logger {
public:
	void log(const char *format, ...) {
		if (!verbose) return;

		char msg[160];
		va_list args;
		va_start(args, format);
		vsnprintf(msg, sizeof(msg), format, args);
		va_end(args);

		if (logName != nullptr) {
			hal_console_write_str(logName);
			hal_console_write_str(": ");
		}
		hal_console_write_str(msg);
		hal_console_write_str("\n");
	}

	inline void setLogOutput(Print& output) { logOutput = &output; }
	inline void setVerbosity(const bool _verbose) { verbose = _verbose; }

protected:
	inline void setLogName(const char* name) { logName = name; }

private:
	const char* logName = nullptr;
	Print *logOutput = nullptr;
	bool verbose = true;
};
