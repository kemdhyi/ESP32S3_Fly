@echo off
REM ============================================================
REM  CF-Drone 飞控 —— ESP-IDF 环境激活包装脚本 (cmd.exe)
REM
REM  用法：
REM    idf_env.bat idf.py build
REM    idf_env.bat idf.py -p COM5 flash monitor
REM ============================================================

set "IDF_PATH=D:\esp\v5.4.4\esp-idf"
set "IDF_TOOLS_PATH=D:\Espressif"
set "IDF_PYTHON_ENV_PATH=D:\Espressif\tools\python\v5.4.4\venv"
set "IDF_PYTHON=%IDF_PYTHON_ENV_PATH%\Scripts\python.exe"

for /f "usebackq delims=" %%i in (`"%IDF_PYTHON%" "%IDF_PATH%\tools\idf_tools.py" export --format key-value`) do set "%%i"

"%IDF_PYTHON%" "%IDF_PATH%\tools\idf.py" %*

exit /b %ERRORLEVEL%
