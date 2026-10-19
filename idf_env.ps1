# ============================================================
#  CF-Drone 飞控 —— ESP-IDF 环境激活包装脚本 (PowerShell)
#
#  本机 ESP-IDF v5.4.4 的环境布局比较特殊：
#    * IDF 源码位于 D:\esp\v5.4.4\esp-idf
#    * 工具链位于 D:\Espressif\tools
#    * Python venv 位于 D:\Espressif\tools\python\v5.4.4\venv
#      （官方 export.ps1 默认去找 D:\Espressif\python_env\idf5.4_py3.9_env，
#        该路径不存在，因此必须显式设置 IDF_PYTHON_ENV_PATH）
#
#  用法：
#    . .\idf_env.ps1              # 激活后可直接用 idf.py
#    .\idf_env.ps1 idf.py build   # 单次执行
# ============================================================

param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$IdfArgs
)

$env:IDF_PATH            = 'D:\esp\v5.4.4\esp-idf'
$env:IDF_TOOLS_PATH      = 'D:\Espressif'
$env:IDF_PYTHON_ENV_PATH = 'D:\Espressif\tools\python\v5.4.4\venv'

$python  = Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts\python.exe'
$idfPy   = Join-Path $env:IDF_PATH 'tools\idf.py'
$toolsPy = Join-Path $env:IDF_PATH 'tools\idf_tools.py'

if (-not (Test-Path $python)) {
    Write-Error "找不到 Python venv: $python"
    return
}

# 导出工具链 PATH / IDF_PYTHON 等环境变量
$exported = & $python $toolsPy export --format key-value --idf-path $env:IDF_PATH
foreach ($line in $exported) {
    if ($line -match '^([^=]+)=(.*)$') {
        Set-Item -Path ("Env:" + $matches[1]) -Value $matches[2]
    }
}

# 让 idf.py 可作为命令调用
function global:idf.py { & $python $idfPy @args }

if ($IdfArgs.Count -gt 0) {
    & $python $idfPy @IdfArgs
}
