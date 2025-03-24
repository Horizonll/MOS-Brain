# 获取当前脚本所在的 scripts 目录
$scriptsDir = Split-Path -Parent $PSCommandPath

# 构建 test 目录路径
$testDir = Join-Path -Path (Split-Path -Parent $scriptsDir) -ChildPath "test"

# Python 文件路径
$pythonFile1 = Join-Path -Path $testDir -ChildPath "tcp_host_test_reciever.py"
$pythonFile2 = Join-Path -Path $testDir -ChildPath "decider_tester.py"

# 检查文件是否存在
if (-not (Test-Path $pythonFile1)) { Write-Error "Error: $pythonFile1 不存在"; return }
if (-not (Test-Path $pythonFile2)) { Write-Error "Error: $pythonFile2 不存在"; return }

# 加载窗口聚焦API
if (-not ("Win32" -as [Type])) {
    Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Win32 {
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern IntPtr FindWindow(string lpClassName, string lpWindowName);
}
"@
}


# 启动第一个Python脚本（处理空格和命令分隔符）
$args1 = @(
    '/k',
    "title tcp_host_test_reciever && python `"$pythonFile1`""
)
Start-Process cmd.exe -ArgumentList $args1

# 启动第二个Python脚本
$args2 = @(
    '/k',
    "title decider_tester && python `"$pythonFile2`""
)
$secondProcess = Start-Process cmd.exe -ArgumentList $args2 -PassThru

# 等待窗口并聚焦
Start-Sleep -Seconds 2
$secondWindowHandle = [Win32]::FindWindow($null, "decider_tester")
if ($secondWindowHandle -ne [IntPtr]::Zero) {
    [Win32]::SetForegroundWindow($secondWindowHandle)
}