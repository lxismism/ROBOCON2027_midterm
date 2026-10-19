@echo off
title STM32H723 DAPLink 调试服务器启动器

REM ===================== 【配置区】 =====================
REM 1. OpenOCD 根目录（绝对路径，到文件夹为止，不要到 bin）
REM    下面两个是本机已知的安装位置，脚本会依次尝试；换电脑时把你自己的路径加进来即可
set "OPENOCD_ROOT=D:\dowm load\ozone_something\OpenOCD&ArmGCC\DevEnv\openocd-v0.12.0-i686-w64-mingw32"
if not exist "%OPENOCD_ROOT%\bin\openocd.exe" set "OPENOCD_ROOT=D:\MyTools\OpenOCD_ArmGCC\OpenOCD&ArmGCC\DevEnv\openocd-v0.12.0-i686-w64-mingw32"

REM 2. DAPLink 配置文件：默认用仓库根目录下的 ozone_daplink.cfg（随工程一起走）
set "DAPLINK_CFG=%~dp0ozone_daplink.cfg"

REM 3. 等待 OpenOCD 启动的时间（ping 次数，4 次约 3 秒，一般不用改）
set "WAIT_COUNT=4"
REM ======================================================

REM 自动拼接 OpenOCD 相关路径（无需修改）
set "OPENOCD_EXE=%OPENOCD_ROOT%\bin\openocd.exe"
set "TARGET_CFG=%OPENOCD_ROOT%\share\openocd\scripts\target\stm32h7x.cfg"

REM 检查文件是否存在，避免路径错误
if not exist "%OPENOCD_EXE%" (
    echo [错误] 未找到 OpenOCD：%OPENOCD_EXE%
    echo 请把配置区的 OPENOCD_ROOT 改成你电脑上 openocd.exe 所在文件夹的上级目录！
    pause
    exit /b 1
)
if not exist "%DAPLINK_CFG%" (
    echo [错误] 未找到 DAPLink 配置文件：%DAPLINK_CFG%
    echo 请确认 ozone_daplink.cfg 和这个 bat 在同一个目录下！
    pause
    exit /b 1
)
if not exist "%TARGET_CFG%" (
    echo [错误] 未找到目标芯片配置：%TARGET_CFG%
    echo 工程是 STM32H723，需要的文件是 scripts\target\stm32h7x.cfg，请确认 OpenOCD 路径正确！
    pause
    exit /b 1
)

REM 1. 启动 OpenOCD 调试服务器
echo 正在启动 OpenOCD 调试服务器...
echo.
echo 连接信息：
echo - GDB 连接端口：3333
echo - Telnet 连接端口：4444
echo - TCL 连接端口：6666
echo.
echo 提示：不要关闭此窗口，否则调试会断开！
echo.

REM 注意：OPENOCD_ROOT 路径里带 &（OpenOCD&ArmGCC），
REM 所以这里必须写成 cmd /k call "<exe>" ...，不能用 cmd /k ""<exe>" ..."
REM 后者会让 cmd 把路径里的 & 当成命令分隔符，子窗口起不来
start "OpenOCD - STM32 调试服务器" cmd /k call "%OPENOCD_EXE%" -f "%DAPLINK_CFG%" -f "%TARGET_CFG%"

REM 2. 等待服务器启动
"%SystemRoot%\System32\PING.EXE" -n %WAIT_COUNT% 127.0.0.1 > nul

REM 3. 检查 3333 端口是否真的在监听
"%SystemRoot%\System32\netstat.exe" -ano | "%SystemRoot%\System32\findstr.exe" /C:":3333" | "%SystemRoot%\System32\findstr.exe" /C:"LISTENING" > nul
if errorlevel 1 (
    echo ==================================================
    echo [警告] 3333 端口没有在监听，OpenOCD 可能没起来！
    echo 请看弹出的那个 OpenOCD 窗口里的错误信息。
    echo 常见原因：DAPLink 没插好、被别的 OpenOCD 占用、驱动没装。
    echo ==================================================
) else (
    echo ==================================================
    echo OpenOCD 调试服务器已启动成功！
    echo 现在可以用 Ozone / VSCode 连接 localhost:3333 进行调试。
    echo ==================================================
)
echo.
pause
exit /b 0
