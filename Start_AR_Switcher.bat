@echo off
setlocal DisableDelayedExpansion
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"
set "AR_APP_DIR="
if exist "%~dp0main_udp_yaw_desktop_switcher.py" set "AR_APP_DIR=%~dp0"
if defined AR_APP_DIR goto app_found
if exist "D:\tools\inair-xreal-ARscreen\main_udp_yaw_desktop_switcher.py" set "AR_APP_DIR=D:\tools\inair-xreal-ARscreen"
if defined AR_APP_DIR goto app_found
:choose_folder
echo.
echo 未找到主程序。请将完整程序包解压后，输入或拖入程序所在文件夹。
echo 文件夹内应包含 main_udp_yaw_desktop_switcher.py。输入 q 退出。
set "AR_APP_DIR="
set /p "AR_APP_DIR=程序文件夹: "
if not defined AR_APP_DIR goto cancelled
set "AR_APP_DIR=%AR_APP_DIR:"=%"
if /i "%AR_APP_DIR%"=="q" goto cancelled
if exist "%AR_APP_DIR%\main_udp_yaw_desktop_switcher.py" goto app_found
echo 此文件夹没有主程序，请重新选择。
goto choose_folder
:app_found
pushd "%AR_APP_DIR%"
if errorlevel 1 goto folder_failed
set "AR_APP_DIR=%CD%"
for %%F in (head_flick.py imu_stream.py hardware_options.py inair_stream.py inair_pose.py VirtualDesktopAccessor.dll) do if not exist "%%F" goto incomplete
echo 程序目录: "%AR_APP_DIR%"
set "AR_RUNTIME="
if defined AR_PYTHON if exist "%AR_PYTHON%" set "AR_RUNTIME=%AR_PYTHON%"
if not defined AR_RUNTIME if exist "%AR_APP_DIR%\inairenv\Scripts\python.exe" set "AR_RUNTIME=%AR_APP_DIR%\inairenv\Scripts\python.exe"
if not defined AR_RUNTIME if exist "%USERPROFILE%\inairenv\Scripts\python.exe" set "AR_RUNTIME=%USERPROFILE%\inairenv\Scripts\python.exe"
if not defined AR_RUNTIME goto missing
"%AR_RUNTIME%" -c "import sys; sys.exit(0 if sys.maxsize > 2**32 else 1)"
if errorlevel 1 goto invalid
"%AR_RUNTIME%" -B -u "%AR_APP_DIR%\main_udp_yaw_desktop_switcher.py"
set "AR_EXIT=%ERRORLEVEL%"
if not "%AR_EXIT%"=="0" echo 程序未正常启动或运行失败，请查看上方错误。
popd
pause
exit /b %AR_EXIT%
:missing
echo 未找到 Python。请保留用户目录下的 inairenv 环境。
echo 也可通过 AR_PYTHON 环境变量指定 Python 解释器完整路径。
goto failed
:invalid
echo Python 无法运行或不是64位版本，请检查解释器路径。
goto failed
:incomplete
echo 程序文件不完整。请完整解压最终版压缩包，不要只复制批处理或主程序。
:failed
popd
pause
exit /b 1
:folder_failed
echo 无法进入程序文件夹，请检查路径及访问权限。
pause
exit /b 1
:cancelled
exit /b 0
