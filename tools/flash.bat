@echo off
REM ============================================================================
REM RTKino Flash Tool — Windows
REM Flashes firmware to ESP32-S3 (LOLIN S3 Pro)
REM ============================================================================

echo.
echo  ============================================
echo    RTKino Flash Tool
echo  ============================================
echo.

REM --- Find esptool ----------------------------------------------------------
if exist "%~dp0esptool.exe" (
    echo [OK] Found bundled esptool.exe
) else (
    echo [ERROR] esptool.exe not found in this folder!
    echo         Re-download the release ZIP from GitHub.
    goto :end
)

REM --- Show available COM ports ----------------------------------------------
echo.
echo Available COM ports:
echo.
mode 2>nul | findstr "COM"
echo.
echo    Tip: if you see nothing, connect the ESP32-S3 via USB.
echo    Driver: https://www.wch-ic.com/downloads/CH343SER_EXE.html
echo.
set /p PORT=Enter COM port (e.g. COM3): 

if "%PORT%"=="" (
    echo [ERROR] No port entered.
    goto :end
)

echo.
echo [OK] Using port: %PORT%
echo.

REM --- Flash -----------------------------------------------------------------
if exist "%~dp0rtkino-merged.bin" (
    echo Flashing rtkino-merged.bin ... this takes about 60 seconds.
    echo.
    "%~dp0esptool.exe" --chip esp32s3 --port %PORT% --baud 921600 write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 "%~dp0rtkino-merged.bin"
) else (
    echo Flashing individual binaries...
    echo.
    "%~dp0esptool.exe" --chip esp32s3 --port %PORT% --baud 921600 write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 "%~dp0bootloader.bin" 0x8000 "%~dp0partitions.bin" 0xe000 "%~dp0boot_app0.bin" 0x10000 "%~dp0firmware.bin"
)

echo.
if %ERRORLEVEL% equ 0 (
    echo  ============================================
    echo    Flash completed!
    echo  ============================================
    echo.
    echo  Next steps:
    echo    1. Connect to WiFi: rtkino_AP
    echo       Password: rtkino_zedf9p
    echo    2. Open http://192.168.4.1
    echo    3. Configure WiFi and NTRIP
) else (
    echo  [ERROR] Flash failed.
    echo.
    echo    - Use a DATA cable, not charge-only
    echo    - Hold BOOT, press RESET, release BOOT, re-run
    echo    - Try a different COM port
)

:end
echo.
pause
