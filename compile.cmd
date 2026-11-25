@echo off
setlocal EnableExtensions

cd /d "%~dp0"

set "PIO_EXE="
where pio >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    for /f "delims=" %%I in ('where pio') do (
        set "PIO_EXE=%%I"
        goto :found_pio
    )
)

if exist "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" (
    set "PIO_EXE=%USERPROFILE%\.platformio\penv\Scripts\pio.exe"
)

:found_pio
if "%PIO_EXE%"=="" (
    echo [ERROR] PlatformIO CLI ^(pio.exe^) not found in PATH or "%USERPROFILE%\.platformio\penv\Scripts\pio.exe".
    exit /b 1
)

echo [BUILD] Building firmware with "%PIO_EXE%"...
"%PIO_EXE%" run
if errorlevel 1 (
    echo [ERROR] Build failed!
    exit /b 1
)

set "SRC_BIN=.pio\build\lolin_c3_mini\firmware.bin"
set "OUT_BIN=firmware.bin"

if not exist "%SRC_BIN%" (
    echo [ERROR] Expected binary not found: "%SRC_BIN%"
    exit /b 1
)

copy /Y "%SRC_BIN%" "%OUT_BIN%" >nul
if errorlevel 1 (
    echo [ERROR] Failed to copy "%SRC_BIN%" to "%OUT_BIN%"
    exit /b 1
)

for %%F in ("%OUT_BIN%") do set "BIN_SIZE=%%~zF"
set "BIN_MD5="
for /f "delims=" %%H in ('certutil -hashfile "%OUT_BIN%" MD5 ^| findstr /V ":"') do set "BIN_MD5=%%H"
echo.
echo [OK] OTA-ready binary: "%~dp0%OUT_BIN%"
echo [OK] Size: %BIN_SIZE% bytes
echo [OK] MD5 : %BIN_MD5%
echo.
echo Upload "%OUT_BIN%" at http://192.168.4.1/update ^(or http://vw-bt.local/update^)
exit /b 0
