@echo off
REM BCU Firmware - SIL Test Build and Run Script
REM Compiles and runs Software-in-the-Loop tests on Windows

echo =============================================================
echo BCU Firmware - SIL Test Builder
echo =============================================================

REM Check for GCC
where gcc >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: GCC not found in PATH
    echo Please install MinGW-w64 or add GCC to PATH
    exit /b 1
)

REM Compile
echo Compiling sil_test_main.c...
gcc -Wall -Wextra -O2 -o sil_tests.exe sil_test_main.c

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Compilation failed!
    exit /b 1
)

echo Compilation successful!
echo.

REM Run tests
echo Running SIL tests...
echo.
sil_tests.exe

echo.
echo =============================================================
echo Done!
echo =============================================================
