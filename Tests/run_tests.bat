@echo off
REM ============================================================================
REM BCU Firmware Unit Tests - Build and Run Script
REM ============================================================================
REM Requirements: GCC compiler (MinGW-w64 recommended)
REM Usage: run_tests.bat [test_name]
REM   test_name: crc, ringbuffer, filter, all (default)
REM ============================================================================

setlocal enabledelayedexpansion

echo.
echo =============================================================
echo BCU Firmware Unit Tests
echo =============================================================
echo.

REM Check for GCC
where gcc >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: GCC not found in PATH
    echo Please install MinGW-w64 or add GCC to PATH
    exit /b 1
)

REM Parse argument
set TEST=%1
if "%TEST%"=="" set TEST=all

REM Compiler flags
set CFLAGS=-Wall -Wextra -O2 -std=c99

REM Create output directory
if not exist "bin" mkdir bin

REM Track results
set TOTAL_PASS=0
set TOTAL_FAIL=0

REM ============================================================================
REM Build and run CRC tests
REM ============================================================================
if "%TEST%"=="crc" goto :test_crc
if "%TEST%"=="all" goto :test_crc
goto :skip_crc

:test_crc
echo [BUILD] test_crc.c
gcc %CFLAGS% -o bin\test_crc.exe test_crc.c Unity\unity.c 2>&1
if %errorlevel% neq 0 (
    echo [FAIL] Build failed for test_crc
    set /a TOTAL_FAIL+=1
    goto :skip_crc
)

echo [RUN] test_crc.exe
bin\test_crc.exe
if %errorlevel% neq 0 (
    set /a TOTAL_FAIL+=1
) else (
    set /a TOTAL_PASS+=1
)
echo.

:skip_crc

REM ============================================================================
REM Build and run RingBuffer tests
REM ============================================================================
if "%TEST%"=="ringbuffer" goto :test_rb
if "%TEST%"=="all" goto :test_rb
goto :skip_rb

:test_rb
echo [BUILD] test_ringbuffer.c
gcc %CFLAGS% -o bin\test_ringbuffer.exe test_ringbuffer.c Unity\unity.c 2>&1
if %errorlevel% neq 0 (
    echo [FAIL] Build failed for test_ringbuffer
    set /a TOTAL_FAIL+=1
    goto :skip_rb
)

echo [RUN] test_ringbuffer.exe
bin\test_ringbuffer.exe
if %errorlevel% neq 0 (
    set /a TOTAL_FAIL+=1
) else (
    set /a TOTAL_PASS+=1
)
echo.

:skip_rb

REM ============================================================================
REM Build and run Filter tests
REM ============================================================================
if "%TEST%"=="filter" goto :test_filter
if "%TEST%"=="all" goto :test_filter
goto :skip_filter

:test_filter
echo [BUILD] test_filter.c
gcc %CFLAGS% -o bin\test_filter.exe test_filter.c Unity\unity.c 2>&1
if %errorlevel% neq 0 (
    echo [FAIL] Build failed for test_filter
    set /a TOTAL_FAIL+=1
    goto :skip_filter
)

echo [RUN] test_filter.exe
bin\test_filter.exe
if %errorlevel% neq 0 (
    set /a TOTAL_FAIL+=1
) else (
    set /a TOTAL_PASS+=1
)
echo.

:skip_filter

REM ============================================================================
REM Summary
REM ============================================================================
echo =============================================================
echo OVERALL SUMMARY
echo =============================================================
echo Test suites passed: %TOTAL_PASS%
echo Test suites failed: %TOTAL_FAIL%
echo =============================================================

if %TOTAL_FAIL% gtr 0 (
    echo SOME TEST SUITES FAILED!
    exit /b 1
) else (
    echo ALL TEST SUITES PASSED!
    exit /b 0
)
