@echo off
REM StampFly Ecosystem - Development Environment Setup (Windows)
REM Usage: Run this in CMD before development work.
REM        使い方: 開発作業の前にCMDでこのスクリプトを実行してください。
REM
REM   setup_env.bat           Set up ESP-IDF environment
REM   setup_env.bat --help    Show this help

if "%1"=="--help" goto :help
if "%1"=="-h" goto :help

setlocal enabledelayedexpansion

echo.
echo [INFO] Setting up StampFly development environment...
echo.

REM --- Discover Python in common install locations ---
REM --- 一般的なインストール場所からPythonを探す ---
for %%d in (
    "%USERPROFILE%\.pyenv\pyenv-win\shims"
    "%LOCALAPPDATA%\Programs\Python\Python313"
    "%LOCALAPPDATA%\Programs\Python\Python312"
    "%LOCALAPPDATA%\Programs\Python\Python311"
    "%LOCALAPPDATA%\Programs\Python\Python310"
    "C:\Python313"
    "C:\Python312"
    "C:\Python311"
    "C:\Python310"
    "%USERPROFILE%\scoop\apps\python\current"
    "%USERPROFILE%\anaconda3"
    "%USERPROFILE%\miniconda3"
) do (
    if exist "%%~d\python.exe" (
        set "PATH=%%~d;!PATH!"
    )
)

REM Verify Python is available
python.exe --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found. Install Python 3.10+ first.
    echo.
    echo   https://www.python.org/downloads/
    echo   winget install Python.Python.3.12
    exit /b 1
)

REM Store discovered PATH for endlocal
set "DISCOVERED_PATH=!PATH!"

REM End setlocal but preserve PATH (endlocal normally discards changes)
endlocal & set "PATH=%DISCOVERED_PATH%"

REM --- Source ESP-IDF environment ---
REM --- ESP-IDF環境を読み込む ---
set "IDF_PATH=%USERPROFILE%\esp\esp-idf"

if not exist "%IDF_PATH%\export.bat" (
    echo [ERROR] ESP-IDF not found at %IDF_PATH%
    echo.
    echo   Run install.bat first to install ESP-IDF.
    exit /b 1
)

echo [INFO] Loading ESP-IDF environment...
call "%IDF_PATH%\export.bat"
if errorlevel 1 (
    echo [ERROR] Failed to load ESP-IDF environment.
    exit /b 1
)

echo.
echo [OK] StampFly development environment ready.
echo.
echo   sf doctor          Check environment
echo   sf build vehicle   Build vehicle firmware
echo   sf --help          Show all commands
echo.
goto :eof

:help
echo.
echo StampFly Ecosystem - Development Environment Setup
echo.
echo Usage:
echo   setup_env.bat       Set up ESP-IDF + Python environment
echo.
echo This script:
echo   1. Discovers Python from common install locations
echo   2. Sources ESP-IDF export.bat
echo   3. Makes 'sf' CLI available
echo.
echo Run this once per CMD session before development work.
