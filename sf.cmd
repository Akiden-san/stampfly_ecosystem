@echo off
REM StampFly CLI wrapper for Windows Device Guard environments
REM Device Guard環境でブロックされるsf.exeを回避して sfcli を起動する

setlocal

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"

REM Ensure local package import works even without editable install
REM editable install が無くてもローカルパッケージを優先して読み込む
set "PYTHONPATH=%ROOT%\lib;%PYTHONPATH%"

set "SF_PYTHON_EXE="
set "SF_PYTHON_ARGS="
set "IDF_ENV_ROOT=%USERPROFILE%\.espressif\python_env"

if exist "%IDF_ENV_ROOT%" (
    for /d %%D in ("%IDF_ENV_ROOT%\idf*_env") do (
        if not defined SF_PYTHON_EXE if exist "%%~fD\Scripts\python.exe" (
            set "SF_PYTHON_EXE=%%~fD\Scripts\python.exe"
        )
    )
)

if not defined SF_PYTHON_EXE (
    where python >nul 2>&1
    if not errorlevel 1 set "SF_PYTHON_EXE=python"
)

if not defined SF_PYTHON_EXE (
    where py >nul 2>&1
    if not errorlevel 1 (
        set "SF_PYTHON_EXE=py"
        set "SF_PYTHON_ARGS=-3"
    )
)

if not defined SF_PYTHON_EXE (
    echo [ERROR] Python not found. Run install.bat first.
    exit /b 1
)

pushd "%ROOT%" >nul
"%SF_PYTHON_EXE%" %SF_PYTHON_ARGS% -m sfcli %*
set "EXIT_CODE=%ERRORLEVEL%"
popd >nul

exit /b %EXIT_CODE%
