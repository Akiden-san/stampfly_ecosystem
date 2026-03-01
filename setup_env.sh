#!/bin/bash
# StampFly Ecosystem - Development Environment Setup (Linux / macOS / WSL2)
# Usage: source setup_env.sh
#
# This script must be sourced, not executed:
#   source setup_env.sh
#
# StampFly開発環境セットアップスクリプト（source で読み込むこと）

# Guard: detect if executed instead of sourced
# 実行ではなく source されたか確認
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[ERROR] This script must be sourced, not executed."
    echo "  Usage: source setup_env.sh"
    exit 1
fi

# Colors
_sf_green='\033[0;32m'
_sf_blue='\033[0;34m'
_sf_red='\033[0;31m'
_sf_nc='\033[0m'

echo
echo -e "${_sf_blue}[INFO]${_sf_nc} Setting up StampFly development environment..."
echo

# Determine project root (directory containing this script)
# プロジェクトルートを特定（このスクリプトのあるディレクトリ）
_sf_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Determine IDF_PATH: .sf/config.toml > IDF_PATH env > default
# IDF_PATHの決定: 設定ファイル > 環境変数 > デフォルトパス
_sf_idf_path=""

# Try .sf/config.toml first
# まず設定ファイルを確認
_sf_config="${_sf_script_dir}/.sf/config.toml"
if [ -f "$_sf_config" ]; then
    _sf_idf_path="$(grep '^path = ' "$_sf_config" 2>/dev/null | head -1 | sed 's/^path = "//;s/"$//')"
fi

# Fallback to IDF_PATH env or default
# 環境変数またはデフォルトにフォールバック
if [ -z "$_sf_idf_path" ] || [ ! -d "$_sf_idf_path" ]; then
    if [ -n "$IDF_PATH" ] && [ -d "$IDF_PATH" ]; then
        _sf_idf_path="$IDF_PATH"
    else
        _sf_idf_path="$HOME/esp/esp-idf"
    fi
fi

# Verify ESP-IDF exists
# ESP-IDFの存在確認
if [ ! -f "$_sf_idf_path/export.sh" ]; then
    echo -e "${_sf_red}[ERROR]${_sf_nc} ESP-IDF not found at $_sf_idf_path"
    echo "  Run ./install.sh first."
    echo
    # Clean up temporary variables
    unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path
    return 1
fi

# WSL2: strip /mnt/ paths to avoid Windows executables with CRLF
# WSL2: CRLFのWindows実行ファイルを回避するため /mnt/ パスを除外
if [ -f /proc/version ] && grep -qi microsoft /proc/version 2>/dev/null; then
    echo -e "${_sf_blue}[INFO]${_sf_nc} WSL2 detected, filtering Windows paths..."
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | tr '\n' ':' | sed 's/:$//')
fi

# Source ESP-IDF environment
# ESP-IDF環境を読み込み
echo -e "${_sf_blue}[INFO]${_sf_nc} Loading ESP-IDF environment..."
source "$_sf_idf_path/export.sh"

echo
echo -e "${_sf_green}[OK]${_sf_nc} StampFly development environment ready."
echo
echo "  sf doctor          Check environment"
echo "  sf build vehicle   Build vehicle firmware"
echo "  sf --help          Show all commands"
echo

# Clean up temporary variables
# 一時変数をクリーンアップ
unset _sf_green _sf_blue _sf_red _sf_nc _sf_script_dir _sf_config _sf_idf_path
