/**
 * @file serial_repl.hpp
 * @brief Serial REPL with linenoise integration
 *
 * Provides command-line interface over USB Serial with:
 * - Command history (up/down arrows)
 * - Tab completion for registered commands
 * - Line editing (cursor movement, backspace, delete)
 *
 * シリアルREPL（linenoise統合）
 * - コマンド履歴（上下矢印）
 * - Tab補完
 * - 行編集（カーソル移動、バックスペース、削除）
 */

#pragma once

#include "esp_err.h"

namespace stampfly {

/**
 * @brief Serial REPL class with linenoise integration
 *
 * Singleton class that manages USB Serial REPL with linenoise
 * for advanced line editing capabilities.
 */
class SerialREPL {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to SerialREPL instance
     */
    static SerialREPL& getInstance();

    /**
     * @brief Initialize Serial REPL
     *
     * Sets up USB Serial JTAG driver and VFS integration.
     * Configures linenoise with history and completion callbacks.
     *
     * USB Serial JTAGドライバとVFS統合を設定。
     * linenoiseの履歴と補完コールバックを設定。
     *
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Run the REPL loop (blocking)
     *
     * Main REPL loop using linenoise for input.
     * This function blocks indefinitely, processing commands.
     * Should be called from a dedicated FreeRTOS task.
     *
     * linenoiseを使用したメインREPLループ。
     * この関数は無限にブロックし、コマンドを処理する。
     * 専用のFreeRTOSタスクから呼び出すこと。
     */
    void run();

    /**
     * @brief Check if REPL is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Set history maximum length
     * @param len Maximum number of history entries (default: 20)
     */
    void setHistoryMaxLen(int len);

private:
    SerialREPL() = default;
    ~SerialREPL() = default;

    // Non-copyable
    // コピー禁止
    SerialREPL(const SerialREPL&) = delete;
    SerialREPL& operator=(const SerialREPL&) = delete;

    bool initialized_ = false;
    int history_max_len_ = 20;

    /**
     * @brief Tab completion callback for linenoise
     * @param buf Current input buffer
     * @param lc Completions structure
     */
    static void completionCallback(const char* buf, void* lc);

    /**
     * @brief Hints callback for linenoise
     * @param buf Current input buffer
     * @param color Output: hint color
     * @param bold Output: whether hint is bold
     * @return Hint string or nullptr
     */
    static char* hintsCallback(const char* buf, int* color, int* bold);
};

}  // namespace stampfly
