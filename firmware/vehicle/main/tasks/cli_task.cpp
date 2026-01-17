/**
 * @file cli_task.cpp
 * @brief CLI Task - Serial REPL with linenoise
 *
 * CLIタスク - linenoiseを使用したSerial REPL
 *
 * Features:
 * - Command history (up/down arrows)
 * - Tab completion for registered commands
 * - Line editing (cursor movement, backspace, delete)
 *
 * Note: Binary logging moved to stampfly_logger component (400Hz via ESP Timer)
 */

#include "tasks_common.hpp"
#include "serial_repl.hpp"

static const char* TAG = "CLITask";

using namespace config;
using namespace globals;

void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    // Get SerialREPL instance
    // SerialREPLインスタンスを取得
    auto& repl = stampfly::SerialREPL::getInstance();

    // Initialize SerialREPL (USB Serial JTAG + linenoise)
    // SerialREPLを初期化（USB Serial JTAG + linenoise）
    esp_err_t ret = repl.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SerialREPL: %s", esp_err_to_name(ret));
        // Fall back to simple loop if initialization fails
        // 初期化に失敗した場合はシンプルなループにフォールバック
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Run REPL loop (blocking)
    // REPLループを実行（ブロッキング）
    // This function never returns under normal operation
    // この関数は通常動作では戻らない
    repl.run();

    // Should not reach here
    // ここには到達しないはず
    ESP_LOGE(TAG, "SerialREPL::run() returned unexpectedly");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
