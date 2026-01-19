/**
 * @file cli_task.cpp
 * @brief CLI Task - Serial CLI using ESP-IDF standard console
 *
 * CLIタスク - ESP-IDF 標準コンソールを使用した Serial CLI
 *
 * Features:
 * - Command history (up/down arrows) via linenoise
 * - Tab completion for registered commands
 * - Line editing (cursor movement, backspace, delete)
 *
 * Note: Uses esp_console_new_repl_usb_cdc() for reliable USB CDC operation
 */

#include "tasks_common.hpp"
#include "console.hpp"

#include "esp_console.h"
#include "esp_vfs_dev.h"

static const char* TAG = "CLITask";

using namespace config;
using namespace globals;

void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    // Configure USB CDC console
    // USB CDC コンソールを設定
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "stampfly> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_usb_cdc_config_t cdc_config =
        ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();

    // Create the console (this initializes esp_console and registers default help)
    // コンソールを作成（esp_console を初期化し、デフォルト help を登録）
    esp_console_repl_t* repl = nullptr;
    esp_err_t ret = esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create USB CDC console: %s", esp_err_to_name(ret));
        vTaskDelete(nullptr);
        return;
    }

    // Register all commands via Console AFTER REPL creation
    // This overwrites the default help command with our custom one
    // REPL 作成後にコマンドを登録（デフォルト help をカスタム版で上書き）
    auto& console = stampfly::Console::getInstance();
    console.registerAllCommands();

    // Start the console
    // コンソールを開始
    ret = esp_console_start_repl(repl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start console: %s", esp_err_to_name(ret));
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "Serial CLI started (ESP-IDF console)");

    // Console runs in its own task, so this task can be deleted
    // コンソールは独自タスクで動作するため、このタスクは削除可能
    vTaskDelete(nullptr);
}
