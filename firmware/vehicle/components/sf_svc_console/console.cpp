/**
 * @file console.cpp
 * @brief Unified Console Implementation using ESP-IDF Console
 *
 * ESP-IDF Console を使用した統合コマンドコンソール実装
 */

#include "console.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "Console";

namespace stampfly {

// =============================================================================
// Thread-Local Storage for Output Redirection
// =============================================================================
// Each thread (Serial REPL, WiFi client, etc.) can have its own output function
// 各スレッド（Serial REPL、WiFiクライアント等）が独自の出力関数を持てる

static __thread Console::OutputFunc t_output_func = nullptr;
static __thread void* t_output_ctx = nullptr;

// =============================================================================
// Console Implementation
// =============================================================================

Console& Console::getInstance()
{
    static Console instance;
    return instance;
}

esp_err_t Console::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Console already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing Console with esp_console");

    // Initialize esp_console
    // esp_console を初期化
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 16,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        .heap_alloc_caps = MALLOC_CAP_DEFAULT,
#endif
        .hint_color = 0,  // Not used in raw mode
        .hint_bold = 0,
    };

    esp_err_t ret = esp_console_init(&console_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize esp_console: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Console initialized");

    return ESP_OK;
}

void Console::registerAllCommands()
{
    if (!initialized_) {
        ESP_LOGW(TAG, "Console not initialized, cannot register commands");
        return;
    }

    ESP_LOGI(TAG, "Registering all commands");

    // Register built-in help command
    // 組み込みhelpコマンドを登録
    esp_console_register_help_command();

    // Register command groups
    // コマンドグループを登録
    register_system_commands();
    register_sensor_commands();
    register_motor_commands();
    register_control_commands();
    register_comm_commands();
    register_calib_commands();
    register_misc_commands();

    ESP_LOGI(TAG, "All commands registered");
}

int Console::run(const char* cmdline)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "Console not initialized");
        return -1;
    }

    if (cmdline == nullptr || strlen(cmdline) == 0) {
        return 0;  // Empty command is OK
    }

    int ret;
    esp_err_t err = esp_console_run(cmdline, &ret);

    if (err == ESP_ERR_NOT_FOUND) {
        print("Unknown command: %s\r\n", cmdline);
        print("Type 'help' for available commands.\r\n");
        return -1;
    } else if (err == ESP_ERR_INVALID_ARG) {
        // Command was empty or whitespace only
        return 0;
    } else if (err != ESP_OK) {
        print("Error executing command: %s\r\n", esp_err_to_name(err));
        return -1;
    }

    return ret;
}

void Console::setOutput(OutputFunc func, void* ctx)
{
    t_output_func = func;
    t_output_ctx = ctx;
}

void Console::clearOutput()
{
    t_output_func = nullptr;
    t_output_ctx = nullptr;
}

void Console::print(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprint(fmt, args);
    va_end(args);
}

void Console::vprint(const char* fmt, va_list args)
{
    char buf[PRINT_BUF_SIZE];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);

    if (len < 0) {
        return;  // Format error
    }

    // Truncate if too long
    // 長すぎる場合は切り詰める
    if (len >= static_cast<int>(sizeof(buf))) {
        len = sizeof(buf) - 1;
        buf[len] = '\0';
    }

    // Output to redirected function or default stdout
    // リダイレクト先関数またはデフォルトのstdoutに出力
    if (t_output_func != nullptr) {
        t_output_func(buf, t_output_ctx);
    } else {
        printf("%s", buf);
        fflush(stdout);
    }
}

}  // namespace stampfly
