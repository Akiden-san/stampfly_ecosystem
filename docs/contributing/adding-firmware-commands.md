# CLI コマンド追加ガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFly の CLI システムは ESP-IDF Console を基盤としており、Serial CLI と WiFi CLI の両方で同じコマンドが使用できます。

### アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    sf_svc_console                           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         コマンド実装（この層に追加）                    │   │
│  │  commands/cmd_system.cpp   - システム系              │   │
│  │  commands/cmd_sensor.cpp   - センサー系              │   │
│  │  commands/cmd_motor.cpp    - モーター系              │   │
│  │  commands/cmd_comm.cpp     - 通信系                  │   │
│  │  commands/cmd_calib.cpp    - キャリブレーション系     │   │
│  │  commands/cmd_control.cpp  - 制御系                  │   │
│  │  commands/cmd_misc.cpp     - その他                  │   │
│  └─────────────────────────────────────────────────────┘   │
│                          ↓                                  │
│              esp_console_cmd_register()                     │
└─────────────────────────────────────────────────────────────┘
                    ↑                    ↑
        ┌───────────┴───────┐    ┌───────┴───────┐
        │    Serial CLI     │    │   WiFi CLI    │
        └───────────────────┘    └───────────────┘
```

## 2. コマンド追加手順

### ステップ 1: 適切なファイルを選択

| カテゴリ | ファイル |
|----------|----------|
| システム（status, reboot等）| `cmd_system.cpp` |
| センサー（sensor, loglevel等）| `cmd_sensor.cpp` |
| モーター | `cmd_motor.cpp` |
| 通信（comm, pair等）| `cmd_comm.cpp` |
| キャリブレーション | `cmd_calib.cpp` |
| 制御（trim, gain等）| `cmd_control.cpp` |
| その他 | `cmd_misc.cpp` |
| **新カテゴリ** | 新規ファイル `cmd_xxx.cpp` を作成 |

ファイルパス: `firmware/vehicle/components/sf_svc_console/commands/`

### ステップ 2: コマンドハンドラを実装

```cpp
// cmd_misc.cpp に追加する例

/**
 * @brief mycommand コマンドハンドラ
 * @param argc 引数の数
 * @param argv 引数配列
 * @return 0: 成功, 非0: エラー
 */
static int cmd_mycommand(int argc, char** argv)
{
    auto& console = Console::getInstance();

    // 引数なしの場合
    if (argc == 1) {
        console.print("Usage: mycommand <subcommand>\r\n");
        console.print("  mycommand status  - Show status\r\n");
        console.print("  mycommand set <value>  - Set value\r\n");
        return 0;
    }

    // サブコマンド処理
    const char* subcmd = argv[1];

    if (strcmp(subcmd, "status") == 0) {
        console.print("Current status: OK\r\n");
        return 0;
    }

    if (strcmp(subcmd, "set") == 0) {
        if (argc < 3) {
            console.print("Error: value required\r\n");
            return 1;
        }
        int value = atoi(argv[2]);
        console.printf("Value set to: %d\r\n", value);
        return 0;
    }

    console.printf("Unknown subcommand: %s\r\n", subcmd);
    return 1;
}
```

### ステップ 3: コマンドを登録

同じファイル内の `register_xxx_commands()` 関数に追加:

```cpp
void register_misc_commands()
{
    // 既存のコマンド登録...

    // 新しいコマンドを追加
    const esp_console_cmd_t mycommand_cmd = {
        .command = "mycommand",
        .help = "My custom command [status|set <value>]",
        .hint = NULL,
        .func = &cmd_mycommand,
        .argtable = NULL,
    };
    esp_console_cmd_register(&mycommand_cmd);
}
```

### ステップ 4: help コマンドを更新

`cmd_system.cpp` の `cmd_help()` 関数にエントリを追加:

```cpp
static int cmd_help(int argc, char** argv)
{
    auto& console = Console::getInstance();

    console.print("\r\nAvailable commands:\r\n");
    // ... 既存のコマンド ...
    console.print("  mycommand - My custom command [status|set <value>]\r\n");  // 追加
    console.print("\r\n");

    return 0;
}
```

### ステップ 5: Tab 補完を更新（任意）

WiFi CLI の補完リスト (`wifi_cli.cpp`) に追加:

```cpp
static void wifiCompletionCallback(const char* buf, LineCompletions* lc)
{
    static const char* commands[] = {
        // ... 既存のコマンド ...
        "mycommand",  // 追加
        nullptr
    };
    // ...
}
```

## 3. コード例

### シンプルなコマンド（引数なし）

```cpp
static int cmd_ping(int argc, char** argv)
{
    auto& console = Console::getInstance();
    console.print("pong\r\n");
    return 0;
}

// 登録
const esp_console_cmd_t ping_cmd = {
    .command = "ping",
    .help = "Respond with pong",
    .hint = NULL,
    .func = &cmd_ping,
    .argtable = NULL,
};
esp_console_cmd_register(&ping_cmd);
```

### 数値引数を取るコマンド

```cpp
static int cmd_delay(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (argc < 2) {
        console.print("Usage: delay <ms>\r\n");
        return 1;
    }

    int ms = atoi(argv[1]);
    if (ms <= 0 || ms > 10000) {
        console.print("Error: ms must be 1-10000\r\n");
        return 1;
    }

    console.printf("Waiting %d ms...\r\n", ms);
    vTaskDelay(pdMS_TO_TICKS(ms));
    console.print("Done\r\n");
    return 0;
}
```

### グローバル状態にアクセスするコマンド

```cpp
#include "globals.hpp"

static int cmd_battery(int argc, char** argv)
{
    auto& console = Console::getInstance();

    // グローバル変数から値を取得
    float voltage = globals::g_power_data.voltage;
    float current = globals::g_power_data.current;

    console.printf("Battery: %.2fV, %.0fmA\r\n", voltage, current);
    return 0;
}
```

## 4. ベストプラクティス

### 出力フォーマット

```cpp
// 改行は \r\n を使用（Telnet互換）
console.print("Hello\r\n");

// 数値出力
console.printf("Value: %d\r\n", value);
console.printf("Float: %.2f\r\n", fvalue);

// テーブル形式
console.print("=== Status ===\r\n");
console.printf("  Item1: %d\r\n", val1);
console.printf("  Item2: %d\r\n", val2);
```

### エラーハンドリング

```cpp
static int cmd_example(int argc, char** argv)
{
    auto& console = Console::getInstance();

    // 引数チェック
    if (argc < 2) {
        console.print("Error: argument required\r\n");
        return 1;  // エラーコードを返す
    }

    // 範囲チェック
    int value = atoi(argv[1]);
    if (value < 0 || value > 100) {
        console.print("Error: value must be 0-100\r\n");
        return 1;
    }

    // 成功
    return 0;
}
```

### 長時間処理

```cpp
static int cmd_longop(int argc, char** argv)
{
    auto& console = Console::getInstance();

    console.print("Starting long operation...\r\n");

    for (int i = 0; i < 10; i++) {
        // 進捗表示
        console.printf("Progress: %d%%\r\n", (i + 1) * 10);

        // 処理
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    console.print("Complete\r\n");
    return 0;
}
```

## 5. 新規コマンドファイルの作成

新しいカテゴリのコマンドを追加する場合:

### ファイル作成: `cmd_newcat.cpp`

```cpp
/**
 * @file cmd_newcat.cpp
 * @brief New category commands
 */

#include "console.hpp"
#include "esp_console.h"
#include <cstring>

using namespace stampfly;

// コマンドハンドラ
static int cmd_newcmd(int argc, char** argv)
{
    auto& console = Console::getInstance();
    console.print("New command executed\r\n");
    return 0;
}

// 登録関数（ヘッダで宣言）
void register_newcat_commands()
{
    const esp_console_cmd_t newcmd = {
        .command = "newcmd",
        .help = "New command description",
        .hint = NULL,
        .func = &cmd_newcmd,
        .argtable = NULL,
    };
    esp_console_cmd_register(&newcmd);
}
```

### ヘッダに宣言を追加: `commands.hpp`

```cpp
void register_newcat_commands();
```

### Console で呼び出し: `console.cpp`

```cpp
void Console::registerAllCommands()
{
    // ... 既存の登録 ...
    register_newcat_commands();  // 追加
}
```

### CMakeLists.txt に追加

```cmake
idf_component_register(
    SRCS
        "console.cpp"
        "commands/cmd_system.cpp"
        # ... 既存 ...
        "commands/cmd_newcat.cpp"  # 追加
    # ...
)
```

---

<a id="english"></a>

## 1. Overview

StampFly's CLI system is built on ESP-IDF Console, allowing the same commands to be used from both Serial CLI and WiFi CLI.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    sf_svc_console                           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         Command Implementation (add here)           │   │
│  │  commands/cmd_system.cpp   - System commands        │   │
│  │  commands/cmd_sensor.cpp   - Sensor commands        │   │
│  │  commands/cmd_motor.cpp    - Motor commands         │   │
│  │  commands/cmd_comm.cpp     - Communication commands │   │
│  │  commands/cmd_calib.cpp    - Calibration commands   │   │
│  │  commands/cmd_control.cpp  - Control commands       │   │
│  │  commands/cmd_misc.cpp     - Miscellaneous          │   │
│  └─────────────────────────────────────────────────────┘   │
│                          ↓                                  │
│              esp_console_cmd_register()                     │
└─────────────────────────────────────────────────────────────┘
                    ↑                    ↑
        ┌───────────┴───────┐    ┌───────┴───────┐
        │    Serial CLI     │    │   WiFi CLI    │
        └───────────────────┘    └───────────────┘
```

## 2. Adding a Command

### Step 1: Choose the appropriate file

| Category | File |
|----------|------|
| System (status, reboot, etc.) | `cmd_system.cpp` |
| Sensors (sensor, loglevel, etc.) | `cmd_sensor.cpp` |
| Motors | `cmd_motor.cpp` |
| Communication (comm, pair, etc.) | `cmd_comm.cpp` |
| Calibration | `cmd_calib.cpp` |
| Control (trim, gain, etc.) | `cmd_control.cpp` |
| Miscellaneous | `cmd_misc.cpp` |
| **New category** | Create new file `cmd_xxx.cpp` |

File path: `firmware/vehicle/components/sf_svc_console/commands/`

### Step 2: Implement the command handler

```cpp
static int cmd_mycommand(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (argc == 1) {
        console.print("Usage: mycommand <subcommand>\r\n");
        return 0;
    }

    const char* subcmd = argv[1];

    if (strcmp(subcmd, "status") == 0) {
        console.print("Current status: OK\r\n");
        return 0;
    }

    console.printf("Unknown subcommand: %s\r\n", subcmd);
    return 1;
}
```

### Step 3: Register the command

```cpp
const esp_console_cmd_t mycommand_cmd = {
    .command = "mycommand",
    .help = "My custom command [status|set <value>]",
    .hint = NULL,
    .func = &cmd_mycommand,
    .argtable = NULL,
};
esp_console_cmd_register(&mycommand_cmd);
```

### Step 4: Update help command

Add entry in `cmd_help()` function in `cmd_system.cpp`.

### Step 5: Update tab completion (optional)

Add to completion list in `wifi_cli.cpp`.

## 3. Best Practices

- Use `\r\n` for line endings (Telnet compatibility)
- Return 0 for success, non-zero for errors
- Validate arguments before processing
- Use `Console::printf()` for formatted output
- Access global state via `globals.hpp`
