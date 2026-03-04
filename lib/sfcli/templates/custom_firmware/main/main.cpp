/**
 * {{PROJECT_NAME}} - Custom StampFly Firmware
 * {{PROJECT_NAME}} - カスタムStampFlyファームウェア
 *
 * Implement setup() and loop_400Hz() to create your custom firmware.
 * setup() と loop_400Hz() を実装してカスタムファームウェアを作成します。
 *
 * All ws:: API functions and StampFlyState are available.
 * ws:: API 全関数と StampFlyState が使用可能です。
 *
 * Build: sf build {{PROJECT_NAME}}
 * Flash: sf flash {{PROJECT_NAME}} -m
 */

#include "workshop_api.hpp"

void setup()
{
    ws::print("{{PROJECT_NAME}}: Ready");
}

void loop_400Hz(float dt)
{
    // Example: Print sensor data at 50 Hz
    // 例: 50 Hz でセンサデータを出力
    static uint32_t tick = 0;
    tick++;
    if (tick % 8 != 0) return;  // 400 / 8 = 50 Hz

    // Teleplot output (>name:value format)
    // Teleplot 出力（>name:value 形式）
    ws::print(">baro_alt:%.2f", ws::baro_altitude());
    ws::print(">tof_bot:%.3f", ws::tof_bottom());
}
