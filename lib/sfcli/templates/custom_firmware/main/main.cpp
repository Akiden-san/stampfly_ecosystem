/**
 * {{PROJECT_NAME}} - Custom StampFly Firmware
 * {{PROJECT_NAME}} - カスタムStampFlyファームウェア
 *
 * This is a custom firmware project for StampFly.
 * Access all sensors and state via StampFlyState.
 * StampFlyの全センサとステートにアクセスできるカスタムファームウェア。
 *
 * Build: sf build {{PROJECT_NAME}}
 * Flash: sf flash {{PROJECT_NAME}} -m
 */

#include <cstdio>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stampfly_state.hpp"

// User loop period / ユーザーループ周期
static constexpr int LOOP_PERIOD_MS = 20;  // 50 Hz

extern "C" void app_main(void)
{
    printf("{{PROJECT_NAME}}: Starting...\n");

    // Get StampFlyState singleton
    // StampFlyStateシングルトンを取得
    auto& state = StampFlyState::getInstance();

    // Main loop / メインループ
    while (true) {
        // Example: Read sensors / 例: センサ読み取り
        // auto imu = state.getIMUData();
        // auto baro = state.getBaroData();
        // auto tof = state.getToFData(ToFPosition::BOTTOM);
        // auto flow = state.getFlowData();
        // auto mag = state.getMagData();
        // auto power = state.getPowerData();
        // auto att = state.getAttitudeEuler();

        // Example: Teleplot output / 例: Teleplot出力
        // printf(">sensor_val:%.2f\n", value);

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
