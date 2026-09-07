/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app.cpp
 * @brief sf::app::* implementation for this application (12_app_task_hello):
 *        the stock controller/estimator, plus one additional FreeRTOS task
 *        that reads the Topic API (sf::api::*) once a second and logs it.
 *        本アプリ（12_app_task_hello）の sf::app::* 実装: 標準コントローラ／
 *        推定器に加え、Topic API（sf::api::*）を1秒ごとに読んでログ出力する
 *        追加 FreeRTOS タスクを1つ持つ。
 *
 * This is the minimal shape of `sf::app::start()`: it only READS topics
 * (sf::api::estimate_latest(), sf::api::is_armed()) — there is no write-side
 * Topic API yet (docs/plans/sf-app-sils-plan.md Phase 4, not implemented).
 * これは `sf::app::start()` の最小形: Topic を「読む」だけ
 * （sf::api::estimate_latest(), sf::api::is_armed()）— 書き込み系 Topic API は
 * まだ無い（docs/plans/sf-app-sils-plan.md Phase 4, 未実装）。
 *
 * @design app_hooks.hpp — sf::app::controller/estimator/start contract   [OK]
 * @design sf_api.hpp — L1 read-only Topic API                            [OK]
 * @design docs/plans/sf-app-sils-plan.md §4 Phase 2 — 12_app_task_hello   [OK]
 */

#include "app_hooks.hpp"
#include "stock_hooks.hpp"
#include "sf_api.hpp"
#include "sf_math.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "AppHello";

namespace {

// Task tuning constants, scoped to this file (not vehicle's own
// main/config.hpp — an application does not edit the vehicle body). The
// priority sits below every standard task's (lowest standard priority is
// config::PRIORITY_CLI/PRIORITY_LOG = 5, main/config.hpp) so this task never
// competes with flight-critical work; the core matches the other
// low-priority, non-realtime tasks (CLITask/LogTask, core 0).
// タスク調整用定数。このファイルに閉じたスコープ（vehicle 本体の
// main/config.hpp は編集しない — アプリは vehicle 本体を書き換えない）。
// 優先度は全標準タスクの下（標準最低は config::PRIORITY_CLI/PRIORITY_LOG=5、
// main/config.hpp）に置き、飛行に関わる処理と競合させない。コアは他の
// 低優先度・非リアルタイムタスク（CLITask/LogTask, core 0）と揃える。
constexpr UBaseType_t kHelloTaskPriority   = 4;
constexpr uint32_t    kHelloTaskStackBytes = 4096;
constexpr BaseType_t  kHelloTaskCore       = 0;
constexpr TickType_t  kHelloIntervalTicks  = pdMS_TO_TICKS(1000);  // 1 second / 1秒

// Radian-to-degree conversion, named so the log line has no magic numbers.
// ラジアン→度変換。ログ出力にマジックナンバーを残さないよう名前を付ける。
constexpr float kRadToDegree = 180.0f / 3.14159265f;

/// Task body: once a second, read the latest fused state estimate and log
/// attitude (degrees) and armed state. Runs forever — start() below creates
/// exactly one instance.
/// タスク本体: 1秒ごとに最新の統合状態推定を読み、姿勢（度）とARM状態を
/// ログ出力する。無限ループで動作 — 下の start() が1つだけ生成する。
void HelloTask(void*)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake_time, kHelloIntervalTicks);

        // StateEstimate::attitude is a quaternion [w,x,y,z] (data_types.hpp);
        // sf::math::Quat::to_euler() converts it to roll/pitch/yaw [rad].
        // StateEstimate::attitude はクォータニオン [w,x,y,z]（data_types.hpp）。
        // sf::math::Quat::to_euler() で roll/pitch/yaw [rad] に変換する。
        sf::StateEstimate state = sf::api::estimate_latest();
        sf::math::Quat attitude(state.attitude[0], state.attitude[1],
                                 state.attitude[2], state.attitude[3]);
        sf::math::Vec3 euler = attitude.to_euler();

        ESP_LOGI(TAG, "roll=%.1f pitch=%.1f yaw=%.1f deg, armed=%d",
                 euler.x * kRadToDegree, euler.y * kRadToDegree,
                 euler.z * kRadToDegree, sf::api::is_armed());
    }
}

}  // namespace

namespace sf::app {

sf::IController& controller()
{
    // This template does not touch the controller — use the vehicle's
    // standard PidController.
    // 本テンプレートはコントローラを触らない — vehicle 標準の PidController。
    return stock::controller();
}

sf::IEstimator& estimator()
{
    // Nor the estimator — use the vehicle's standard one (ESKF / complementary,
    // by param estimator.type).
    // 推定器も同様 — vehicle 標準（estimator.type で ESKF／相補）。
    return stock::estimator();
}

void start()
{
    xTaskCreatePinnedToCore(HelloTask, "AppHelloTask", kHelloTaskStackBytes,
                             nullptr, kHelloTaskPriority, nullptr, kHelloTaskCore);
}

}  // namespace sf::app
