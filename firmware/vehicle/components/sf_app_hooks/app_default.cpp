/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_default.cpp
 * @brief Default sf::app::* implementation, compiled only when no application
 *        directory (SF_APP_DIR) is present. Reproduces vehicle's stock
 *        behavior exactly — this file is what CMakeLists.txt swaps OUT for
 *        the user's own *.cpp.
 *        既定の sf::app::* 実装。アプリディレクトリ（SF_APP_DIR）が無いときのみ
 *        コンパイルされる。vehicle の既定挙動をそのまま再現する — ユーザーの
 *        *.cpp に置き換えられるのがこのファイル（CMakeLists.txt 参照）。
 *
 * controller() and estimator() are moved here UNCHANGED from their previous
 * homes (control_task.cpp:63's file-scope `static sf::PidController` and
 * imu_task.cpp's `createEstimator()`) — only the calling convention changed
 * (a function returning IController&/IEstimator& instead of a bare static or
 * a raw-pointer-returning factory), not the behavior.
 *
 * controller() と estimator() は、以前の置き場所（control_task.cpp:63 の
 * ファイルスコープ `static sf::PidController` と imu_task.cpp の
 * `createEstimator()`）から挙動を変えずに移設した — 呼び出し規約（生の
 * static／ポインタを返すファクトリ、ではなく IController&/IEstimator& を
 * 返す関数）だけが変わった。
 *
 * @design app_hooks.hpp — sf::app::controller/estimator/start contract   [OK]
 * @design docs/plans/sf-app-sils-plan.md §2 — Default app hooks           [OK]
 */

#include "app_hooks.hpp"

#include "esp_log.h"

#include "complementary_estimator.hpp"
#include "eskf_estimator.hpp"
#include "params.hpp"
#include "pid_controller.hpp"

static const char* TAG = "AppDefault";

namespace sf::app {

sf::IController& controller()
{
    // Function-local static: constructed once (first call), matching the
    // lifetime of the former file-scope `static sf::PidController controller;`
    // in control_task.cpp. init() is called exactly once, on that first call —
    // the hook contract says ControlTask calls this once, but the guard costs
    // nothing and protects against a future caller that does not honor it.
    // 関数内 static: 初回呼び出しで構築される — control_task.cpp にあった
    // 旧来のファイルスコープ `static sf::PidController controller;` と同じ
    // 寿命。init() は最初の呼び出しで一度だけ実行する — フック契約では
    // ControlTask が1回だけ呼ぶ約束だが、ガードは無害で、契約を守らない
    // 将来の呼び出し元からも保護する。
    static sf::PidController pid_controller;
    static bool initialized = false;
    if (!initialized) {
        pid_controller.init();
        initialized = true;
    }
    return pid_controller;
}

sf::IEstimator& estimator()
{
    // Estimator factory: select by estimator.type (0 = ESKF, 1 = complementary),
    // construct statically (no heap), initialize, and return via IEstimator.
    // Moved unchanged from imu_task.cpp's createEstimator() (RESET_PLAN P2:
    // algorithm-independence) — this is still the only place that names the
    // concrete estimator types; everything else uses IEstimator.
    // 推定器ファクトリ: estimator.type（0=ESKF, 1=相補）で選び、静的生成・
    // 初期化して IEstimator で返す。imu_task.cpp の createEstimator() から
    // 挙動を変えず移設（RESET_PLAN P2: アルゴリズム非依存）— 具象型を知るのは
    // 今もここだけで、他は全て IEstimator を使う。
    static sf::EskfEstimator eskf;
    static sf::ComplementaryEstimator comp;
    int32_t type = 0;
    sf::params::get_int("estimator.type", type);
    if (type == 1) {
        comp.init();
        ESP_LOGI(TAG, "Estimator: complementary filter (attitude + rate)");
        return comp;
    }
    eskf.init();
    ESP_LOGI(TAG, "Estimator: ESKF (15-state)");
    return eskf;
}

void start()
{
    // No application present — nothing to start.
    // アプリ無し — 起動するものはない。
}

}  // namespace sf::app
