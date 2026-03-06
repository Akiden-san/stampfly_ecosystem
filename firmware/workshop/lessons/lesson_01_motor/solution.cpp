#include "workshop_api.hpp"

// =========================================================================
// Lesson 1: Motor Control - Solution
// レッスン 1: モータ制御 - 解答
// =========================================================================

static uint32_t motor_timer = 0;
static int current_motor = 1;

// @@snippet: setup
void setup()
{
    ws::print("Lesson 1: Motor Control - Solution");

    // Enable motor output (motors won't spin without this)
    // モーター出力を有効化（これがないとモーターは回らない）
    ws::arm();
}
// @@end-snippet: setup

// @@snippet: loop
void loop_400Hz(float dt)
{
    // Cycle through motors: each motor spins for 2 seconds
    // モータを順番に回す: 各モータ2秒ずつ
    motor_timer++;

    // 400Hz * 2s = 800 ticks per motor
    // 400Hz × 2秒 = 800ティック/モータ
    int phase = (motor_timer / 800) % 4;
    current_motor = phase + 1;

    // Set only the current motor, stop the others
    // 現在のモータのみ回転、他は停止
    for (int m = 1; m <= 4; m++) {
        if (m == current_motor) {
            ws::motor_set_duty(m, 0.1f);
        } else {
            ws::motor_set_duty(m, 0.0f);
        }
    }

    // Print which motor is active every 2 seconds
    // 2秒ごとにアクティブなモータを表示
    if (motor_timer % 800 == 0) {
        ws::print("Motor %d active", current_motor);
    }
}
// @@end-snippet: loop
