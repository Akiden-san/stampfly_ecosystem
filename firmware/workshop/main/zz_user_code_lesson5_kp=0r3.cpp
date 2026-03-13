#include "workshop_api.hpp"

// =========================================================================
// Lesson 5: Rate P-Control + First Flight
// レッスン 5: レートP制御 + 初フライト
// =========================================================================
//
// Goal: Implement proportional (P) feedback control on angular rate.
// 目標: 角速度に対する比例(P)フィードバック制御を実装する
//
// Block diagram:
//   rc_stick --> [scale] --> target_rate -->(+)--> error --> [Kp] --> output --> motor_mixer
//                                           ^(-)
//                                           |
//                                      gyro (actual rate)

void setup()
{
    ws::print("Lesson 5: Rate P-Control");

    // TODO: Set your WiFi channel (1, 6, or 11)
    // TODO: 自分のWiFiチャンネルを設定する（1, 6, 11のいずれか）
    // ws::set_channel(1);
    ws::set_channel(1);
}

void loop_400Hz(float dt)
{
    (void)dt;  
    // dt is not used in this lesson, but you can use it for more advanced control later
    // dtはこのレッスンでは使用しませんが、後でより高度な制御に使用できます
    // Safety: only run control when armed
    // 安全: ARM状態のときだけ制御を実行
    if (!ws::is_armed()) {
        ws::motor_stop_all();
        ws::led_color(0, 255, 0);  // Green when disarmed / disarmedのときは緑色
        return;
    }

    float throttle = ws::rc_throttle();

    // --- P gain ---
    // P ゲイン
    float Kp_roll  = 0.3f;  // TODO: tune this value / この値を調整 default 0.5f
    float Kp_pitch = 0.3f;  // TODO: tune this value / この値を調整 default 0.5f
    float Kp_yaw   = 2.0f;  // TODO: tune this value / この値を調整 default 2.0f

    // --- Maximum angular rate [rad/s] ---
    // 最大角速度 [rad/s]
    float rate_max_rp  = 1.0f;  // roll/pitch max rate
    float rate_max_yaw = 5.0f;  // yaw max rate

    // --- Roll axis ---
    // ロール軸
    // TODO: Compute target rate from stick input
    // ヒント: float roll_target = ws::rc_roll() * rate_max_rp;
    float roll_target = ws::rc_roll() * rate_max_rp;

    // TODO: Read actual rate from gyro
    // ヒント: float roll_actual = ws::gyro_x();
    float roll_actual = ws::gyro_x();

    // TODO: Compute error (target - actual)
    // TODO: Apply P gain to get control output
    float roll_error = roll_target - roll_actual;   // Error between target and actual / 目標と実際の誤差
    float roll_output = Kp_roll * roll_error;       // Replace with your calculation / 計算結果に置き換える

    // --- Pitch axis ---
    // ピッチ軸
    // TODO: Same as roll but for pitch axis
    // ヒント: ws::rc_pitch(), ws::gyro_y()
    float pitch_target = ws::rc_pitch() * rate_max_rp;
    float pitch_actual = ws::gyro_y();
    float pitch_error = pitch_target - pitch_actual;
    float pitch_output = Kp_pitch * pitch_error;  // Replace with your calculation / 計算結果に置き換える

    // --- Yaw axis ---
    // ヨー軸
    // TODO: Same as roll but for yaw axis
    // ヒント: ws::rc_yaw(), ws::gyro_z()
    float yaw_target = ws::rc_yaw() * rate_max_yaw;
    float yaw_actual = ws::gyro_z();
    float yaw_error = yaw_target - yaw_actual;
    float yaw_output = Kp_yaw * yaw_error;  // Replace with your calculation / 計算結果に置き換える

    // --- Apply to motors ---
    // モーターに適用
    ws::motor_mixer(throttle, roll_output, pitch_output, yaw_output);
}
