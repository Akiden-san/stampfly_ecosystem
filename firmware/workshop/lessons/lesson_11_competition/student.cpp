#include "workshop_api.hpp"
#include <cmath>

// Rate PID state (inner loop)
// 角速度PID状態変数（内側ループ）
static float roll_rate_int = 0, pitch_rate_int = 0, yaw_rate_int = 0;
static float roll_rate_prev = 0, pitch_rate_prev = 0, yaw_rate_prev = 0;

// Attitude PID state (outer loop)
// 姿勢PID状態変数（外側ループ）
static float roll_angle_int = 0, pitch_angle_int = 0;
static float roll_angle_prev = 0, pitch_angle_prev = 0;

void setup()
{
    ws::print("Lesson 11: Competition - Optimize your code!");
}

void loop_400Hz(float dt)
{
    // TODO: Optimize gains for competition
    // TODO: Add altitude hold using ws::estimated_altitude()
    // TODO: Try cascade control (attitude -> rate)
    // TODO: ゲインを最適化する
    // TODO: ws::estimated_altitude()で高度保持を追加する
    // TODO: カスケード制御（姿勢 -> 角速度）を試す

    // ── Outer loop: stick -> angle -> rate target ──
    // 外側ループ: スティック -> 角度 -> 角速度目標
    constexpr float angle_Kp = 5.0f;
    constexpr float max_angle = 0.5f;  // rad (~30 deg)
    constexpr float rate_max = 1.0f;

    float roll_angle_target  = ws::rc_roll()  * max_angle;
    float pitch_angle_target = ws::rc_pitch() * max_angle;

    float roll_angle_error  = roll_angle_target  - ws::estimated_roll();
    float pitch_angle_error = pitch_angle_target - ws::estimated_pitch();

    float roll_rate_target  = angle_Kp * roll_angle_error;
    float pitch_rate_target = angle_Kp * pitch_angle_error;
    float yaw_rate_target   = ws::rc_yaw() * 5.0f;

    // Clamp rate targets / 角速度目標をクランプ
    constexpr float rate_clamp = rate_max * 3.0f;
    if (roll_rate_target  >  rate_clamp) roll_rate_target  =  rate_clamp;
    if (roll_rate_target  < -rate_clamp) roll_rate_target  = -rate_clamp;
    if (pitch_rate_target >  rate_clamp) pitch_rate_target =  rate_clamp;
    if (pitch_rate_target < -rate_clamp) pitch_rate_target = -rate_clamp;

    // ── Inner loop: rate PID ──
    // 内側ループ: 角速度PID
    constexpr float Kp = 0.5f, Ki = 0.3f, Kd = 0.005f;
    constexpr float Kp_yaw = 2.0f, Ki_yaw = 0.5f, Kd_yaw = 0.01f;
    constexpr float int_limit = 0.5f;

    float roll_error  = roll_rate_target  - ws::gyro_x();
    float pitch_error = pitch_rate_target - ws::gyro_y();
    float yaw_error   = yaw_rate_target   - ws::gyro_z();

    // Integral with anti-windup
    // 積分（アンチワインドアップ付き）
    roll_rate_int  += roll_error  * dt;
    pitch_rate_int += pitch_error * dt;
    yaw_rate_int   += yaw_error   * dt;

    if (roll_rate_int  >  int_limit) roll_rate_int  =  int_limit;
    if (roll_rate_int  < -int_limit) roll_rate_int  = -int_limit;
    if (pitch_rate_int >  int_limit) pitch_rate_int =  int_limit;
    if (pitch_rate_int < -int_limit) pitch_rate_int = -int_limit;
    if (yaw_rate_int   >  int_limit) yaw_rate_int   =  int_limit;
    if (yaw_rate_int   < -int_limit) yaw_rate_int   = -int_limit;

    // Derivative
    // 微分
    float roll_d  = (roll_error  - roll_rate_prev)  / dt;
    float pitch_d = (pitch_error - pitch_rate_prev) / dt;
    float yaw_d   = (yaw_error   - yaw_rate_prev)   / dt;
    roll_rate_prev  = roll_error;
    pitch_rate_prev = pitch_error;
    yaw_rate_prev   = yaw_error;

    // PID output
    // PID出力
    float roll_out  = Kp     * roll_error  + Ki     * roll_rate_int  + Kd     * roll_d;
    float pitch_out = Kp     * pitch_error + Ki     * pitch_rate_int + Kd     * pitch_d;
    float yaw_out   = Kp_yaw * yaw_error   + Ki_yaw * yaw_rate_int   + Kd_yaw * yaw_d;

    ws::motor_mixer(ws::rc_throttle(), roll_out, pitch_out, yaw_out);
}
