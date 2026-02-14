# Lesson 11: Competition

## Goal / 目標
Apply everything you have learned to optimize your drone for a flight competition.

これまで学んだ全てを活用して、フライトコンペティションに向けてドローンを最適化する。

## Overview / 概要

This is the final lesson. You start with a working cascade controller (attitude + rate PID)
and optimize it for the best performance. There is no single "correct" solution - this is
about engineering judgment and experimentation.

これが最終レッスン。動作するカスケード制御器（姿勢 + 角速度PID）からスタートし、
最高のパフォーマンスを目指して最適化する。唯一の「正解」はない - エンジニアリングの
判断力と実験がカギ。

## Starting Template / スタートテンプレート

The `student.cpp` provides a **cascade controller**:
```
Stick Input ──> [Attitude PID] ──> [Rate PID] ──> Motor Mixer
                 (outer loop)      (inner loop)
```

| Parameter | Default Value | Description |
|-----------|--------------|-------------|
| `angle_Kp` | 5.0 | Attitude P-gain (outer loop) |
| `max_angle` | 0.5 rad (30 deg) | Maximum tilt angle |
| `Kp` (rate) | 0.5 | Rate P-gain (inner loop) |
| `Ki` (rate) | 0.3 | Rate I-gain |
| `Kd` (rate) | 0.005 | Rate D-gain |
| `Kp_yaw` | 2.0 | Yaw rate P-gain |
| `Ki_yaw` | 0.5 | Yaw rate I-gain |
| `Kd_yaw` | 0.01 | Yaw rate D-gain |

## Competition Rules / コンペティションルール

### Categories / カテゴリ
| Category | Description | Scoring |
|----------|-------------|---------|
| Stability | Hover in place for 30 seconds | Smallest position drift wins |
| Agility | Complete a square pattern (1m x 1m) | Fastest time wins |
| Altitude hold | Hold 0.5m altitude for 30 seconds | Smallest altitude error wins |

### Safety Rules / 安全ルール
- Propeller guards must be installed at all times
- Maximum altitude: 1.5m
- Flight area boundary: 3m x 3m
- Instructor must approve code before flight
- Emergency stop: release all sticks (auto-disarm)

## Optimization Ideas / 最適化のアイデア

### 1. Tune PID Gains / PIDゲインの調整
```
Start with rate PID (inner loop), then tune attitude PID (outer loop).
内側ループ（角速度PID）から調整し、次に外側ループ（姿勢PID）を調整する。
```

| Step | Action | What to observe |
|------|--------|----------------|
| 1 | Increase rate Kp until oscillation | Response speed |
| 2 | Back off Kp by 20% | Stability margin |
| 3 | Increase Ki until steady-state error is zero | Offset correction |
| 4 | Add Kd to reduce overshoot | Damping |
| 5 | Tune attitude Kp for desired angle response | Outer loop response |

### 2. Add Altitude Hold / 高度保持を追加する
```cpp
// Example altitude hold (add to loop_400Hz)
// 高度保持の例
float alt_target = 0.5f;  // Target: 0.5m
float alt_error = alt_target - ws::estimated_altitude();
float alt_Kp = 0.5f;
float thrust = base_thrust + alt_Kp * alt_error;
```

### 3. Add D-Term Filtering / D項にフィルタを追加する
```cpp
// Low-pass filter on D-term to reduce noise
// D項にローパスフィルタをかけてノイズを低減
static float d_filtered = 0;
float d_raw = (error - prev_error) / dt;
d_filtered = 0.8f * d_filtered + 0.2f * d_raw;  // alpha = 0.8
```

### 4. Use Telemetry for Analysis / テレメトリで解析する
```cpp
ws::telemetry_send("roll_error", roll_error);
ws::telemetry_send("roll_out", roll_out);
ws::telemetry_send("altitude", ws::estimated_altitude());
```
Then view with `sf log wifi` to analyze performance.

## Steps / 手順
1. `sf lesson switch 11`
2. Flash and test the default cascade controller
3. Record baseline telemetry: `sf log wifi`
4. Pick an optimization strategy from the list above
5. Modify `student.cpp`, flash, and test
6. Compare telemetry before and after
7. Iterate until you are satisfied with performance
8. Submit your code for competition!

## Tips / ヒント
- Change one parameter at a time
- Always record telemetry before and after changes
- If the drone becomes unstable, reduce all gains by 50%
- Use `sf log wifi` and plot in Jupyter to see the effect of changes
- Battery voltage affects thrust - check `ws::battery_voltage()`

## Key Concepts / キーコンセプト
- Cascade control: outer loop sets targets for inner loop
- PID tuning is an iterative, experimental process
- Altitude hold requires a vertical position estimate
- D-term filtering reduces noise amplification
- Engineering is about trade-offs: stability vs agility
