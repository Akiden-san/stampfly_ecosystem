"""
Plant Model Fitting from Closed-Loop Flight Data
閉ループフライトデータからのプラントモデル同定

Identifies open-loop plant parameters from P-control flight data:
  G_p(s) = K / (s * (tau_m * s + 1))

Where:
  K    : Plant gain [rad/s^2 per duty]
  tau_m: Motor time constant [s]

Algorithm:
  Plant I/O is reconstructed from telemetry via one of two INPUT MODES
  (--input {auto,duty,kp}; see fit_plant()):

    "duty" -- the actual plant input, recovered from the 400Hz motor duty
      log (motor_duty_FR/RR/RL/FL, from firmware sending the kPktDuty400
      entry, see data_stream_wire.hpp / udp_capture.py). The 4 motor duties
      are inverted to recover the per-axis differential command the
      rate-loop PID actually output, without having to know Kp or assume it
      never changed (autotune, gain schedule) or that the actuator never
      saturated. WHICH inversion is used is selected by --mixer
      {legacy,vehicle} (default "legacy"):
        "legacy" -- inverts the SIMPLE LINEAR "voltage-scale" X-quad mixer
          (ws_internal.hpp motor_mixer / vehicle_old setMixerOutput): duty
          IS the differential command, additively mixed, no battery-voltage
          or motor-curve nonlinearity. Correct for firmware/workshop (`sf
          lesson`) and firmware/vehicle_old logs. See
          _duty_differential_legacy_linear().
        "vehicle" -- inverts firmware/vehicle's ACTUAL mixer
          (sf_actuator/actuator.cpp mixerCompute()): a physical-unit B^-1
          allocation (thrust [N] / torque [Nm]) followed by a NONLINEAR
          motor curve (duty = f(sqrt(T/Ct)) / Vbat). Required for logs from
          `sf app` (firmware/vehicle-based custom controllers) -- the
          "legacy" inversion silently fits the WRONG signal on those (looked
          like a fit failure / physically-impossible K,tau_m in practice).
          See _duty_differential_vehicle().
      The CSV column layout is IDENTICAL either way (same LogStreamSample
      wire struct), so this cannot be auto-detected from the file alone --
      the caller must know which firmware produced the log.
    "kp" -- the LEGACY reconstruction: since Kp is assumed known and
      constant, u_plant is approximated as Kp * (target - gyro). Two CSV
      schemas are auto-detected from the header (see _detect_csv_format):
        "stream" (400Hz Data Stream, `sf log wifi -o *.csv`):
          target(t) = rate_ref_<axis>(t)          <- already rad/s
        "legacy" (pre-migration analysis CSV):
          target(t) = ctrl_<axis>(t) * rate_max   <- normalized stick * rate_max
    "auto" (default) -- "duty" when motor_duty_* columns exist AND --kp was
      NOT given; otherwise "kp" (which then requires --kp).

  Either way: y_plant(t) = gyro(t). The open-loop model is fitted directly
  via MSE minimization:
    u_plant -> G_p(s) -> y_simulated
    minimize |y_simulated - y_plant|^2  ->  K, tau_m

プラント入出力は2つの「入力モード」（--input {auto,duty,kp}。fit_plant()
参照）のいずれかで再構成する:

  "duty" -- 400Hz モータduty ログ（motor_duty_FR/RR/RL/FL。ファームが
    kPktDuty400 エントリを送っている必要がある。data_stream_wire.hpp /
    udp_capture.py 参照）から復元した「実際のプラント入力」。4モータduty を
    逆算し、レートループ PID が実際に出力した軸別の差動指令を復元する —
    Kp を知る必要も、Kp が飛行中に不変（自動チューニング・ゲイン
    スケジューリング無し）だったことも、アクチュエータが飽和しなかった
    ことも仮定しない。**どちらの逆算を使うかは --mixer {legacy,vehicle}
    で選ぶ（既定 "legacy"）**:
      "legacy" -- 単純な線形「電圧スケール」X字ミキサー（ws_internal.hpp
        motor_mixer / vehicle_old の setMixerOutput）を逆算する: duty が
        そのまま差動指令（加減算のみ、バッテリ電圧も非線形モータ曲線も
        無し）。firmware/workshop（`sf lesson`）と firmware/vehicle_old の
        ログに正しい。_duty_differential_legacy_linear() 参照。
      "vehicle" -- firmware/vehicle の実際のミキサー
        （sf_actuator/actuator.cpp mixerCompute()）を逆算する: 物理単位
        （推力[N]・トルク[Nm]）の B^-1 配分＋非線形モータ曲線
        （duty = f(√(T/Ct)) / Vbat）。`sf app`（firmware/vehicle ベースの
        自作コントローラ）のログにはこちらが必須 —
        "legacy" 逆算のまま使うと誤った信号を静かにフィットしてしまう
        （実際にはフィット失敗や物理的にありえない K・tau_m として現れる）。
        _duty_differential_vehicle() 参照。
    CSV の列構成はどちらの由来でも同一（同じ LogStreamSample 電文構造体）
    なので、ファイル単体からは自動判別できない — 呼び出し側がどちらの
    ファームで記録したログかを知っている必要がある。
  "kp" -- 従来の再構成方式: Kp が既知・一定と仮定し、
    u_plant を Kp × (target − gyro) で近似する。CSV ヘッダから2種類の
    形式を自動判別する（_detect_csv_format 参照）:
      "stream"（400Hz Data Stream、`sf log wifi -o *.csv`）:
        target(t) = rate_ref_<axis>(t)            <- 既に rad/s
      "legacy"（移行前の分析用 CSV）:
        target(t) = ctrl_<axis>(t) × rate_max     <- 正規化スティック × rate_max
  "auto"（既定）-- motor_duty_* 列があり、かつ --kp 未指定なら "duty"。
    それ以外は "kp"（この場合 --kp が必須）。

いずれも: y_plant(t) = gyro(t)。開ループモデルは MSE 最小化で直接フィット
する:
  u_plant -> G_p(s) -> y_simulated
  minimize |y_simulated - y_plant|^2  ->  K, tau_m
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize

from .defaults import get_flat_defaults
from ._generated_params import EXPECTED_ARM, EXPECTED_KAPPA, EXPECTED_CT, \
    EXPECTED_IXX, EXPECTED_IYY, EXPECTED_IZZ


# Reference plant gains from L06 System Modeling. Units match --input duty's
# "legacy" mixer (--mixer legacy, the default): [rad/s^2 per differential-duty
# unit]. Do NOT use these for --mixer vehicle -- see
# REFERENCE_PLANT_GAINS_VEHICLE below, which is in the different physical
# units (Nm) that mode's u_plant is reconstructed in.
# L06 システムモデリングからの参照プラントゲイン。単位は --input duty の
# "legacy" ミキサー（--mixer legacy、既定）用: [rad/s^2 / 差動duty単位]。
# --mixer vehicle には使わないこと -- 下の REFERENCE_PLANT_GAINS_VEHICLE
# 参照（あちらは u_plant の単位が異なる[Nm]）。
REFERENCE_PLANT_GAINS: Dict[str, float] = {
    'roll': 102.0,    # [rad/s^2 per duty]
    'pitch': 70.0,
    'yaw': 19.0,
}

# Reference plant gains for --mixer vehicle: u_plant there is a physical
# differential TORQUE [Nm] (see _duty_differential_vehicle()), so the
# steady-state (motor-lag-free) plant gain is simply 1/I_axis (Newton's
# second law for rotation: domega/dt = torque / I) -- NOT the empirical
# duty-unit REFERENCE_PLANT_GAINS above. Also used to SEED the optimizer
# (K_init in fit_plant()): the duty-unit reference is off by several orders
# of magnitude in Nm terms and gives L-BFGS-B a useless starting point.
# I_axis is the SAME EXPECTED_IXX/IYY/IZZ tracked by `sf params check`
# (tools/params_audit/params_manifest.py "Ixx"/"Iyy"/"Izz" groups), so this
# stays in sync with the confirmed body-inertia measurement automatically.
# --mixer vehicle 用の参照プラントゲイン: u_plant は物理的な差動トルク
# [Nm]（_duty_differential_vehicle() 参照）なので、定常（モータ遅れ無視）
# ゲインは単純に 1/I_axis（回転のニュートンの第2法則:
# domega/dt = torque / I）—— 上の duty単位版 REFERENCE_PLANT_GAINS とは別物。
# 最適化の初期値（fit_plant() の K_init）にも使う: duty単位の参照値は
# Nm換算で桁が何桁も違い、L-BFGS-B に無意味な初期点を与えてしまう。
# I_axis は `sf params check`（tools/params_audit/params_manifest.py の
# "Ixx"/"Iyy"/"Izz" グループ）が追跡しているのと同じ EXPECTED_IXX/IYY/IZZ
# なので、確定済みの機体慣性実測値と自動的に同期する。
REFERENCE_PLANT_GAINS_VEHICLE: Dict[str, float] = {
    'roll': 1.0 / EXPECTED_IXX,    # [rad/s^2 per Nm]
    'pitch': 1.0 / EXPECTED_IYY,
    'yaw': 1.0 / EXPECTED_IZZ,
}

# Body FRD axis order (roll about x, pitch about y, yaw about z) -- same
# convention used throughout the repo (see e.g. tools/log_analyzer/rate_sysid.py
# and data_stream_wire.hpp), kept identical here so `--axis` selection and the
# gyro sign/axis mapping never drift from the README's coordinate system.
# Body FRD 軸順（roll=x軸周り, pitch=y軸周り, yaw=z軸周り）— リポジトリ全体で
# 使われる規約と同一（例: tools/log_analyzer/rate_sysid.py, data_stream_wire.hpp）。
# --axis 選択とジャイロの軸・符号対応が README の座標系からずれないよう、
# ここでも同じ規約を使う。
_AXIS_NAMES: Tuple[str, str, str] = ('roll', 'pitch', 'yaw')

# "stream" format (sf log wifi -o *.csv): gyro column per axis.
# "stream" 形式（sf log wifi -o *.csv）: 軸ごとのジャイロ列。
_STREAM_GYRO_COL: Dict[str, str] = {'roll': 'gyro_x', 'pitch': 'gyro_y', 'yaw': 'gyro_z'}
# "stream" format: rate target column per axis (already rad/s, see
# workshop_control_task.cpp publishLogStream() / ws::set_rate_target()).
# "stream" 形式: 軸ごとの角速度目標列（既に rad/s。
# workshop_control_task.cpp publishLogStream() / ws::set_rate_target() 参照）。
_STREAM_TARGET_COL: Dict[str, str] = {
    'roll': 'rate_ref_roll', 'pitch': 'rate_ref_pitch', 'yaw': 'rate_ref_yaw',
}

# "legacy" format: normalized stick column per axis (multiplied by --rate-max).
# "legacy" 形式: 軸ごとの正規化スティック列（--rate-max を掛ける）。
_LEGACY_CTRL_COL: Dict[str, str] = {
    'roll': 'ctrl_roll', 'pitch': 'ctrl_pitch', 'yaw': 'ctrl_yaw',
}
# "legacy" format: gyro column per axis, preferring the bias-corrected value
# when present (falls back to _STREAM_GYRO_COL's raw gyro_x/y/z).
# "legacy" 形式: 軸ごとのジャイロ列。バイアス補正済みがあればそちらを優先
# （無ければ _STREAM_GYRO_COL の生ジャイロ gyro_x/y/z にフォールバック）。
_LEGACY_GYRO_COL: Dict[str, str] = {
    'roll': 'gyro_corrected_x', 'pitch': 'gyro_corrected_y', 'yaw': 'gyro_corrected_z',
}

# Data Stream CSV column names for the 4 motor duties (400Hz when the
# firmware sends kPktDuty400, else 50Hz-forward-filled -- see
# udp_capture.py save_stream_csv()). Order matches the wire layout FR,RR,RL,FL.
# Data Stream CSV のモータduty 4列（ファームが kPktDuty400 を送っていれば
# 400Hz、無ければ50Hz前方補完 — udp_capture.py save_stream_csv() 参照）。
# 電文と同じ順序 FR,RR,RL,FL。
_DUTY_COLS: Tuple[str, str, str, str] = (
    'motor_duty_FR', 'motor_duty_RR', 'motor_duty_RL', 'motor_duty_FL',
)

# X-quad mixer differential-duty coefficient (ws_internal.hpp motor_mixer,
# reproduced digit-for-digit from vehicle_old's setMixerOutput):
#   M1 FR = T + k*(-R+P+Y)   M2 RR = T + k*(-R-P-Y)
#   M3 RL = T + k*( R-P+Y)   M4 FL = T + k*( R+P-Y)      k = 0.25/3.7
# X字クアッドミキサの差動duty係数（ws_internal.hpp motor_mixer。vehicle_old
# の setMixerOutput と数値まで一致させた原典の再現）。
_MIXER_K: float = 0.25 / 3.7


def _duty_differential_legacy_linear(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    axis: str,
) -> np.ndarray:
    """
    Invert the SIMPLE LINEAR "voltage-scale" X-quad mixer (ws_internal.hpp
    motor_mixer / vehicle_old setMixerOutput) to recover the per-axis
    differential duty command (R, P, or Y) that produced the 4 motor duties.
    --mixer legacy (the default). Correct for firmware/workshop (`sf lesson`)
    and firmware/vehicle_old logs ONLY -- for firmware/vehicle (`sf app`)
    logs use _duty_differential_vehicle() instead (see the module docstring
    for why the two firmwares need different inversions: vehicle's actual
    mixer is a physical B^-1 allocation through a NONLINEAR motor curve, not
    this linear one).
    単純な線形「電圧スケール」X字ミキサー（ws_internal.hpp motor_mixer /
    vehicle_old の setMixerOutput）を逆算し、4モータduty から軸別の差動
    duty指令（R/P/Y）を復元する。--mixer legacy（既定）。
    firmware/workshop（`sf lesson`）と firmware/vehicle_old のログにのみ
    正しい -- firmware/vehicle（`sf app`）のログには
    _duty_differential_vehicle() を使うこと（2つのファームで逆算が異なる
    理由はモジュール docstring 参照: vehicle の実ミキサーは非線形モータ
    曲線を介した物理単位の B^-1 配分であり、この線形式とは別物）。

    T cancels out in every combination below (it appears identically in all
    4 motor equations), so this needs no knowledge of thrust or hover duty --
    the result is u_plant in the SAME units the rate-loop PID's raw output
    entered the mixer with, whether or not the duty saturated.
    下式はどれも T が相殺する（4つのモータ式全てに同一に現れるため）ので、
    推力やホバーduty の知識は不要 -- 結果はレートループ PID の生出力が
    ミキサへ入ったのと同じ単位の u_plant になる（duty が飽和していても）。

    Args:
        duty_fr, duty_rr, duty_rl, duty_fl: motor duty arrays [0, 1]
        axis: 'roll', 'pitch', or 'yaw'

    Returns:
        Differential duty command array (same length/shape as the inputs).
    """
    if axis == 'roll':
        diff = -duty_fr - duty_rr + duty_rl + duty_fl
    elif axis == 'pitch':
        diff = duty_fr - duty_rr - duty_rl + duty_fl
    elif axis == 'yaw':
        diff = duty_fr - duty_rr + duty_rl - duty_fl
    else:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")
    return diff / (4.0 * _MIXER_K)


# -----------------------------------------------------------------------------
# --mixer vehicle: firmware/vehicle's ACTUAL mixer (sf_actuator/actuator.cpp
# mixerCompute() / thrustToDuty()), reproduced digit-for-digit. Unlike the
# "legacy" linear mixer above, duty here is NOT the differential command
# itself -- it is a NONLINEAR function of the per-motor thrust and the live
# battery voltage, so recovering the differential torque command requires
# inverting the full physical chain: duty -> volts -> omega -> thrust -> (B^-1
# combination) -> torque. See _thrust_from_duty() / _duty_differential_vehicle().
#
# --mixer vehicle: firmware/vehicle の実際のミキサー（sf_actuator/
# actuator.cpp の mixerCompute() / thrustToDuty()）を一字一句再現する。
# 上の "legacy" 線形ミキサーと異なり、duty はそのまま差動指令ではない --
# 各モータ推力と実電源電圧の非線形関数なので、差動トルク指令を復元するには
# 物理チェーン全体を逆算する必要がある: duty -> volts -> omega -> thrust ->
# （B^-1結合）-> torque。_thrust_from_duty() / _duty_differential_vehicle()
# 参照。
# -----------------------------------------------------------------------------

# B^-1 allocation geometry (actuator.cpp mixerCompute(), ARM_D/KAPPA):
#   T_FR = 0.25*(thrust - up/ARM_D + uq/ARM_D + ur/KAPPA)
#   T_RR = 0.25*(thrust - up/ARM_D - uq/ARM_D - ur/KAPPA)
#   T_RL = 0.25*(thrust + up/ARM_D - uq/ARM_D + ur/KAPPA)
#   T_FL = 0.25*(thrust + up/ARM_D + uq/ARM_D - ur/KAPPA)
# ARM_D/KAPPA are imported from tools/sysid/_generated_params.py (machine-
# generated from control/models/stampfly_physical.yaml by `sf params
# generate`) -- the SAME values `sf params check` verifies against
# actuator.cpp's ARM_D/KAPPA (tools/params_audit/params_manifest.py "arm"/
# "kappa" groups), so this cannot silently drift from firmware the way a
# hand-typed literal could.
# B^-1 配分幾何（actuator.cpp mixerCompute()、ARM_D/KAPPA）。ARM_D/KAPPA は
# tools/sysid/_generated_params.py から import する（`sf params generate` が
# control/models/stampfly_physical.yaml から機械生成）—— `sf params check`
# が actuator.cpp の ARM_D/KAPPA と照合しているのと同じ値（
# tools/params_audit/params_manifest.py の "arm"/"kappa" グループ）なので、
# 手書きリテラルのように黙ってファームからずれることがない。
_ARM_D: float = EXPECTED_ARM
_KAPPA: float = EXPECTED_KAPPA

# Motor curve (actuator.cpp thrustToDuty()): omega=sqrt(T/Ct);
# volts=Am*omega^2+Bm*omega+Cm; duty=volts/vbat. MOTOR_CT is imported the
# same way as ARM_D/KAPPA above (EXPECTED_CT, `sf params check` "C_T" group).
# MOTOR_AM/MOTOR_BM/MOTOR_CM are the "flight_anchored_motor_curve" family
# (control/models/stampfly_physical.yaml) -- NOT machine-generated on the
# Python side as of this writing (see params_manifest.py's own comment on its
# hand-typed EXPECTED_AM_FA/EXPECTED_BM_FA/EXPECTED_CM), so these three are
# hand-copied from actuator.cpp exactly as params_manifest.py's copies are,
# and registered in that same manifest ("Am_flight_anchored"/
# "Bm_flight_anchored"/"Cm" groups) so `sf params check` catches drift here
# too.
# モータ曲線（actuator.cpp の thrustToDuty()）: omega=sqrt(T/Ct);
# volts=Am*omega^2+Bm*omega+Cm; duty=volts/vbat。MOTOR_CT は上の ARM_D/KAPPA
# と同じく import する（EXPECTED_CT、`sf params check` "C_T" グループ）。
# MOTOR_AM/MOTOR_BM/MOTOR_CM は "flight_anchored_motor_curve" ファミリ
# （control/models/stampfly_physical.yaml）—— 本稿執筆時点で Python 側は
# 機械生成されていない（params_manifest.py 自身の EXPECTED_AM_FA/
# EXPECTED_BM_FA/EXPECTED_CM 手書きコメント参照）ため、この3つは
# params_manifest.py のコピーと同様に actuator.cpp から手動転記し、同じ
# マニフェスト（"Am_flight_anchored"/"Bm_flight_anchored"/"Cm" グループ）に
# 登録して `sf params check` がここのずれも検出できるようにする。
_MOTOR_CT: float = EXPECTED_CT
_MOTOR_AM: float = 6.0368e-8    # V/(rad/s)^2 -- flight_anchored_motor_curve.Am
_MOTOR_BM: float = 6.699042e-4  # V/(rad/s)   -- flight_anchored_motor_curve.Bm
_MOTOR_CM: float = 1.53e-2      # V           -- flight_anchored_motor_curve.Cm

# Nominal 1S LiPo voltage (actuator.cpp V_BATT_NOMINAL) -- fallback ONLY when
# the CSV has no `vbat` column (older udp_capture.py, or no PKT_STATUS
# packets received) or vbat is implausible. A short flight's real cell
# voltage stays close enough to this for the fit to remain useful; it is
# reported (duty_reason) whenever used so the caller knows the accuracy
# caveat applies.
# 公称 1S LiPo 電圧（actuator.cpp の V_BATT_NOMINAL）—— CSV に `vbat` 列が
# 無い（旧 udp_capture.py、または PKT_STATUS 未受信）か値が非現実的な場合の
# みフォールバックする。短時間フライトなら実際のセル電圧はこの値に十分
# 近く、フィットは実用的なまま。使用時は必ず duty_reason で伝え、精度低下の
# 可能性を呼び出し側に示す。
_V_BATT_NOMINAL: float = 3.7
_V_BATT_MIN: float = 2.5   # below this, a logged vbat is treated as invalid (actuator.cpp V_BATT_MIN)


def _thrust_from_duty(duty: np.ndarray, vbat: np.ndarray) -> np.ndarray:
    """
    Invert actuator.cpp's thrustToDuty(): recover per-motor thrust [N] from
    an OBSERVED duty and the battery voltage at that instant.
    actuator.cpp の thrustToDuty() を逆算し、観測された duty とその瞬間の
    バッテリ電圧から各モータ推力 [N] を復元する。

    Forward model: omega = sqrt(T/Ct); volts = Am*omega^2 + Bm*omega + Cm;
    duty = volts/vbat (thrust<=0 -> duty=MIN_DUTY=0). Inverting: given
    volts = duty*vbat, solve the quadratic Am*omega^2 + Bm*omega +
    (Cm-volts) = 0 for the positive root, then T = Ct*omega^2. Uses the
    OBSERVED (possibly clamped/saturated) duty, so saturation is correctly
    reflected in the recovered thrust -- same design intent as the "legacy"
    inversion's T-cancellation comment above (no assumption the actuator
    never saturated).
    順方向モデル: omega=sqrt(T/Ct); volts=Am*omega^2+Bm*omega+Cm;
    duty=volts/vbat（thrust<=0 なら duty=MIN_DUTY=0）。逆算: volts=duty*vbat
    として、2次方程式 Am*omega^2+Bm*omega+(Cm-volts)=0 を正の根について解き、
    T=Ct*omega^2 とする。観測された（飽和していれば飽和済みの）duty を
    そのまま使うため、飽和も正しく復元推力に反映される（上の "legacy"
    逆算の T相殺コメントと同じ設計意図: アクチュエータが飽和しなかったと
    仮定しない）。

    Args:
        duty: observed motor duty array [0, 1]
        vbat: battery voltage array [V], same length as duty

    Returns:
        Per-motor thrust array [N] (same length), clamped to >= 0.
    """
    volts = duty * vbat
    # volts <= Cm -> omega=0 (below the curve's zero-speed intercept, same as
    # the forward function's thrust<=0 -> MIN_DUTY=0 boundary).
    # volts <= Cm なら omega=0（曲線のゼロ速度切片以下 -- 順方向関数の
    # thrust<=0 -> MIN_DUTY=0 という境界条件と対応）。
    discriminant = np.maximum(_MOTOR_BM ** 2 - 4.0 * _MOTOR_AM * (_MOTOR_CM - volts), 0.0)
    omega = np.maximum((-_MOTOR_BM + np.sqrt(discriminant)) / (2.0 * _MOTOR_AM), 0.0)
    return _MOTOR_CT * omega ** 2


def _duty_differential_vehicle(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    vbat: np.ndarray, axis: str,
) -> np.ndarray:
    """
    Invert firmware/vehicle's ACTUAL mixer (B^-1 allocation + nonlinear motor
    curve, sf_actuator/actuator.cpp) to recover the per-axis differential
    TORQUE command [Nm] that produced the 4 motor duties. --mixer vehicle.
    firmware/vehicle の実際のミキサー（B^-1配分＋非線形モータ曲線、
    sf_actuator/actuator.cpp）を逆算し、4モータduty から軸別の差動トルク
    指令 [Nm] を復元する。--mixer vehicle。

    Unlike _duty_differential_legacy_linear(), the per-motor thrust must be
    recovered FIRST (_thrust_from_duty(), which needs vbat -- the motor curve
    is nonlinear and voltage-dependent) before the B^-1 combination below,
    which is algebraically the SAME sign pattern as the legacy linear mixer
    (same derivation technique -- T/thrust cancels identically) but with
    ARM_D (roll/pitch) and KAPPA (yaw) as the per-channel scale instead of a
    single shared k:
      roll  = ARM_D * (-T_fr - T_rr + T_rl + T_fl)
      pitch = ARM_D * ( T_fr - T_rr - T_rl + T_fl)
      yaw   = KAPPA * ( T_fr - T_rr + T_rl - T_fl)
    _duty_differential_legacy_linear() と異なり、下の B^-1 結合の前に、まず
    各モータ推力を復元する必要がある（_thrust_from_duty()。モータ曲線が
    非線形・電圧依存のため vbat が要る）。結合式自体は legacy 線形ミキサー
    と代数的に同じ符号パターン（同じ導出技法 -- T/thrust が同様に相殺する）
    だが、単一の共有係数 k の代わりにチャンネルごとの ARM_D（roll/pitch）と
    KAPPA（yaw）を使う。

    Args:
        duty_fr, duty_rr, duty_rl, duty_fl: motor duty arrays [0, 1]
        vbat: battery voltage array [V], same length as the duty arrays
        axis: 'roll', 'pitch', or 'yaw'

    Returns:
        Differential torque command array [Nm] (same length as the inputs).
    """
    t_fr = _thrust_from_duty(duty_fr, vbat)
    t_rr = _thrust_from_duty(duty_rr, vbat)
    t_rl = _thrust_from_duty(duty_rl, vbat)
    t_fl = _thrust_from_duty(duty_fl, vbat)

    if axis == 'roll':
        return _ARM_D * (-t_fr - t_rr + t_rl + t_fl)
    elif axis == 'pitch':
        return _ARM_D * (t_fr - t_rr - t_rl + t_fl)
    elif axis == 'yaw':
        return _KAPPA * (t_fr - t_rr + t_rl - t_fl)
    else:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")


# Data Stream CSV column recording which rate motor_duty_* actually came from
# (400 = real kPktDuty400 entry, 50 = CtrlRef forward-fill) -- see
# udp_capture.py save_stream_csv(). Appended AFTER the pre-existing 28
# columns, so its absence just means an older udp_capture.py wrote the file.
# motor_duty_* が実際どちらのレートで来たかを記録する Data Stream CSV の列
# （400=本物の kPktDuty400 エントリ、50=CtrlRef前方補完）— udp_capture.py の
# save_stream_csv() 参照。既存28列の後ろに追記するため、無ければ単に旧い
# udp_capture.py が書いた CSV というだけ。
_DUTY_RATE_HZ_COL = 'duty_rate_hz'

# Data Stream CSV column carrying the 1Hz PKT_STATUS battery voltage,
# forward-filled onto every 400Hz row -- see udp_capture.py save_stream_csv().
# Appended AFTER duty_rate_hz (so, like it, absent in CSVs from an older
# udp_capture.py or a log with no PKT_STATUS packets received). ONLY needed
# for --mixer vehicle's _thrust_from_duty(); --mixer legacy never reads it.
# 1Hz の PKT_STATUS バッテリ電圧を全400Hz行へ前方補完した Data Stream CSV
# 列 -- udp_capture.py の save_stream_csv() 参照。duty_rate_hz の後ろに追記
# するため、それと同様、旧い udp_capture.py が書いた CSV や PKT_STATUS を
# 一度も受信していないログには無い。--mixer vehicle の
# _thrust_from_duty() だけが必要とする列 -- --mixer legacy は読まない。
_VBAT_COL = 'vbat'

# Heuristic threshold for CSVs WITHOUT the duty_rate_hz column: if more than
# this fraction of rows repeat the previous row's 4 duties exactly, the data
# looks like a 50Hz-forward-filled staircase (a genuine 50Hz CtrlRef entry
# repeats for ~8 consecutive 400Hz rows, ~87.5% duplicates) rather than real
# 400Hz duty (which essentially never repeats bit-for-bit).
# duty_rate_hz 列が無い CSV 向けのヒューリスティック閾値: 直前行と4 duty が
# 完全一致する行の割合がこれを超えたら、50Hz前方補完の階段状データ（本物の
# 50Hz CtrlRef エントリは400Hz中約8行連続で同一値、重複率約87.5%）とみなす
# （本物の400Hz duty はビット単位で繰り返すことがほぼ無い）。
_DUTY_STAIRSTEP_FRACTION_THRESHOLD = 0.5


def _classify_duty_source(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    duty_rate_hz_col: Optional[np.ndarray],
) -> Tuple[str, str]:
    """
    Classify whether the CSV's motor_duty_* columns are genuine 400Hz duty
    (safe input for the 'duty' fit mode) or a 50Hz-forward-filled staircase
    (would silently identify off a stale/quantized signal -- must NOT be
    auto-selected).
    CSV の motor_duty_* 列が本物の400Hz duty（'duty'入力モードで安全）か、
    50Hz前方補完の階段状データ（黙って使うと古い/粗い信号で誤同定する --
    自動選択してはならない）かを判別する。

    Returns:
        (quality, reason). quality is 'duty400' (safe) or 'duty50' (unsafe --
        callers should fall back to the 'kp' mode). `reason` is a
        human-readable explanation, always non-empty, for display/errors.
    """
    if duty_rate_hz_col is not None and len(duty_rate_hz_col) > 0:
        hz = float(np.median(duty_rate_hz_col))
        if hz >= 200.0:   # nominal 400, generous margin above the 50Hz case
            return 'duty400', f"duty_rate_hz column says {hz:.0f} Hz (0x4A entry present)"
        return 'duty50', f"duty_rate_hz column says {hz:.0f} Hz (50Hz CtrlRef forward-fill)"

    # No duty_rate_hz column (CSV from an older udp_capture.py) -- fall back
    # to the consecutive-duplicate-row heuristic.
    # duty_rate_hz 列が無い（旧 udp_capture.py の CSV）-- 連続重複行の
    # ヒューリスティックにフォールバック。
    if len(duty_fr) < 2:
        return 'duty400', "too few rows for the stairstep heuristic -- assuming 400Hz"

    same = ((duty_fr[1:] == duty_fr[:-1]) & (duty_rr[1:] == duty_rr[:-1])
            & (duty_rl[1:] == duty_rl[:-1]) & (duty_fl[1:] == duty_fl[:-1]))
    dup_fraction = float(np.mean(same))
    threshold_pct = _DUTY_STAIRSTEP_FRACTION_THRESHOLD * 100.0
    if dup_fraction > _DUTY_STAIRSTEP_FRACTION_THRESHOLD:
        return ('duty50',
                f"no duty_rate_hz column; {dup_fraction * 100:.0f}% of rows repeat "
                f"the previous row's duty exactly (> {threshold_pct:.0f}% threshold) "
                "-- looks like a 50Hz-forward-filled staircase")
    return ('duty400',
            f"no duty_rate_hz column; only {dup_fraction * 100:.0f}% of rows repeat "
            f"the previous row's duty (<= {threshold_pct:.0f}% threshold) -- looks "
            "like genuine high-rate duty")


@dataclass
class PlantFitResult:
    """Plant model identification result / プラントモデル同定結果

    Model: G_p(s) = K / (s * (tau_m * s + 1))
    """
    K: float              # Plant gain [rad/s^2 per duty]
    tau_m: float          # Motor time constant [s]
    K_std: float          # K uncertainty (std dev across segments)
    tau_m_std: float      # tau_m uncertainty (std dev across segments)
    r_squared: float      # Fit quality (mean R^2 across segments)
    rmse: float           # RMSE [rad/s] (mean across segments)
    axis: str             # 'roll', 'pitch', or 'yaw'
    kp_used: Optional[float]  # Kp used for plant input reconstruction ('kp' mode only)
    n_segments: int       # Number of segments used for fitting
    input_mode: str = 'kp'    # 'duty' (mixer-inverse of motor_duty_*) or 'kp'
                               # (legacy Kp*(target-gyro) reconstruction)
    mixer: str = 'legacy'     # 'legacy' or 'vehicle' -- which duty inversion
                               # produced K (see module docstring --mixer).
                               # Meaningless when input_mode == 'kp'.
    duty_quality: Optional[str] = None  # 'duty400'/'duty50'/None -- see
                                         # _classify_duty_source()
    duty_reason: str = ''     # human-readable reason for duty_quality, always
                               # shown so the input-mode choice is explained

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        defaults = get_flat_defaults()
        # K's reference/units depend on which mixer produced it (duty-diff
        # gain for 'legacy', torque gain 1/I_axis for 'vehicle') -- see
        # REFERENCE_PLANT_GAINS vs REFERENCE_PLANT_GAINS_VEHICLE above.
        # K の参照値・単位は、どちらのミキサーが生成したかで異なる
        # （'legacy' は duty差動ゲイン、'vehicle' はトルクゲイン 1/I_axis）
        # -- 上の REFERENCE_PLANT_GAINS / REFERENCE_PLANT_GAINS_VEHICLE 参照。
        ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if self.mixer == 'vehicle' else REFERENCE_PLANT_GAINS
        ref_K = ref_gains.get(self.axis, 0.0)
        ref_tau_m = defaults['tau_m']

        result: Dict[str, Any] = {
            'method': 'plant_fit',
            'timestamp': datetime.now().isoformat(),
            'model': 'K / (s * (tau_m * s + 1))',
            'axis': self.axis,
            'input_mode': self.input_mode,
            'mixer': self.mixer,
            'duty_quality': self.duty_quality,
            'duty_reason': self.duty_reason,
            'kp_used': self.kp_used,
            'n_segments': self.n_segments,
            'estimated': {
                'K': round(self.K, 2),
                'K_uncertainty': round(self.K_std, 2),
                'tau_m': round(self.tau_m, 4),
                'tau_m_uncertainty': round(self.tau_m_std, 4),
            },
            'fit_quality': {
                'r_squared': round(self.r_squared, 4),
                'rmse': round(self.rmse, 4),
            },
            'reference': {
                'K': ref_K,
                'tau_m': ref_tau_m,
            },
            'comparison': {},
        }

        # Comparison with reference values
        # 参照値との比較
        if ref_K > 0:
            K_err = abs(self.K - ref_K) / ref_K * 100
            result['comparison']['K'] = {
                'error_percent': round(K_err, 1),
                'status': 'OK' if K_err < 30 else 'CHECK',
            }
        if ref_tau_m > 0:
            tau_err = abs(self.tau_m - ref_tau_m) / ref_tau_m * 100
            result['comparison']['tau_m'] = {
                'error_percent': round(tau_err, 1),
                'status': 'OK' if tau_err < 30 else 'CHECK',
            }

        # Derived: design Kp for zeta=0.7
        # Closed-loop: Kp = 1 / (4 * zeta^2 * K * tau_m)
        zeta = 0.7
        if self.K > 0 and self.tau_m > 0:
            Kp_design = 1.0 / (4.0 * zeta**2 * self.K * self.tau_m)
            result['derived'] = {
                'design_Kp_zeta_0_7': round(Kp_design, 4),
            }
            if ref_K > 0:
                Kp_ref = 1.0 / (4.0 * zeta**2 * ref_K * ref_tau_m)
                result['derived']['reference_Kp_zeta_0_7'] = round(Kp_ref, 4)

        return result


def _simulate_plant(
    K: float,
    tau_m: float,
    u: np.ndarray,
    dt: float,
    omega0: float,
    z0: float = 0.0,
) -> np.ndarray:
    """
    Simulate plant G_p(s) = K / (s * (tau_m * s + 1))
    プラントをシミュレート

    State space representation:
        x1_dot = x2               (omega_dot = z)
        x2_dot = -x2/tau_m + K*u/tau_m  (motor first-order dynamics)
        y      = x1               (output = angular velocity)

    Uses exact discretization for motor dynamics (x2)
    and trapezoidal integration for the integrator (x1).

    Args:
        K: Plant gain [rad/s^2 per duty]
        tau_m: Motor time constant [s]
        u: Plant input array (duty)
        dt: Sample period [s]
        omega0: Initial angular velocity [rad/s]
        z0: Initial motor-filtered acceleration [rad/s^2]

    Returns:
        Simulated angular velocity [rad/s]
    """
    n = len(u)
    alpha = np.exp(-dt / tau_m)
    gain = K * (1.0 - alpha)

    # Motor-filtered acceleration (x2 state)
    # 1次遅れ（モータ動特性）の厳密離散化
    z = np.empty(n)
    z[0] = z0
    for i in range(1, n):
        z[i] = alpha * z[i - 1] + gain * u[i - 1]

    # Integrate to angular velocity (trapezoidal rule)
    # 台形積分で角速度を計算
    omega = np.empty(n)
    omega[0] = omega0
    omega[1:] = omega0 + np.cumsum((z[:-1] + z[1:]) * 0.5 * dt)

    return omega


def _fit_segment(
    u_seg: np.ndarray,
    y_seg: np.ndarray,
    dt: float,
    K_init: float = 100.0,
    tau_m_init: float = 0.02,
    K_bounds: Tuple[float, float] = (1.0, 1000.0),
) -> Optional[Tuple[float, float, float, float]]:
    """
    Fit K and tau_m for a single data segment
    単一セグメントの K, tau_m をフィット

    Args:
        u_seg: Plant input for segment
        y_seg: Measured output (gyro) for segment
        dt: Sample period [s]
        K_init: Initial guess for K
        tau_m_init: Initial guess for tau_m
        K_bounds: (min, max) bounds for K in the optimizer. Default is the
            "legacy" duty-differential-gain scale ([rad/s^2 / duty],
            REFERENCE_PLANT_GAINS ~ 20-100). --mixer vehicle's K is a
            torque gain [rad/s^2 / Nm] several orders of magnitude larger
            (REFERENCE_PLANT_GAINS_VEHICLE ~ 1e5) -- callers in that mode
            MUST pass a matching K_bounds, or the true optimum sits outside
            this default and the fit degenerates to the bound.
            K の最適化境界（既定値）。"legacy" の duty差動ゲイン単位
            （[rad/s^2 / duty]、REFERENCE_PLANT_GAINS ~ 20-100）向け。
            --mixer vehicle の K はトルクゲイン単位 [rad/s^2 / Nm] で
            桁が何桁も大きい（REFERENCE_PLANT_GAINS_VEHICLE ~ 1e5）—— この
            モードの呼び出し側は対応する K_bounds を渡すこと。渡さないと
            真の最適値が既定境界の外にあり、フィットが境界に張り付いて
            縮退する。

    Returns:
        (K, tau_m, r_squared, rmse) or None if fitting fails
    """
    if len(u_seg) < 20:
        return None

    omega0 = y_seg[0]
    # Estimate initial angular acceleration from first few samples
    # 最初の数サンプルから初期角加速度を推定
    n_init = min(10, len(y_seg) - 1)
    z0 = float(y_seg[n_init] - y_seg[0]) / (n_init * dt)

    def objective(params):
        K = params[0]
        tau_m = np.exp(params[1])  # log transform ensures tau_m > 0
        y_sim = _simulate_plant(K, tau_m, u_seg, dt, omega0, z0)
        return np.mean((y_sim - y_seg) ** 2)

    try:
        result = minimize(
            objective,
            x0=[K_init, np.log(tau_m_init)],
            method='L-BFGS-B',
            bounds=[K_bounds, (np.log(0.003), np.log(0.5))],
            options={'maxiter': 200},
        )
    except Exception:
        return None

    if not result.success and result.fun > 1.0:
        return None

    K_opt = result.x[0]
    tau_m_opt = np.exp(result.x[1])

    # Compute fit quality metrics
    # フィット品質の計算
    y_sim = _simulate_plant(K_opt, tau_m_opt, u_seg, dt, omega0, z0)
    residuals = y_seg - y_sim
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y_seg - np.mean(y_seg)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    return K_opt, tau_m_opt, r_squared, rmse


def _detect_csv_format(fieldnames: Optional[List[str]]) -> str:
    """
    Auto-detect which of the two CSV schemas a flight log uses, from its
    header alone.
    CSV のヘッダだけから、フライトログが2種類のうちどちらの形式かを自動判別。

    Returns:
        "stream" if rate_ref_<axis> + gyro_x/y/z columns are present (current
            400Hz Data Stream -- `sf log wifi -o *.csv`, shared by vehicle and
            workshop).
        "legacy" if ctrl_<axis> columns are present (pre-migration analysis
            CSV -- normalized stick + gyro_corrected_x/y/z, falling back to
            gyro_x/y/z).

    Raises:
        ValueError: neither schema's required columns are present.
    """
    cols = set(fieldnames or [])

    has_stream_target = any(c in cols for c in _STREAM_TARGET_COL.values())
    has_stream_gyro = all(c in cols for c in _STREAM_GYRO_COL.values())
    if has_stream_target and has_stream_gyro:
        return "stream"

    has_legacy_ctrl = any(c in cols for c in _LEGACY_CTRL_COL.values())
    if has_legacy_ctrl:
        return "legacy"

    raise ValueError(
        "CSV format not recognized -- need either the current Data Stream "
        "columns (rate_ref_roll/pitch/yaw + gyro_x/y/z, from `sf log wifi "
        "-o *.csv`) or the legacy analysis columns (ctrl_roll/pitch/yaw + "
        f"gyro_corrected_x/y/z or gyro_x/y/z). Header columns found: "
        f"{sorted(cols)}"
    )


def _load_axis_data(
    filepath: str | Path,
    axis: str,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
    mixer: str = 'legacy',
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, str,
           Optional[np.ndarray], Optional[str], str]:
    """
    Load and extract axis-specific plant I/O from a flight-log CSV.
    CSV から軸固有のプラント入出力を読み込み・抽出する。

    Self-contained CSV reader (csv.DictReader + numpy only) that auto-detects
    the "stream" vs "legacy" schema via _detect_csv_format() -- see the module
    docstring for both schemas' column names and semantics.
    自己完結の CSV リーダ（csv.DictReader + numpy のみ）。
    _detect_csv_format() で "stream"/"legacy" を自動判別する — 両形式の列名・
    意味はモジュール docstring 参照。

    Args:
        mixer: 'legacy' (default) or 'vehicle' -- selects which duty
            inversion produces duty_diff (see the module docstring's --mixer
            section). Only affects the "stream"-format duty_diff
            reconstruction; ignored for "legacy"-format CSVs (no motor_duty_*
            columns there).

    Returns:
        (time_s, target_raw, gyro, throttle, dt, fmt, duty_diff, duty_quality,
        duty_reason)
        target_raw is the RAW target column: already rad/s for "stream"
        (caller must NOT multiply by rate_max), normalized stick [-1, 1] for
        "legacy" (caller multiplies by rate_max). `fmt` tells the caller which.
        duty_diff is the mixer-inverted per-axis differential duty/torque
        (see _duty_differential_legacy_linear() /
        _duty_differential_vehicle(), selected by `mixer`) when
        motor_duty_FR/RR/RL/FL columns are present in the CSV, else None.
        duty_quality/duty_reason classify that duty as genuine 400Hz data
        ('duty400') or a 50Hz-forward-filled staircase ('duty50') -- see
        _classify_duty_source(); duty_quality is None (with an explanatory
        duty_reason) when duty_diff is None. For mixer='vehicle' without a
        `vbat` column, duty_reason additionally notes the V_BATT_NOMINAL
        fallback.
        target_raw は列の生値: "stream" は既に rad/s（呼び出し側は rate_max を
        掛けてはいけない）、"legacy" は正規化スティック値 [-1, 1]（呼び出し側が
        rate_max を掛ける）。どちらかは `fmt` で判別する。
        duty_diff は motor_duty_FR/RR/RL/FL 列が CSV にあればミキサ逆算した
        軸別差動duty/トルク（_duty_differential_legacy_linear() /
        _duty_differential_vehicle()。`mixer` で選択）、無ければ None。
        duty_quality/duty_reason はその duty が本物の400Hzデータ
        （'duty400'）か50Hz前方補完の階段状データ（'duty50'）かを判別する
        （_classify_duty_source() 参照）。duty_diff が None のときは
        duty_quality も None（duty_reason に理由）。mixer='vehicle' で
        `vbat` 列が無い場合、duty_reason に V_BATT_NOMINAL フォールバックの
        旨も追記する。
    """
    import csv as _csv

    if axis not in _AXIS_NAMES:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")
    if mixer not in ('legacy', 'vehicle'):
        raise ValueError(f"Unknown mixer: {mixer!r}. Choose from: legacy, vehicle")

    with open(filepath, newline='') as f:
        reader = _csv.DictReader(f)
        fieldnames = reader.fieldnames
        fmt = _detect_csv_format(fieldnames)
        rows = list(reader)

    if len(rows) < 100:
        raise ValueError(f"Too few samples: {len(rows)}")

    cols = set(fieldnames or [])

    def col(row: Dict[str, str], name: str, default: float = 0.0) -> float:
        value = row.get(name)
        if value is None or value == '':
            return default
        try:
            return float(value)
        except ValueError:
            return default

    # --- Timestamp -> time_s, refining fs from the data when it looks sane ---
    # タイムスタンプ -> time_s。データから妥当な範囲ならサンプルレートを補正。
    ts_col = next((c for c in ('timestamp_us', 'timestamp', 'timestamp_ms') if c in cols), None)
    if ts_col is not None:
        scale_to_us = 1000.0 if ts_col == 'timestamp_ms' else 1.0
        ts_us = np.array([col(r, ts_col) for r in rows]) * scale_to_us
        time_s = (ts_us - ts_us[0]) / 1e6
        deltas = np.diff(time_s)
        deltas = deltas[deltas > 0]
        if len(deltas) > 10:
            detected_fs = 1.0 / float(np.median(deltas))
            if 0.5 * fs <= detected_fs <= 2.0 * fs:   # reject obviously-wrong units
                fs = detected_fs
    else:
        time_s = np.arange(len(rows)) / fs
    dt = 1.0 / fs

    # --- Target + gyro, per detected schema ---
    # 目標値 + ジャイロ（判別した形式に応じて）
    if fmt == "stream":
        target = np.array([col(r, _STREAM_TARGET_COL[axis]) for r in rows])
        gyro = np.array([col(r, _STREAM_GYRO_COL[axis]) for r in rows])
    else:
        target = np.array([col(r, _LEGACY_CTRL_COL[axis]) for r in rows])
        gyro_col = _LEGACY_GYRO_COL[axis] if _LEGACY_GYRO_COL[axis] in cols else _STREAM_GYRO_COL[axis]
        gyro = np.array([col(r, gyro_col) for r in rows])

    # --- Throttle-equivalent for flight-segment detection ---
    # 飛行区間検出用のスロットル相当量
    if fmt == "legacy":
        throttle = np.array([col(r, 'ctrl_throttle') for r in rows])
    elif 'total_thrust' in cols:
        throttle = np.array([col(r, 'total_thrust') for r in rows])
    elif all(f'motor_duty_{m}' in cols for m in ('FR', 'RR', 'RL', 'FL')):
        throttle = np.mean(
            [[col(r, f'motor_duty_{m}') for m in ('FR', 'RR', 'RL', 'FL')] for r in rows],
            axis=1,
        )
    else:
        raise ValueError(
            "stream-format CSV needs a flight-activity column to find flight "
            "segments: total_thrust (ws::motor_mixer thrust) or "
            "motor_duty_FR/RR/RL/FL. Neither was found in the header."
        )

    # --- Motor duty -> mixer-inverted differential duty (the "duty" input
    # mode's u_plant) when the 4 motor_duty_* columns are present, plus a
    # classification of whether that duty is genuine 400Hz data or a 50Hz
    # forward-filled staircase (see _classify_duty_source()) -- the 'auto'
    # input mode must NOT identify off the latter.
    # モータduty -> ミキサ逆算した差動duty（"duty" 入力モードの u_plant）。
    # motor_duty_* の4列が揃っていれば計算し、あわせてそのduty が本物の400Hz
    # データか50Hz前方補完の階段状データかを判別する（_classify_duty_source()
    # 参照）-- 'auto' 入力モードは後者で同定してはならない。
    duty_diff: Optional[np.ndarray] = None
    duty_quality: Optional[str] = None
    duty_reason = "no motor_duty_FR/RR/RL/FL columns in CSV"
    if all(c in cols for c in _DUTY_COLS):
        duty_fr = np.array([col(r, 'motor_duty_FR') for r in rows])
        duty_rr = np.array([col(r, 'motor_duty_RR') for r in rows])
        duty_rl = np.array([col(r, 'motor_duty_RL') for r in rows])
        duty_fl = np.array([col(r, 'motor_duty_FL') for r in rows])

        if mixer == 'vehicle':
            # vbat is forward-filled from the 1Hz PKT_STATUS entry (see
            # udp_capture.py save_stream_csv()); fall back to the nominal 1S
            # LiPo voltage when absent (older CSV / no PKT_STATUS received)
            # or implausible, and say so in duty_reason.
            # vbat は1Hz PKT_STATUSエントリからの前方補完（udp_capture.py の
            # save_stream_csv() 参照）。無い場合（旧CSV/PKT_STATUS未受信）や
            # 非現実的な値の場合は公称1S LiPo電圧にフォールバックし、
            # duty_reason にその旨を記す。
            vbat_note = ''
            if _VBAT_COL in cols:
                vbat = np.array([col(r, _VBAT_COL, default=0.0) for r in rows])
                bad = vbat < _V_BATT_MIN
                if np.any(bad):
                    vbat = np.where(bad, _V_BATT_NOMINAL, vbat)
                    vbat_note = (
                        f" ({int(np.sum(bad))}/{len(vbat)} rows had no/implausible "
                        f"vbat -- used the nominal {_V_BATT_NOMINAL}V there)"
                    )
            else:
                vbat = np.full(len(rows), _V_BATT_NOMINAL)
                vbat_note = (
                    f" (no vbat column in CSV -- used the nominal "
                    f"{_V_BATT_NOMINAL}V throughout; re-capture with a current "
                    "udp_capture.py for the real battery-sag-corrected fit)"
                )
            duty_diff = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat, axis)
        else:
            duty_diff = _duty_differential_legacy_linear(duty_fr, duty_rr, duty_rl, duty_fl, axis)
            vbat_note = ''

        duty_rate_hz_col = (
            np.array([col(r, _DUTY_RATE_HZ_COL) for r in rows])
            if _DUTY_RATE_HZ_COL in cols else None
        )
        duty_quality, duty_reason = _classify_duty_source(
            duty_fr, duty_rr, duty_rl, duty_fl, duty_rate_hz_col,
        )
        duty_reason += vbat_note

    # Apply time range filter
    # 時間範囲フィルタを適用
    if time_range is not None:
        t_start, t_end = time_range
        mask = (time_s >= t_start) & (time_s <= t_end)
        time_s = time_s[mask]
        target = target[mask]
        gyro = gyro[mask]
        throttle = throttle[mask]
        if duty_diff is not None:
            duty_diff = duty_diff[mask]

    return time_s, target, gyro, throttle, dt, fmt, duty_diff, duty_quality, duty_reason


def _find_flight_segments(
    throttle: np.ndarray,
    seg_samples: int,
    throttle_threshold: float = 0.3,
) -> List[Tuple[int, int]]:
    """
    Find flight segments where throttle > threshold
    スロットルが閾値以上の飛行区間を検出

    Returns:
        List of (start_idx, end_idx) tuples for analysis segments
    """
    in_flight = throttle > throttle_threshold

    # Find contiguous flight regions
    # 連続的な飛行区間を検出
    flight_starts = []
    flight_ends = []
    in_region = False
    for i in range(len(in_flight)):
        if in_flight[i] and not in_region:
            flight_starts.append(i)
            in_region = True
        elif not in_flight[i] and in_region:
            flight_ends.append(i)
            in_region = False
    if in_region:
        flight_ends.append(len(in_flight))

    # Split flight regions into analysis segments
    # 飛行区間を分析セグメントに分割
    min_seg = seg_samples // 2
    segments = []
    for start, end in zip(flight_starts, flight_ends):
        if end - start < min_seg:
            continue
        for seg_start in range(start, end - min_seg, seg_samples):
            seg_end = min(seg_start + seg_samples, end)
            if seg_end - seg_start >= min_seg:
                segments.append((seg_start, seg_end))

    return segments


def fit_plant(
    filepath: str | Path,
    axis: str = 'roll',
    kp: Optional[float] = None,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
    segment_length: float = 3.0,
    min_activity: float = 0.01,
    input_mode: str = 'auto',
    mixer: str = 'legacy',
) -> PlantFitResult:
    """
    Fit open-loop plant model from closed-loop flight data
    閉ループフライトデータから開ループプラントモデルを同定

    Args:
        filepath: Path to CSV flight log
        axis: 'roll', 'pitch', or 'yaw'
        kp: P gain used during flight (must match firmware value). Required
            when input_mode resolves to 'kp'; ignored (may be left None) in
            'duty' mode.
        rate_max: Maximum angular rate [rad/s] (maps ctrl [-1,+1] to rate).
            Only used by the 'kp' mode's "legacy" CSV path.
        fs: Sample rate [Hz] (default: 400)
        time_range: Optional (start, end) in seconds to restrict analysis
        segment_length: Segment duration [s] for fitting (default: 3.0)
        min_activity: Minimum std of plant input to include segment
        input_mode: 'auto' (default), 'duty', or 'kp' -- see the module
            docstring. 'auto' picks 'duty' only when the CSV has
            motor_duty_FR/RR/RL/FL columns, kp is None, AND
            _classify_duty_source() says the duty is genuine 400Hz data
            (not a 50Hz-forward-filled staircase); otherwise 'kp'.
        mixer: 'legacy' (default) or 'vehicle' -- which duty inversion the
            'duty'/'auto' input modes use (see the module docstring's
            --mixer section). 'legacy' is correct for firmware/workshop
            (`sf lesson`) and firmware/vehicle_old logs; 'vehicle' is
            required for firmware/vehicle (`sf app`) logs. The CSV cannot
            say which firmware produced it -- the caller must know.
            Ignored when input_mode resolves to 'kp'.

    Returns:
        PlantFitResult with identified K, tau_m and fit metrics

    Raises:
        ValueError: If data is insufficient, fitting fails, or the resolved
            input mode's required data/argument is missing.
    """
    if mixer not in ('legacy', 'vehicle'):
        raise ValueError(f"Unknown mixer: {mixer!r}. Choose from: legacy, vehicle")

    # Load data
    # データ読み込み
    time_s, target_raw, gyro, throttle, dt, fmt, duty_diff, duty_quality, duty_reason = (
        _load_axis_data(filepath, axis, fs, time_range, mixer=mixer)
    )

    # Resolve the input mode -- see the module docstring for the 3 modes.
    # AUTO must not silently pick 'duty' on a 50Hz-forward-filled staircase
    # (duty_quality == 'duty50'): that would fit a stale/quantized signal
    # and look like it worked (good R^2, wrong physics). Falls back to 'kp'
    # instead, which then requires --kp.
    # 入力モードを解決する — 3モードの詳細はモジュール docstring 参照。
    # AUTO は50Hz前方補完の階段状データ（duty_quality=='duty50'）で黙って
    # 'duty' を選んではならない — 古い/粗い信号でフィットしてしまい、
    # 見かけ上は動作したように見える（R^2は良いが物理的に誤り）。代わりに
    # 'kp' へフォールバックし、--kp を要求する。
    if input_mode not in ('auto', 'duty', 'kp'):
        raise ValueError(f"Unknown input_mode: {input_mode!r}. Choose from: auto, duty, kp")
    resolved_mode = input_mode
    if resolved_mode == 'auto':
        if duty_diff is not None and kp is None and duty_quality == 'duty400':
            resolved_mode = 'duty'
        else:
            resolved_mode = 'kp'

    if resolved_mode == 'duty':
        if duty_diff is None:
            raise ValueError(
                "--input duty requested but this CSV has no "
                "motor_duty_FR/RR/RL/FL columns -- needs firmware sending "
                "the 400Hz duty entry (kPktDuty400/0x4A), or the 50Hz "
                "CtrlRef forward-fill (see udp_capture.py "
                "save_stream_csv()). Pass --kp to use the legacy "
                "Kp*(target-gyro) reconstruction instead."
            )
        if duty_quality != 'duty400':
            raise ValueError(
                f"--input duty requested but this log is from OLD firmware "
                f"(motor_duty_* is 50Hz-forward-filled, not real 400Hz "
                f"data): {duty_reason}. Pass --kp instead (--input kp) -- "
                "duty here is too coarse to identify a ~20ms motor lag."
            )
        # u_plant(t) = mixer-inverse(motor_duty_FR/RR/RL/FL)(t) -- the actual
        # differential command the rate-loop PID output this cycle, using
        # WHICHEVER mixer inversion `mixer` selected (legacy duty-diff vs
        # vehicle torque -- see the module docstring's --mixer section). If
        # the fit below fails or gives a physically-implausible K/tau_m,
        # the most likely cause is the WRONG --mixer for this log's firmware.
        # u_plant(t) = ミキサ逆算(motor_duty_FR/RR/RL/FL)(t) -- レートループ
        # PID がその周期に実際に出力した差動指令。`mixer` が選んだ方の逆算
        # （legacy の duty差動 vs vehicle のトルク -- モジュール docstring の
        # --mixer 節参照）を使う。下のフィットが失敗する、または物理的に
        # ありえない K/tau_m になる場合、最も疑わしい原因はこのログの
        # ファームに対して --mixer が間違っていること。
        u_plant = duty_diff
        kp_used: Optional[float] = None
    else:  # 'kp'
        if kp is None:
            if duty_quality == 'duty50':
                raise ValueError(
                    f"this log is from OLD firmware: {duty_reason}. "
                    "motor_duty_* is only 50Hz-resolution here (not the "
                    "real 400Hz kPktDuty400 entry), so --kp is required "
                    "for a reliable fit (--input kp)."
                )
            raise ValueError(
                "--input kp (explicit, or auto without motor_duty_* "
                "columns) requires --kp -- the P gain that flew. Capture a "
                "log with `sf log wifi` on firmware sending the 400Hz duty "
                "entry to use --input duty instead (no --kp needed)."
            )
        # Reconstruct plant I/O. "stream" CSVs already record the physical
        # rate target [rad/s] (ws::set_rate_target / vehicle rate_ref) so
        # rate_max is NOT applied; "legacy" CSVs store a normalized stick
        # value that must be scaled by rate_max first.
        # プラント入出力を復元。"stream" 形式は既に物理量の角速度目標 [rad/s]
        # （ws::set_rate_target / vehicle の rate_ref）を記録しているため
        # rate_max は適用しない。"legacy" 形式は正規化スティック値のため
        # rate_max でスケールする。
        #   u_plant(t) = Kp * (target(t) - gyro(t))
        target = target_raw if fmt == "stream" else target_raw * rate_max
        u_plant = kp * (target - gyro)
        kp_used = kp

    y_plant = gyro

    # Find flight segments
    # 飛行区間の検出
    seg_samples = int(segment_length * (1.0 / dt))
    segments = _find_flight_segments(throttle, seg_samples)

    if not segments:
        raise ValueError(
            "No valid flight segments found. "
            "Check that throttle > 0.3 during flight."
        )

    # K's scale/units -- and everything calibrated against it (the optimizer
    # bounds below, AND the min_activity floor just below) -- depend on which
    # mixer produced u_plant when resolved_mode == 'duty' (torque-input K is
    # ~1000x the duty-differential-input K -- see REFERENCE_PLANT_GAINS_VEHICLE's
    # docstring above); 'kp' mode always uses the duty-differential scale
    # (u_plant there is directly comparable to the legacy mixer's u_plant by
    # construction).
    # K の尺度・単位 -- それに較正された最適化境界（下）と min_activity 閾値
    # （すぐ下）も -- は、resolved_mode=='duty' のときどちらのミキサーが
    # u_plant を作ったかに依存する（トルク入力の K は duty差動入力の K より
    # およそ3桁大きい -- 上の REFERENCE_PLANT_GAINS_VEHICLE のdocstring
    # 参照）。'kp' モードは常に duty差動スケール（そちらの u_plant は構成上
    # legacy ミキサーの u_plant と直接比較可能）。
    use_vehicle_scale = resolved_mode == 'duty' and mixer == 'vehicle'
    ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if use_vehicle_scale else REFERENCE_PLANT_GAINS
    ref_K = ref_gains.get(axis, 100.0)
    # Vehicle-scale K sits ~1e5 (1/I_axis); bound wide around it rather than
    # the legacy (1.0, 1000.0) default, which would clip the true optimum.
    # ヴィークル尺度の K はおよそ1e5（1/I_axis）に位置する。legacy の既定
    # (1.0, 1000.0) では真の最適値を境界で切り詰めてしまうため、その周囲を
    # 広く取る。
    K_bounds = (1.0, 1.0e9) if use_vehicle_scale else (1.0, 1000.0)
    # min_activity is a floor on std(u_plant); the caller's default (0.01) is
    # calibrated for the legacy duty-differential scale. Roughly rescale it
    # for the vehicle torque scale by the same K ratio used for K_bounds
    # above (equal plant RESPONSE needs roughly K_legacy/K_vehicle times less
    # torque than duty-differential input) -- this only needs to reject
    # near-silent segments, not be exact.
    # min_activity は std(u_plant) の下限。呼び出し側の既定値（0.01）は
    # legacy の duty差動スケール用に較正されている。上の K_bounds と同じ K比
    # でヴィークルのトルクスケール向けに概算し直す（同じプラント応答を得る
    # のに必要なトルクは duty差動入力よりおおよそ K_legacy/K_vehicle 倍小さい
    # ）-- ほぼ無音のセグメントを排除できれば十分で、厳密さは不要。
    min_activity_effective = min_activity
    if use_vehicle_scale:
        legacy_ref_K = REFERENCE_PLANT_GAINS.get(axis, 100.0)
        min_activity_effective = min_activity * (legacy_ref_K / ref_K)

    # Filter segments by control activity level
    # 制御入力が十分な区間のみ抽出
    active_segments = []
    for seg_start, seg_end in segments:
        if np.std(u_plant[seg_start:seg_end]) > min_activity_effective:
            active_segments.append((seg_start, seg_end))

    if not active_segments:
        raise ValueError(
            f"No segments with sufficient control activity "
            f"(std > {min_activity_effective:.3g}). "
            "Ensure the pilot made stick inputs during flight."
        )

    # Fit each segment
    # 各セグメントをフィット
    K_estimates: List[float] = []
    tau_m_estimates: List[float] = []
    r2_values: List[float] = []
    rmse_values: List[float] = []

    for seg_start, seg_end in active_segments:
        u_seg = u_plant[seg_start:seg_end]
        y_seg = y_plant[seg_start:seg_end]

        fit = _fit_segment(u_seg, y_seg, dt, K_init=ref_K, tau_m_init=0.02, K_bounds=K_bounds)
        if fit is not None:
            K, tau_m, r2, rmse = fit
            # Filter out unreasonable fits
            # 不合理なフィット結果を除外
            if r2 > 0.3 and K > 1.0 and 0.003 < tau_m < 0.5:
                K_estimates.append(K)
                tau_m_estimates.append(tau_m)
                r2_values.append(r2)
                rmse_values.append(rmse)

    if not K_estimates:
        raise ValueError(
            "Fitting failed for all segments. Check data quality and Kp "
            "value -- if this is a --input duty fit, also check --mixer: "
            "'legacy' is for firmware/workshop (`sf lesson`) and "
            "firmware/vehicle_old logs, 'vehicle' is for firmware/vehicle "
            "(`sf app`) logs. Using the wrong one reconstructs a "
            "physically-wrong u_plant and degrades exactly like this."
        )

    # Aggregate results using median (robust to outliers)
    # 中央値で集約（外れ値に頑健）
    K_final = float(np.median(K_estimates))
    tau_m_final = float(np.median(tau_m_estimates))
    K_std = float(np.std(K_estimates)) if len(K_estimates) > 1 else 0.0
    tau_m_std = float(np.std(tau_m_estimates)) if len(tau_m_estimates) > 1 else 0.0
    r2_mean = float(np.mean(r2_values))
    rmse_mean = float(np.mean(rmse_values))

    return PlantFitResult(
        K=K_final,
        tau_m=tau_m_final,
        K_std=K_std,
        tau_m_std=tau_m_std,
        r_squared=r2_mean,
        rmse=rmse_mean,
        axis=axis,
        kp_used=kp_used,
        n_segments=len(K_estimates),
        input_mode=resolved_mode,
        mixer=mixer if resolved_mode == 'duty' else 'legacy',
        duty_quality=duty_quality,
        duty_reason=duty_reason,
    )


def compute_fit_timeseries(
    filepath: str | Path,
    result: PlantFitResult,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
) -> Dict[str, np.ndarray]:
    """
    Compute time series data for plotting fit results
    フィット結果のプロット用時系列データを計算

    Args:
        filepath: Path to CSV flight log (same file used for fitting)
        result: PlantFitResult from fit_plant()
        rate_max: Maximum angular rate [rad/s]
        fs: Sample rate [Hz]
        time_range: Optional (start, end) in seconds

    Returns:
        Dictionary with keys:
            'time': Time array [s]
            'u_plant': Reconstructed plant input
            'y_measured': Measured angular velocity (gyro)
            'y_simulated': Simulated angular velocity
            'residual': y_measured - y_simulated
    """
    time_s, target_raw, gyro, throttle, dt, fmt, duty_diff, _duty_quality, _duty_reason = (
        _load_axis_data(filepath, result.axis, fs, time_range, mixer=result.mixer)
    )

    # Reconstruct plant I/O -- same input-mode logic as fit_plant(), using
    # whichever mode the fit ACTUALLY used (result.input_mode), not rate_max
    # or --kp implied by the caller.
    # プラント入出力を復元 -- fit_plant() と同じ入力モードのロジック。
    # フィットが実際に使ったモード（result.input_mode）に従う（呼び出し側の
    # rate_max/--kp に引きずられない）。
    if result.input_mode == 'duty':
        if duty_diff is None:
            raise ValueError(
                "fit used the 'duty' input mode but this CSV has no "
                "motor_duty_FR/RR/RL/FL columns"
            )
        u_plant = duty_diff
    else:
        target = target_raw if fmt == "stream" else target_raw * rate_max
        u_plant = result.kp_used * (target - gyro)
    y_measured = gyro

    # Simulate full time series with identified parameters
    # 同定パラメータで全時系列をシミュレート
    omega0 = y_measured[0]
    n_init = min(10, len(y_measured) - 1)
    z0 = float(y_measured[n_init] - y_measured[0]) / (n_init * dt)

    y_simulated = _simulate_plant(
        result.K, result.tau_m, u_plant, dt, omega0, z0,
    )

    return {
        'time': time_s,
        'u_plant': u_plant,
        'y_measured': y_measured,
        'y_simulated': y_simulated,
        'residual': y_measured - y_simulated,
    }


# =============================================================================
# Self-test: synthesize a "stream"-format flight from a KNOWN plant, recover
# it. Run via `sf sysid fit --selftest`.
# 自己テスト: 既知プラントから "stream" 形式のフライトを合成し、復元を検証。
# `sf sysid fit --selftest` から実行。
# =============================================================================

def selftest(verbose: bool = True) -> bool:
    """
    Closed-loop self-test for the "stream" (current Data Stream) CSV path.
    既知プラントを P 制御閉ループで離散シミュレーションし、`sf log wifi
    -o *.csv` と同じ "stream" 形式の CSV を合成、fit_plant() が K, tau_m を
    許容誤差内で復元することを確認する。

    Verifies end to end: _detect_csv_format() picks "stream", _load_axis_data()
    reads rate_ref_roll/gyro_x/total_thrust correctly, and the (format-
    independent) MSE fit in _fit_segment() recovers the plant that generated
    the data. Uses the EXACT same discretization as _simulate_plant() to
    generate the synthetic flight, so any recovery error reflects the fit
    tool's own accuracy, not a model mismatch.
    一気通貫の検証: _detect_csv_format() が "stream" を選び、_load_axis_data()
    が rate_ref_roll/gyro_x/total_thrust を正しく読み、（形式非依存の）
    _fit_segment() の MSE フィットが生成元のプラントを復元できることを確認
    する。合成フライトの生成には _simulate_plant() と全く同じ離散化を使うため、
    復元誤差はフィットツール自体の精度を反映し、モデル不整合には起因しない。
    """
    import csv as _csv
    import os
    import tempfile

    axis = 'roll'
    K_true = REFERENCE_PLANT_GAINS[axis]   # 102.0 [rad/s^2 per duty]
    tau_m_true = 0.02                      # [s] -- L06 nominal motor lag
    kp = 0.5
    fs = 400.0
    dt = 1.0 / fs
    n = 8000                               # 20 s @ 400 Hz

    alpha = np.exp(-dt / tau_m_true)
    gain = K_true * (1.0 - alpha)

    t = np.arange(n) * dt
    # Broadband log-chirp target (0.5->20 Hz over 5 s, repeated): similar
    # spectral richness to a pilot's stick doublets, wide enough to resolve
    # both the integrator gain K and the ~8 Hz motor-lag corner (tau_m=20ms).
    # 広帯域対数チャープ目標（0.5〜20Hz、5s周期で繰り返し）: パイロットの
    # スティックダブレットに近いスペクトルで、積分ゲイン K とモータ遅れの
    # コーナー周波数（tau_m=20ms、約8Hz）の両方を解ける帯域幅を持つ。
    f0, f1, period = 0.5, 20.0, 5.0
    k_chirp = (f1 / f0) ** (1.0 / period)
    tm = t % period
    target = 0.35 * np.sin(2 * np.pi * f0 * ((k_chirp ** tm) - 1.0) / np.log(k_chirp))

    # Closed-loop P-control simulation with the EXACT discretization
    # _simulate_plant() uses (motor-lag alpha filter + trapezoidal
    # integration) -- z[i]/omega[i] depend only on u[i-1], so this is causal
    # and needs no algebraic-loop solving.
    # _simulate_plant() と全く同じ離散化（モータ遅れの指数フィルタ + 台形
    # 積分）による閉ループ P 制御シミュレーション -- z[i]/omega[i] は
    # u[i-1] のみに依存するため、代数ループを解く必要のない因果的な計算。
    omega = np.zeros(n)
    z = np.zeros(n)
    u = np.zeros(n)
    for i in range(1, n):
        error = target[i - 1] - omega[i - 1]
        u[i - 1] = kp * error
        z[i] = alpha * z[i - 1] + gain * u[i - 1]
        omega[i] = omega[i - 1] + 0.5 * dt * (z[i - 1] + z[i])
    u[-1] = kp * (target[-1] - omega[-1])

    rng = np.random.default_rng(7)
    gyro_meas = omega + rng.normal(0.0, 0.003, n)   # [rad/s] BMI270-scale noise

    # Synthesize the 4 motor duties from `u` (the roll-axis PID output above)
    # through the FORWARD X-quad mixer (ws_internal.hpp motor_mixer, P=Y=0
    # for a roll-only excitation): FR=RR=T-k*R, RL=FL=T+k*R. This is the
    # exact inverse-of-_duty_differential() construction, so the "duty"
    # input mode below must recover u (and hence K, tau_m) as well as the
    # "kp" mode does.
    # `u`（上のロール軸PID出力）から4モータduty を「順」ミキサ
    # （ws_internal.hpp motor_mixer、ロール単独励振なので P=Y=0）で合成する:
    # FR=RR=T-k*R, RL=FL=T+k*R。これは _duty_differential() の厳密な逆構成
    # なので、下の "duty" 入力モードは "kp" モードと同等に u（ひいては
    # K, tau_m）を復元できるはずである。
    T_hover = 0.4
    duty_fr = T_hover - _MIXER_K * u
    duty_rr = T_hover - _MIXER_K * u
    duty_rl = T_hover + _MIXER_K * u
    duty_fl = T_hover + _MIXER_K * u

    # Write a "stream"-format Data Stream CSV (same header shape as `sf log
    # wifi -o *.csv`): only the test axis carries nonzero rate_ref/gyro,
    # total_thrust is a constant in-flight value (> the 0.3 flight threshold),
    # and motor_duty_FR/RR/RL/FL carry the synthesized duty above (the "duty"
    # input mode's data source).
    # "stream" 形式の Data Stream CSV を書き出す（`sf log wifi -o *.csv` と
    # 同じヘッダ形状）: テスト対象軸のみ rate_ref/gyro を非ゼロにし、
    # total_thrust は飛行中を示す定数値（飛行判定閾値 0.3 を超える）、
    # motor_duty_FR/RR/RL/FL は上で合成した duty（"duty" 入力モードのデータ源）。
    fieldnames = (['timestamp_us']
                  + list(_STREAM_GYRO_COL.values())
                  + list(_STREAM_TARGET_COL.values())
                  + ['total_thrust']
                  + list(_DUTY_COLS))
    gyro_col = _STREAM_GYRO_COL[axis]
    target_col = _STREAM_TARGET_COL[axis]

    fd, csv_path = tempfile.mkstemp(suffix='.csv', prefix='plant_fit_selftest_')
    try:
        with os.fdopen(fd, 'w', newline='') as f:
            writer = _csv.writer(f)
            writer.writerow(fieldnames)
            for i in range(n):
                row = {name: 0.0 for name in fieldnames}
                row['timestamp_us'] = t[i] * 1e6
                row[gyro_col] = gyro_meas[i]
                row[target_col] = target[i]
                row['total_thrust'] = 0.4
                row['motor_duty_FR'] = duty_fr[i]
                row['motor_duty_RR'] = duty_rr[i]
                row['motor_duty_RL'] = duty_rl[i]
                row['motor_duty_FL'] = duty_fl[i]
                writer.writerow([row[name] for name in fieldnames])

        result_kp = fit_plant(csv_path, axis=axis, kp=kp, rate_max=1.0, fs=fs,
                               input_mode='kp')
        result_duty = fit_plant(csv_path, axis=axis, rate_max=1.0, fs=fs,
                                 input_mode='duty')
        # 'auto' on this genuine (continuously-varying) 400Hz duty CSV must
        # ALSO pick 'duty' -- proves _classify_duty_source()'s heuristic has
        # no false positive on real data (no duty_rate_hz column here either,
        # so this exercises the heuristic branch, not the authoritative one).
        # この本物の（連続的に変化する）400Hz duty CSV では 'auto' も 'duty'
        # を選ぶこと -- _classify_duty_source() のヒューリスティックが実データで
        # 偽陽性を出さないことの証明（ここも duty_rate_hz 列は無く、権威的
        # 判定ではなくヒューリスティック分岐を検証する）。
        result_auto = fit_plant(csv_path, axis=axis, rate_max=1.0, fs=fs,
                                 input_mode='auto')
    finally:
        os.unlink(csv_path)

    # --- Additional case: a 50Hz-forward-filled STAIRCASE duty (each value
    # held for 8 consecutive 400Hz rows, no duty_rate_hz column -- exercises
    # the heuristic in _classify_duty_source()). 'auto' must NOT silently
    # fit off this stale/quantized signal: it must fall back to 'kp' and
    # require --kp.
    # 追加ケース: 50Hz前方補完の階段状duty（各値を400Hz中8行連続保持、
    # duty_rate_hz列なし -- _classify_duty_source() のヒューリスティックを
    # 検証）。'auto' はこの古い/粗い信号で黙ってフィットしてはならない --
    # 'kp' へフォールバックし --kp を要求すること。
    u_stair = u.copy()
    for start in range(0, n, 8):
        u_stair[start:start + 8] = u[start]
    duty_fr_stair = T_hover - _MIXER_K * u_stair
    duty_rr_stair = T_hover - _MIXER_K * u_stair
    duty_rl_stair = T_hover + _MIXER_K * u_stair
    duty_fl_stair = T_hover + _MIXER_K * u_stair

    fd2, csv_path2 = tempfile.mkstemp(suffix='.csv', prefix='plant_fit_selftest_stair_')
    try:
        with os.fdopen(fd2, 'w', newline='') as f:
            writer = _csv.writer(f)
            writer.writerow(fieldnames)
            for i in range(n):
                row = {name: 0.0 for name in fieldnames}
                row['timestamp_us'] = t[i] * 1e6
                row[gyro_col] = gyro_meas[i]
                row[target_col] = target[i]
                row['total_thrust'] = 0.4
                row['motor_duty_FR'] = duty_fr_stair[i]
                row['motor_duty_RR'] = duty_rr_stair[i]
                row['motor_duty_RL'] = duty_rl_stair[i]
                row['motor_duty_FL'] = duty_fl_stair[i]
                writer.writerow([row[name] for name in fieldnames])

        try:
            fit_plant(csv_path2, axis=axis, kp=None, rate_max=1.0, fs=fs, input_mode='auto')
            stair_rejected_without_kp = False
            stair_reject_msg = "(did not raise)"
        except ValueError as e:
            stair_reject_msg = str(e)
            stair_rejected_without_kp = 'old firmware' in stair_reject_msg.lower()

        result_stair_kp = fit_plant(csv_path2, axis=axis, kp=kp, rate_max=1.0, fs=fs,
                                     input_mode='auto')
    finally:
        os.unlink(csv_path2)

    stair_resolved_kp = (result_stair_kp.input_mode == 'kp'
                          and result_stair_kp.duty_quality == 'duty50')
    ok_stair = stair_rejected_without_kp and stair_resolved_kp
    if verbose:
        print(f"[stair] auto w/o --kp rejected ({stair_reject_msg[:70]}...): "
              f"{stair_rejected_without_kp}")
        print(f"[stair] auto w/ --kp resolves to 'kp' (duty_quality="
              f"{result_stair_kp.duty_quality}): {stair_resolved_kp}")

    def _check(result, label):
        K_err = abs(result.K / K_true - 1.0)
        tau_err = abs(result.tau_m / tau_m_true - 1.0)
        passed = K_err < 0.15 and tau_err < 0.30 and result.r_squared > 0.9
        if verbose:
            print(f"[{label}] fit: K={result.K:.1f} ({K_err * 100:.1f}% err)  "
                  f"tau_m={result.tau_m * 1000:.1f} ms ({tau_err * 100:.1f}% err)  "
                  f"R^2={result.r_squared:.3f}  n_segments={result.n_segments}  "
                  f"input_mode={result.input_mode}")
        return passed

    if verbose:
        print(f"true : K={K_true:.1f} [rad/s^2/duty]  tau_m={tau_m_true * 1000:.1f} ms")
    ok_kp = _check(result_kp, 'kp')
    ok_duty = _check(result_duty, 'duty')
    ok_auto = _check(result_auto, 'auto') and result_auto.input_mode == 'duty'

    # --- --mixer vehicle regression: same closed-loop synthesis, but the
    # roll-axis PID output `u_v` is now a physical differential TORQUE [Nm]
    # (not a duty-differential command), converted to 4 motor duties through
    # the FORWARD physical chain (B^-1 allocation -> thrustToDuty(), the same
    # _ARM_D/_MOTOR_AM/_MOTOR_BM/_MOTOR_CM/_MOTOR_CT constants
    # _duty_differential_vehicle()/_thrust_from_duty() invert) instead of the
    # linear ws_internal mixer used by `u`/duty_fr above. Proves the vehicle
    # inversion recovers its own forward model correctly (sign, scale, AND
    # vbat -- vbat_true is deliberately off V_BATT_NOMINAL so a bug that
    # silently used the nominal fallback instead of the CSV's vbat column
    # would show up as a scale error here) -- the same sanity the "duty"
    # case above gets from the ws_internal forward mixer.
    # --mixer vehicle 回帰: 同じ閉ループ合成だが、ロール軸PID出力 `u_v` は
    # duty差動指令ではなく物理的な差動トルク[Nm]で、線形の ws_internal
    # ミキサーではなく順方向の物理チェーン（B^-1配分 -> thrustToDuty()。
    # _duty_differential_vehicle()/_thrust_from_duty() が逆算するのと同じ
    # _ARM_D/_MOTOR_AM/_MOTOR_BM/_MOTOR_CM/_MOTOR_CT 定数を使う）で4モータ
    # duty に変換する。vehicle 逆算が自身の順方向モデルを正しく逆算できる
    # こと（符号・スケール・**vbat** -- vbat_true は意図的に
    # V_BATT_NOMINAL からずらしてあるので、CSV の vbat 列を使わず黙って
    # ノミナルへフォールバックするバグがあればここでスケール誤差として
    # 現れる）を証明する -- 上の "duty" ケースが順方向 ws_internal ミキサー
    # で得ているのと同じ健全性チェック。
    K_true_vehicle = REFERENCE_PLANT_GAINS_VEHICLE[axis]   # ~1/Ixx [rad/s^2/Nm]
    kp_vehicle = 1.0e-3    # [Nm/(rad/s)] -- same order as firmware rate.roll.kp
    vbat_true = 3.85       # [V] -- deliberately != V_BATT_NOMINAL (3.7)
    thrust_hover_total = 0.4   # [N] -- arbitrary in-flight value (> the 0.3
                               # flight-segment threshold, matching T_hover
                               # above), only needs to keep all 4 per-motor
                               # thrusts positive across the excitation
                               # amplitude

    alpha_v = np.exp(-dt / tau_m_true)
    gain_v = K_true_vehicle * (1.0 - alpha_v)

    omega_v = np.zeros(n)
    z_v = np.zeros(n)
    u_v = np.zeros(n)   # roll differential torque [Nm]
    for i in range(1, n):
        error = target[i - 1] - omega_v[i - 1]
        u_v[i - 1] = kp_vehicle * error
        z_v[i] = alpha_v * z_v[i - 1] + gain_v * u_v[i - 1]
        omega_v[i] = omega_v[i - 1] + 0.5 * dt * (z_v[i - 1] + z_v[i])
    u_v[-1] = kp_vehicle * (target[-1] - omega_v[-1])

    gyro_meas_v = omega_v + rng.normal(0.0, 0.003, n)

    # FORWARD B^-1 allocation (roll-only excitation: pitch=yaw=0, matching
    # actuator.cpp mixerCompute()) then the FORWARD motor curve (matching
    # actuator.cpp thrustToDuty()) -- the exact inverse of what
    # _duty_differential_vehicle()/_thrust_from_duty() compute.
    t_fr_v = 0.25 * (thrust_hover_total - u_v / _ARM_D)
    t_rr_v = 0.25 * (thrust_hover_total - u_v / _ARM_D)
    t_rl_v = 0.25 * (thrust_hover_total + u_v / _ARM_D)
    t_fl_v = 0.25 * (thrust_hover_total + u_v / _ARM_D)

    def _thrust_to_duty(t):
        omega_m = np.sqrt(np.maximum(t, 0.0) / _MOTOR_CT)
        volts = _MOTOR_AM * omega_m ** 2 + _MOTOR_BM * omega_m + _MOTOR_CM
        return volts / vbat_true

    duty_fr_v = _thrust_to_duty(t_fr_v)
    duty_rr_v = _thrust_to_duty(t_rr_v)
    duty_rl_v = _thrust_to_duty(t_rl_v)
    duty_fl_v = _thrust_to_duty(t_fl_v)

    fieldnames_v = fieldnames + ['vbat']
    fd3, csv_path3 = tempfile.mkstemp(suffix='.csv', prefix='plant_fit_selftest_vehicle_')
    try:
        with os.fdopen(fd3, 'w', newline='') as f:
            writer = _csv.writer(f)
            writer.writerow(fieldnames_v)
            for i in range(n):
                row = {name: 0.0 for name in fieldnames_v}
                row['timestamp_us'] = t[i] * 1e6
                row[gyro_col] = gyro_meas_v[i]
                row[target_col] = target[i]
                row['total_thrust'] = thrust_hover_total
                row['motor_duty_FR'] = duty_fr_v[i]
                row['motor_duty_RR'] = duty_rr_v[i]
                row['motor_duty_RL'] = duty_rl_v[i]
                row['motor_duty_FL'] = duty_fl_v[i]
                row['vbat'] = vbat_true
                writer.writerow([row[name] for name in fieldnames_v])

        result_vehicle = fit_plant(csv_path3, axis=axis, rate_max=1.0, fs=fs,
                                    input_mode='duty', mixer='vehicle')
    finally:
        os.unlink(csv_path3)

    K_err_v = abs(result_vehicle.K / K_true_vehicle - 1.0)
    tau_err_v = abs(result_vehicle.tau_m / tau_m_true - 1.0)
    ok_vehicle = K_err_v < 0.15 and tau_err_v < 0.30 and result_vehicle.r_squared > 0.9
    if verbose:
        print(f"[vehicle] true: K={K_true_vehicle:.1f} [rad/s^2/Nm]  "
              f"tau_m={tau_m_true * 1000:.1f} ms")
        print(f"[vehicle] fit : K={result_vehicle.K:.1f} ({K_err_v * 100:.1f}% err)  "
              f"tau_m={result_vehicle.tau_m * 1000:.1f} ms ({tau_err_v * 100:.1f}% err)  "
              f"R^2={result_vehicle.r_squared:.3f}  n_segments={result_vehicle.n_segments}  "
              f"mixer={result_vehicle.mixer}")

    ok = ok_kp and ok_duty and ok_auto and ok_stair and ok_vehicle

    if verbose:
        print("SELFTEST:", "PASS" if ok else "FAIL")

    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)
