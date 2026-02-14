"""
ESKF Educational Wrapper
ESKF 教育用ラッパー

Provides simplified interface to the full ESKF implementation
with sensor enable/disable controls for pedagogical experiments.
"""

import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Import the real ESKF implementation
# 実際の ESKF 実装をインポート
_eskf_dir = Path(__file__).parent.parent.parent.parent / "tools" / "eskf_sim"
if str(_eskf_dir) not in sys.path:
    sys.path.insert(0, str(_eskf_dir))

from eskf import ESKF, ESKFConfig


class ESKFEducational:
    """Educational wrapper for ESKF with sensor controls.

    センサ制御付き ESKF の教育用ラッパー。

    Allows students to:
    学生ができること:
    - Enable/disable individual sensors to see their effect
    - Adjust Q and R noise parameters
    - Compare gyro-only integration vs full ESKF
    - Visualize filter state and covariance

    Args:
        config: ESKFConfig override / ESKFConfig の上書き
        enable_baro: Enable barometer updates / 気圧計更新を有効化
        enable_tof: Enable ToF updates / ToF 更新を有効化
        enable_mag: Enable magnetometer updates / 磁気計更新を有効化
        enable_flow: Enable optical flow updates / オプティカルフロー更新を有効化
        enable_accel_att: Enable accel attitude correction / 加速度姿勢補正を有効化

    Usage:
        >>> eskf = ESKFEducational(enable_baro=True, enable_tof=True)
        >>> eskf.sensor_status()
        {'baro': True, 'tof': True, 'mag': False, 'flow': False, 'accel_att': True}
    """

    def __init__(
        self,
        config: Optional[ESKFConfig] = None,
        enable_baro: bool = True,
        enable_tof: bool = True,
        enable_mag: bool = False,
        enable_flow: bool = False,
        enable_accel_att: bool = True,
    ):
        self.config = config or ESKFConfig()
        self.eskf = ESKF(self.config)

        # Sensor enable flags
        # センサ有効フラグ
        self._sensors = {
            "baro": enable_baro,
            "tof": enable_tof,
            "mag": enable_mag,
            "flow": enable_flow,
            "accel_att": enable_accel_att,
        }

        # History for plotting
        # プロット用の履歴
        self._history = {
            "time": [],
            "pos": [],
            "vel": [],
            "att_euler": [],
            "gyro_bias": [],
            "accel_bias": [],
        }

    def sensor_status(self) -> dict:
        """Get current sensor enable status.

        現在のセンサ有効状態を取得する。
        """
        return dict(self._sensors)

    def enable_sensor(self, name: str, enabled: bool = True):
        """Enable or disable a sensor.

        センサを有効/無効にする。

        Args:
            name: "baro", "tof", "mag", "flow", or "accel_att"
            enabled: True to enable, False to disable
        """
        if name not in self._sensors:
            raise ValueError(f"Unknown sensor: {name}. Available: {list(self._sensors.keys())}")
        self._sensors[name] = enabled

    def predict(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """ESKF prediction step (IMU propagation).

        ESKF 予測ステップ（IMU 伝搬）。

        Args:
            gyro: Gyroscope measurement [gx, gy, gz] (rad/s)
            accel: Accelerometer measurement [ax, ay, az] (m/s²)
            dt: Time step (s)
        """
        self.eskf.predict(gyro, accel, dt)

    def update_baro(self, altitude: float):
        """Update with barometer measurement (if enabled).

        気圧計測定値で更新する（有効時のみ）。
        """
        if self._sensors["baro"]:
            self.eskf.update_baro(altitude)

    def update_tof(self, distance: float):
        """Update with ToF measurement (if enabled).

        ToF 測定値で更新する（有効時のみ）。
        """
        if self._sensors["tof"]:
            self.eskf.update_tof(distance)

    def update_accel_attitude(self, accel: np.ndarray):
        """Update attitude with accelerometer (if enabled).

        加速度計で姿勢を更新する（有効時のみ）。
        """
        if self._sensors["accel_att"]:
            self.eskf.update_accel_attitude(accel)

    def get_state(self) -> dict:
        """Get current filter state as a readable dict.

        現在のフィルタ状態を読みやすい辞書で取得する。
        """
        pos = self.eskf.position.copy()
        vel = self.eskf.velocity.copy()
        q = self.eskf.quaternion.copy()

        # Quaternion to Euler
        # クォータニオンからオイラー角へ
        euler = self._quat_to_euler(q)

        return {
            "position": pos,
            "velocity": vel,
            "attitude_euler_deg": np.degrees(euler),
            "quaternion": q,
            "gyro_bias": self.eskf.gyro_bias.copy(),
            "accel_bias": self.eskf.accel_bias.copy(),
        }

    def record_state(self, t: float):
        """Record current state for later plotting.

        後でプロットするために現在の状態を記録する。
        """
        state = self.get_state()
        self._history["time"].append(t)
        self._history["pos"].append(state["position"].copy())
        self._history["vel"].append(state["velocity"].copy())
        self._history["att_euler"].append(state["attitude_euler_deg"].copy())
        self._history["gyro_bias"].append(state["gyro_bias"].copy())
        self._history["accel_bias"].append(state["accel_bias"].copy())

    def get_history_df(self) -> pd.DataFrame:
        """Get recorded history as a DataFrame.

        記録された履歴を DataFrame で取得する。
        """
        if not self._history["time"]:
            return pd.DataFrame()

        pos = np.array(self._history["pos"])
        vel = np.array(self._history["vel"])
        att = np.array(self._history["att_euler"])

        return pd.DataFrame({
            "time": self._history["time"],
            "x": pos[:, 0], "y": pos[:, 1], "z": pos[:, 2],
            "vx": vel[:, 0], "vy": vel[:, 1], "vz": vel[:, 2],
            "roll": att[:, 0], "pitch": att[:, 1], "yaw": att[:, 2],
        })

    def plot_history(self, title: str = "ESKF State History / ESKF 状態履歴") -> plt.Figure:
        """Plot recorded state history.

        記録された状態履歴をプロットする。
        """
        df = self.get_history_df()
        if df.empty:
            print("No history recorded. Call record_state() during simulation.")
            return None

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

        # Position
        for col, label in zip(["x", "y", "z"], ["X", "Y", "Z"]):
            axes[0].plot(df["time"], df[col], label=label)
        axes[0].set_ylabel("Position (m) / 位置")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Velocity
        for col, label in zip(["vx", "vy", "vz"], ["Vx", "Vy", "Vz"]):
            axes[1].plot(df["time"], df[col], label=label)
        axes[1].set_ylabel("Velocity (m/s) / 速度")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Attitude
        for col, label in zip(["roll", "pitch", "yaw"], ["Roll", "Pitch", "Yaw"]):
            axes[2].plot(df["time"], df[col], label=label)
        axes[2].set_ylabel("Attitude (deg) / 姿勢")
        axes[2].set_xlabel("Time (s) / 時間")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        fig.suptitle(title, fontsize=14)
        fig.tight_layout()
        return fig

    def reset(self):
        """Reset filter and history.

        フィルタと履歴をリセットする。
        """
        self.eskf = ESKF(self.config)
        self._history = {k: [] for k in self._history}

    @staticmethod
    def _quat_to_euler(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw].

        クォータニオン [w, x, y, z] からオイラー角 [roll, pitch, yaw] へ変換。
        """
        w, x, y, z = q

        # Roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)

        # Yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])


def compare_gyro_vs_eskf(
    sensor_data: pd.DataFrame,
    eskf_config: Optional[ESKFConfig] = None,
    dt: float = 0.0025,
) -> plt.Figure:
    """Compare pure gyro integration vs ESKF attitude estimation.

    純粋なジャイロ積分と ESKF 姿勢推定を比較する。

    Args:
        sensor_data: DataFrame with gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
        eskf_config: ESKF configuration
        dt: Time step (s)

    Returns:
        Matplotlib Figure
    """
    config = eskf_config or ESKFConfig()

    # Gyro-only integration
    # ジャイロのみの積分
    roll_gyro = np.zeros(len(sensor_data))
    pitch_gyro = np.zeros(len(sensor_data))
    yaw_gyro = np.zeros(len(sensor_data))

    for i in range(1, len(sensor_data)):
        gx = sensor_data.iloc[i].get("gyro_x", sensor_data.iloc[i].get("p", 0))
        gy = sensor_data.iloc[i].get("gyro_y", sensor_data.iloc[i].get("q", 0))
        gz = sensor_data.iloc[i].get("gyro_z", sensor_data.iloc[i].get("r", 0))

        roll_gyro[i] = roll_gyro[i-1] + gx * dt
        pitch_gyro[i] = pitch_gyro[i-1] + gy * dt
        yaw_gyro[i] = yaw_gyro[i-1] + gz * dt

    # ESKF
    eskf = ESKFEducational(config=config, enable_accel_att=True)
    for i in range(len(sensor_data)):
        row = sensor_data.iloc[i]
        gx = row.get("gyro_x", row.get("p", 0))
        gy = row.get("gyro_y", row.get("q", 0))
        gz = row.get("gyro_z", row.get("r", 0))
        ax = row.get("accel_x", row.get("ax", 0))
        ay = row.get("accel_y", row.get("ay", 0))
        az = row.get("accel_z", row.get("az", -9.81))

        gyro = np.array([gx, gy, gz])
        accel = np.array([ax, ay, az])

        eskf.predict(gyro, accel, dt)
        eskf.update_accel_attitude(accel)
        eskf.record_state(i * dt)

    df_eskf = eskf.get_history_df()
    t = np.arange(len(sensor_data)) * dt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    for ax, gyro_data, eskf_col, title in zip(
        axes,
        [roll_gyro, pitch_gyro, yaw_gyro],
        ["roll", "pitch", "yaw"],
        ["Roll / ロール", "Pitch / ピッチ", "Yaw / ヨー"],
    ):
        ax.plot(t, np.degrees(gyro_data), "r-", alpha=0.7,
                label="Gyro integration / ジャイロ積分")
        if eskf_col in df_eskf.columns:
            ax.plot(df_eskf["time"], df_eskf[eskf_col], "b-", linewidth=1.5,
                    label="ESKF")
        ax.set_ylabel(f"{title} (deg)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s) / 時間")
    fig.suptitle("Gyro Integration vs ESKF / ジャイロ積分 vs ESKF", fontsize=14)
    fig.tight_layout()

    return fig
