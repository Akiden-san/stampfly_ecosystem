"""
Allan Variance Analysis of Gyroscope Noise
ジャイロスコープノイズの Allan 分散解析

Session 7: システム同定
"""

import numpy as np
import matplotlib.pyplot as plt
from stampfly_edu import load_sample_data


def allan_variance(data, dt, max_cluster=None):
    """Compute Allan variance for a 1D time series."""
    n = len(data)
    if max_cluster is None:
        max_cluster = n // 4

    m_values = []
    m = 1
    while m <= max_cluster:
        m_values.append(m)
        m = int(m * 1.5) + 1

    tau, adev = [], []
    for m in m_values:
        n_clusters = n // m
        if n_clusters < 2:
            break
        clusters = data[:n_clusters * m].reshape(n_clusters, m).mean(axis=1)
        diff = np.diff(clusters)
        avar = 0.5 * np.mean(diff**2)
        tau.append(m * dt)
        adev.append(np.sqrt(avar))

    return np.array(tau), np.array(adev)


# Load static noise data
# 静止ノイズデータを読み込む
noise = load_sample_data("static_noise")
dt = noise["time"].iloc[1] - noise["time"].iloc[0]

fig, ax = plt.subplots(figsize=(10, 7))

for col, label in [("p", "Gyro X"), ("q", "Gyro Y"), ("r", "Gyro Z")]:
    tau, adev = allan_variance(noise[col].values, dt)
    ax.loglog(tau, adev, linewidth=1.5, label=label)

ax.set_xlabel("Averaging time τ (s)")
ax.set_ylabel("Allan deviation σ(τ) (rad/s)")
ax.set_title("Allan Deviation / Allan 偏差")
ax.legend()
ax.grid(True, alpha=0.3, which="both")

plt.tight_layout()
plt.savefig("allan_variance.png", dpi=150, bbox_inches="tight")
plt.show()
print("Saved: allan_variance.png")
