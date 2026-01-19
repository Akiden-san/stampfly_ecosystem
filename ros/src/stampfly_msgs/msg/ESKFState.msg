# Error State Kalman Filter estimates
# 誤差状態カルマンフィルタ推定値

std_msgs/Header header

# Attitude quaternion [w, x, y, z]
# 姿勢クォータニオン
geometry_msgs/Quaternion orientation

# Position [m] (ENU frame in ROS)
# 位置（ROSではENU座標系）
geometry_msgs/Point position

# Velocity [m/s] (ENU frame in ROS)
# 速度（ROSではENU座標系）
geometry_msgs/Vector3 velocity

# Gyro bias estimate [rad/s]
# ジャイロバイアス推定値
geometry_msgs/Vector3 gyro_bias

# Accel bias estimate [m/s^2]
# 加速度バイアス推定値
geometry_msgs/Vector3 accel_bias

# ESKF status flags
# ESKF状態フラグ
# bit0: initialized/converged
uint8 status
