# Bias-corrected IMU data (what control loop sees)
# バイアス補正済みIMUデータ（制御ループが見る値）

std_msgs/Header header

# Bias-corrected angular velocity [rad/s]
# バイアス補正済み角速度
geometry_msgs/Vector3 angular_velocity

# Bias-corrected linear acceleration [m/s^2]
# バイアス補正済み加速度
geometry_msgs/Vector3 linear_acceleration
