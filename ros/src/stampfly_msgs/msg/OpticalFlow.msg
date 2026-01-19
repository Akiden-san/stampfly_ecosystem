# Optical flow sensor data
# 光学フローセンサデータ

std_msgs/Header header

# Raw flow values [sensor units]
# 生のフロー値（センサ単位）
int16 flow_x
int16 flow_y

# Quality/confidence [0-255]
# 品質/信頼度
uint8 quality
