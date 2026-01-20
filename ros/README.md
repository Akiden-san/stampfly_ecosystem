# StampFly ROS2 Bridge

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

StampFly の WebSocket テレメトリを ROS2 トピックにブリッジするパッケージ群。

## 1. 概要

### パッケージ構成

| パッケージ | 説明 |
|-----------|------|
| `stampfly_msgs` | カスタムメッセージ定義 |
| `stampfly_bridge` | WebSocket → ROS2 ブリッジノード |

### 主な機能

- 400Hz テレメトリストリーミング
- NED → ENU 座標変換
- TF2 ブロードキャスト
- 自動再接続

## 2. インストール

### 依存関係

```bash
# ROS2 Humble 以降
sudo apt install ros-humble-tf2-ros

# Python依存
pip install websockets
```

### ビルド

```bash
cd ros
source /opt/ros/humble/setup.bash
colcon build --packages-select stampfly_msgs stampfly_bridge
source install/setup.bash
```

## 3. 使用方法

### 基本起動

```bash
# StampFly WiFi に接続後
ros2 launch stampfly_bridge bridge.launch.py

# カスタムIP指定
ros2 launch stampfly_bridge bridge.launch.py host:=192.168.4.1
```

### RViz2 可視化

```bash
ros2 launch stampfly_bridge rviz.launch.py
```

### トピック確認

```bash
ros2 topic list | grep stampfly
ros2 topic hz /stampfly/imu  # ~400Hz expected
ros2 topic echo /stampfly/pose
```

## 4. トピック一覧

### カスタムメッセージ

| トピック | 型 | レート | 説明 |
|---------|-----|--------|------|
| `/stampfly/imu/raw` | stampfly_msgs/ImuRaw | 400Hz | 生IMUデータ |
| `/stampfly/imu/corrected` | stampfly_msgs/ImuCorrected | 400Hz | バイアス補正IMU |
| `/stampfly/eskf/state` | stampfly_msgs/ESKFState | 400Hz | ESKF推定値 |
| `/stampfly/control/input` | stampfly_msgs/ControlInput | 400Hz | 制御入力 |
| `/stampfly/range/sensors` | stampfly_msgs/RangeSensors | 400Hz | 距離センサ |
| `/stampfly/flow` | stampfly_msgs/OpticalFlow | 400Hz | 光学フロー |

### 標準ROS2メッセージ

| トピック | 型 | レート | 説明 |
|---------|-----|--------|------|
| `/stampfly/pose` | geometry_msgs/PoseStamped | 400Hz | 位置・姿勢 |
| `/stampfly/velocity` | geometry_msgs/TwistStamped | 400Hz | 速度 |
| `/stampfly/imu` | sensor_msgs/Imu | 400Hz | 標準IMU |
| `/stampfly/range/bottom` | sensor_msgs/Range | 400Hz | 底面ToF |
| `/stampfly/range/front` | sensor_msgs/Range | 400Hz | 前方ToF |

### TF フレーム

```
odom → base_link (400Hz)
```

### 制御トピック（Phase 2）

| トピック | 型 | 方向 | 説明 |
|---------|-----|------|------|
| `/stampfly/cmd_vel` | geometry_msgs/Twist | Subscribe | 速度コマンド |

### サービス

| サービス | 型 | 説明 |
|---------|-----|------|
| `/stampfly/arm` | std_srvs/SetBool | ARM/DISARM |

## 5. パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `host` | string | "192.168.4.1" | StampFly IPアドレス |
| `port` | int | 80 | WebSocketポート |
| `path` | string | "/ws" | WebSocketパス |
| `auto_reconnect` | bool | true | 自動再接続 |
| `publish_tf` | bool | true | TFブロードキャスト |
| `odom_frame` | string | "odom" | オドメトリフレームID |
| `base_frame` | string | "base_link" | ベースフレームID |
| `imu_frame` | string | "imu_link" | IMUフレームID |
| `enable_control` | bool | false | 制御機能有効化 |
| `control_rate` | double | 50.0 | 制御送信レート (Hz) |
| `max_throttle` | double | 0.8 | 最大スロットル（安全制限） |

## 6. 座標系

### StampFly（NED）

- X: North（前方）
- Y: East（右）
- Z: Down（下）

### ROS2（ENU）

- X: East（右）
- Y: North（前方）
- Z: Up（上）

変換: `(x_enu, y_enu, z_enu) = (y_ned, x_ned, -z_ned)`

## 7. トラブルシューティング

### 接続できない

1. StampFly WiFi AP に接続しているか確認
2. IP アドレスを確認: `ping 192.168.4.1`
3. WebSocket エンドポイント確認: `curl -v ws://192.168.4.1/ws`

### レートが低い

1. WiFi 信号強度を確認
2. 他の WiFi クライアントを切断
3. 距離を近づける

## 8. 制御機能（Phase 2）

### 制御モードの起動

```bash
ros2 launch stampfly_bridge bridge.launch.py enable_control:=true
```

### ARM/DISARM

```bash
# ARM（モーター有効化）
ros2 service call /stampfly/arm std_srvs/srv/SetBool "{data: true}"

# DISARM（モーター無効化）
ros2 service call /stampfly/arm std_srvs/srv/SetBool "{data: false}"
```

### 速度コマンド送信

```bash
# Twistメッセージで制御
# linear.z: スロットル (0.0-1.0)
# angular.x: ロールレート (-1.0 to 1.0)
# angular.y: ピッチレート (-1.0 to 1.0)
# angular.z: ヨーレート (-1.0 to 1.0)

ros2 topic pub /stampfly/cmd_vel geometry_msgs/Twist "{linear: {z: 0.3}, angular: {z: 0.1}}"
```

### 安全上の注意

- `enable_control` はデフォルトで `false`（明示的に有効化が必要）
- `max_throttle` で最大スロットルを制限（デフォルト 0.8）
- シャットダウン時は自動でDISARM
- 通信途絶時は StampFly 側で 500ms 後に自動停止

---

<a id="english"></a>

# StampFly ROS2 Bridge (English)

ROS2 bridge packages for StampFly WebSocket telemetry.

## 1. Overview

### Packages

| Package | Description |
|---------|-------------|
| `stampfly_msgs` | Custom message definitions |
| `stampfly_bridge` | WebSocket → ROS2 bridge node |

### Features

- 400Hz telemetry streaming
- NED → ENU coordinate transformation
- TF2 broadcasting
- Auto-reconnection

## 2. Installation

### Dependencies

```bash
# ROS2 Humble or later
sudo apt install ros-humble-tf2-ros

# Python dependencies
pip install websockets
```

### Build

```bash
cd ros
source /opt/ros/humble/setup.bash
colcon build --packages-select stampfly_msgs stampfly_bridge
source install/setup.bash
```

## 3. Usage

### Basic Launch

```bash
# After connecting to StampFly WiFi
ros2 launch stampfly_bridge bridge.launch.py

# Custom IP
ros2 launch stampfly_bridge bridge.launch.py host:=192.168.4.1
```

### RViz2 Visualization

```bash
ros2 launch stampfly_bridge rviz.launch.py
```

### Topic Verification

```bash
ros2 topic list | grep stampfly
ros2 topic hz /stampfly/imu  # ~400Hz expected
ros2 topic echo /stampfly/pose
```

## 4. Topics

See Japanese section above for complete topic list.

## 5. Coordinate Frames

### StampFly (NED)

- X: North (forward)
- Y: East (right)
- Z: Down

### ROS2 (ENU)

- X: East (right)
- Y: North (forward)
- Z: Up

Transform: `(x_enu, y_enu, z_enu) = (y_ned, x_ned, -z_ned)`
