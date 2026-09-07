# 04_read_imu — IMUのデータを読む

## 目的

SPI通信でBMI270 IMU（慣性計測装置）からデータを取得する方法を学びます。

## 必要な知識

- SPI通信の基本（MOSI, MISO, SCK, CS）
- 加速度とジャイロスコープの意味

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | BMI270 6軸IMU (SPI2) |

| 信号 | GPIO |
|------|------|
| MOSI | 14 |
| MISO | 43 |
| SCK | 44 |
| CS | 46 |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/04_read_imu
sf flash vehicle/examples/04_read_imu -m             # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

加速度（g）と角速度（rad/s）の6軸データを10Hzでシリアルコンソールに表示します。

## 学べること

- BMI270Wrapper HALドライバの使い方
- SPI通信の初期化
- IMUデータの読み取りと表示
- 加速度と角速度の物理的意味
