# 05_read_tof — ToFセンサで距離を測る

## 目的

I2C通信でVL53L3CX Time-of-Flightセンサからレーザー距離計測を行う方法を学びます。

## 必要な知識

- I2C通信の基本（SDA, SCL, アドレス）
- Time-of-Flight測距の原理

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | VL53L3CX ToFセンサ (I2C) |

| 信号 | GPIO |
|------|------|
| SDA | 3 |
| SCL | 4 |
| XSHUT (bottom) | 7 |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/05_read_tof
sf flash vehicle/examples/05_read_tof -m             # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

底面のToFセンサで連続測距し、距離（mm）をシリアルコンソールに表示します。

## 学べること

- I2Cマスターバスの初期化
- VL53L3CXWrapper HALドライバの使い方
- ToFセンサの測距開始・データ読み取り
- 測距ステータスによるデータ品質判定
