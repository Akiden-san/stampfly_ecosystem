# 08_battery_monitor — バッテリー電圧・電流を監視する

## 目的

INA3221電力モニタICを使って、バッテリーの電圧・電流・電力をリアルタイムに監視する方法を学びます。

## 必要な知識

- I2C通信の基本
- 電圧・電流・電力の関係（P = V x I）
- LiPoバッテリーの電圧特性（4.2V満充電、3.3V空）

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | INA3221 電力モニタ (I2C, addr=0x40) |

| 信号 | GPIO |
|------|------|
| SDA | 3 |
| SCL | 4 |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/08_battery_monitor
sf flash vehicle/examples/08_battery_monitor -m      # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

バッテリー電圧（V）、電流（mA）、電力（mW）を2Hzでシリアルコンソールに表示します。電圧が3.4V以下になると警告が出ます。

## 学べること

- INA3221（PowerMonitor）HALドライバの使い方
- シャント抵抗による電流測定の原理
- バッテリー状態の監視と低電圧検出
- USB給電とバッテリー給電の判定
