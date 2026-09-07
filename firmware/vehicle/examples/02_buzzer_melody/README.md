# 02_buzzer_melody — ブザーでメロディを鳴らす

## 目的

ESP-IDFのLEDC PWMペリフェラルを使って、ブザーからトーンを生成する方法を学びます。

## 必要な知識

- PWM（パルス幅変調）の基本概念
- 周波数と音階の関係

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | ブザー (GPIO 40) |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/02_buzzer_melody
sf flash vehicle/examples/02_buzzer_melody -m        # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

Cメジャースケール（ドレミファソラシド）を繰り返し再生します。

## 学べること

- LEDC PWMの初期化（タイマー、チャネル）
- PWM周波数の変更によるトーン生成
- デューティサイクルと音量の関係
