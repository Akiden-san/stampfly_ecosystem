# 03_button_event — ボタンでLEDを制御する

## 目的

GPIO入力の読み取りとソフトウェアデバウンスの仕組みを学びます。

## 必要な知識

- GPIO入力（プルアップ、アクティブLOW）
- チャタリング（バウンス）とは何か

## ハードウェア

| 部品 | 説明 |
|------|------|
| M5Stamp S3 | ボタン (GPIO 0, アクティブLOW) |
| M5Stamp S3 | MCU LED (GPIO 21, WS2812) |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/03_button_event
sf flash vehicle/examples/03_button_event -m         # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

- ボタンを押すとLEDが緑に点灯
- ボタンを離すとLEDが消灯
- シリアルモニタに押下回数を表示

## 学べること

- `gpio_config` によるGPIO入力設定
- ソフトウェアデバウンスのアルゴリズム
- ポーリングによるイベント検出
