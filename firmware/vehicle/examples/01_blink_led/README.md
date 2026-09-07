# 01_blink_led — MCU内蔵LEDを光らせる

## 目的

ESP-IDFの基本的なプロジェクト構造と、WS2812 RGB LEDの制御方法を学びます。

## 必要な知識

- ESP-IDFプロジェクトのビルド・書き込み方法
- C/C++の基本文法

## ハードウェア

| 部品 | 説明 |
|------|------|
| M5Stamp S3 | MCU内蔵WS2812 LED (GPIO 21) |

## 実行手順

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/01_blink_led
sf flash vehicle/examples/01_blink_led -m            # 書き込み後にモニタを開く。終了は Ctrl+]
```

`idf.py` を直接使う場合は例題ディレクトリで `idf.py build flash monitor`
（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 動作

LEDが赤→緑→青の順に1秒間隔で切り替わります。

## 学べること

- `led_strip` APIによるWS2812制御
- FreeRTOS `vTaskDelay` による時間待ち
- ESP-IDFのログマクロ (`ESP_LOGI`)
