# コントローラ改造計画：メニューシステム＆マルチ通信モード

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**ステータス: ✅ 主要機能完了**

---

## 完了サマリー

### 実装状況

| Phase | 内容 | 状態 |
|-------|------|------|
| Phase 1 | メニュー基盤 | ✅ 完了 (2025-01-06) |
| Phase 2 | メニュー操作 | ✅ 完了 (2025-01-06) |
| Phase 3 | メニュー項目実装 | ✅ 完了 |
| Phase 4 | USB HIDモード | ✅ 完了 (2025-01-07) |
| Phase 5 | WiFi UDPモード | ✅ 完了（追加実装） |
| Phase 6 | 設定永続化 | ✅ 完了 |
| Phase 7 | シミュレータ連携 | ⏳ 未着手 |

### 実装済み機能

| 機能 | 説明 |
|------|------|
| 3通信モード | ESP-NOW / WiFi UDP / USB HID |
| メニューシステム | 9画面、10メニュー項目 |
| NVS設定保存 | 通信モード、スティックモード、バッテリー警告、デッドバンド、キャリブレーション |
| スティックキャリブレーション | 手動キャリブレーション機能 |
| デッドバンド調整 | 0-5%可変 |
| バッテリー警告 | 3.0V-4.0V閾値設定 |

---

## 1. 概要

### 目標

1. **メニューシステム**: M5ボタンでメニュー画面を表示、スティックで操作
2. **トリプルモード**: ESP-NOW / WiFi UDP / USB HID の3モード切り替え
3. **拡張性**: 将来の設定・機能追加に対応できる設計

### 現在の実装状態

| 項目 | 状態 |
|------|------|
| LCD | M5GFX、9画面状態、10Hz更新 |
| ボタン | M5.BtnA（メニュー/ペアリング） |
| スティックボタン | Arm, Flip, Mode, Option |
| 通信 | ESP-NOW TDMA / WiFi UDP / USB HID |
| 設定保存 | NVS（5項目） |

## 2. アーキテクチャ

### 通信モード遷移図

```
                    ┌─────────────────────────────────────────┐
                    │           起動時モード選択              │
                    │       NVSから読込 → 3モードのいずれか   │
                    └─────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ↓                   ↓                   ↓
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  ESP-NOW モード  │  │  WiFi UDP モード │  │  USB HID モード  │
│   (実機制御)     │  │   (実機制御)     │  │  (シミュレータ)  │
│                 │  │                 │  │                 │
│  TDMA同期       │  │  Vehicle APに   │  │  PCにジョイ     │
│  10台対応       │  │  自動接続       │  │  スティック認識  │
│  50Hz送信       │  │  50Hz送信       │  │  100Hz送信      │
└─────────────────┘  └─────────────────┘  └─────────────────┘
          │                   │                   │
          └───────────────────┼───────────────────┘
                              ↓
                    メニューから切り替え
                    （USB HIDは再起動必要）
```

### タスク構成

```
┌─────────────────────────────────────────────────────────────────┐
│ app_main                                                        │
│   ├── 起動モード判定（NVSから読込）                              │
│   │      COMM_MODE_ESPNOW (0)                                   │
│   │      COMM_MODE_UDP (1)                                      │
│   │      COMM_MODE_USB_HID (2)                                  │
│   │                                                             │
│   ├── 共通タスク                                                │
│   │      input_task (100Hz, Core1) ← ジョイスティック読み取り   │
│   │      display_task (10Hz, Core1) ← LCD更新                   │
│   │                                                             │
│   ├── ESP-NOWモード専用                                         │
│   │      tdma_send_task                                         │
│   │      beacon_task (Master時)                                 │
│   │      main_loop() → 50Hz制御送信                            │
│   │                                                             │
│   ├── WiFi UDPモード専用                                        │
│   │      udp_main_loop() → 50Hz UDP送信                        │
│   │      Vehicle AP自動検出・接続                               │
│   │                                                             │
│   └── USB HIDモード専用                                         │
│          usb_hid_main_loop() → 100Hz HIDレポート送信           │
└─────────────────────────────────────────────────────────────────┘
```

## 3. メニューシステム

### 画面状態

| 状態 | 説明 |
|------|------|
| SCREEN_FLIGHT | 飛行画面（通常） |
| SCREEN_MENU | メニュー一覧 |
| SCREEN_SETTING | 設定変更画面 |
| SCREEN_ABOUT | バージョン情報 |
| SCREEN_BATTERY_WARN | バッテリー警告設定 |
| SCREEN_CHANNEL | チャンネル表示 |
| SCREEN_MAC | MACアドレス表示 |
| SCREEN_CALIBRATION | キャリブレーション |
| SCREEN_STICK_TEST | スティックテスト |

### メニュー項目（実装済み）

```
┌────────────────────┐
│ ▶ Stick Mode: 2   │  ← Mode 2/3切り替え
│   Comm: ESP-NOW   │  ← ESP-NOW/UDP/USB HID
│   Batt Warn: 3.3V │  ← 3.0V-4.0V
│   Deadband: 2%    │  ← 0-5%
│   Stick Test      │  ← スティック動作確認
│   Calibration     │  ← キャリブレーション
│   Channel         │  ← TDMA CH表示
│   MAC Address     │  ← MAC表示
│   About           │  ← バージョン
│   << Back         │  ← 飛行画面に戻る
└────────────────────┘
```

### メニュー操作

| 入力 | 動作 |
|------|------|
| M5.BtnA 短押し | メニュー ON/OFF |
| M5.BtnA 長押し | ペアリング（ESP-NOW時） |
| スティック ↑ | カーソル上移動 |
| スティック ↓ | カーソル下移動 |
| Mode ボタン | 決定 |

## 4. 通信モード詳細

### ESP-NOW モード（デフォルト）

| 項目 | 内容 |
|------|------|
| プロトコル | ESP-NOW + TDMA同期 |
| 最大コントローラ数 | 10台 |
| 送信レート | 50Hz |
| ペアリング | M5.BtnA長押し |
| 特徴 | 低遅延、マルチコントローラ対応 |

### WiFi UDP モード

| 項目 | 内容 |
|------|------|
| プロトコル | WiFi STA + UDP |
| Vehicle AP | "StampFly_*" を自動検出 |
| Vehicle IP | 192.168.4.1 |
| 制御ポート | 8888 |
| テレメトリポート | 8889 |
| 送信レート | 50Hz |
| 特徴 | 簡単接続、単一Vehicle |

### USB HID モード

| 項目 | 内容 |
|------|------|
| プロトコル | USB 2.0 HID Joystick |
| VID/PID | 0x303A / 0x8001 |
| レポートサイズ | 6バイト |
| 送信レート | 100Hz |
| 特徴 | PC/シミュレータ用 |

**HIDレポート構造:**

| Offset | 名前 | 説明 |
|--------|------|------|
| 0 | Throttle | スロットル (0-255) |
| 1 | Roll | ロール (0-255) |
| 2 | Pitch | ピッチ (0-255) |
| 3 | Yaw | ヨー (0-255) |
| 4 | Buttons | bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode |
| 5 | Reserved | 0x00 |

## 5. NVS設定

| キー | 型 | デフォルト | 範囲 | 説明 |
|-----|------|---------|-------|---------|
| `comm_mode` | u8 | 0 | 0-2 | 通信モード（ESP-NOW/UDP/USB HID） |
| `stick_mode` | u8 | 2 | 2, 3 | スティックモード |
| `batt_warn` | u8 | 33 | 30-40 | バッテリー警告閾値（×0.1V） |
| `deadband` | u8 | 2 | 0-5 | デッドバンド（%） |
| `stick_cal` | blob | {0,0,0,0} | 8B | キャリブレーションオフセット |

## 6. コンポーネント構成

### 実装済みコンポーネント

```
firmware/controller/components/
├── atoms3joy/          # ジョイスティックI2Cドライバ
├── buzzer/             # PWMブザー制御
├── espnow_tdma/        # ESP-NOW TDMAプロトコル
├── menu_system/        # メニューUIシステム
├── sf_udp_client/      # WiFi UDPクライアント
└── usb_hid/            # USB HIDジョイスティック
```

### 主要ファイル

| ファイル | 行数 | 説明 |
|----------|------|------|
| `main/main.cpp` | 2118 | メインエントリ、モードディスパッチャ |
| `components/menu_system/` | 438 | メニューUI |
| `components/usb_hid/` | 150+ | USB HID実装 |
| `components/sf_udp_client/` | 180+ | UDP通信 |
| `components/espnow_tdma/` | - | TDMA同期 |

## 7. 残タスク

### Phase 7: シミュレータ連携（未着手）

```
[ ] simulator/interfaces/joystick.py 更新
    - 新VID/PID対応 (0x303A:0x8001)
    - 自動検出機能
    - read_normalized() 実装
[ ] Genesisシミュレータとの動作確認
```

### 将来の改善

| 項目 | 説明 |
|------|------|
| ESP-NOW ↔ UDP ホットスイッチ | 現在は再起動不要だが、改善の余地あり |
| PLL同期 | TDMAドリフト補正の改善 |
| マルチVehicle（UDP） | 現在は単一Vehicleのみ |

## 8. 制約事項

| 項目 | 内容 |
|------|------|
| LCD | 小型（128x128）→ 1画面6-7行が限界 |
| ボタン | M5.BtnA 1個 → スティック併用必須 |
| USB HIDモード切替 | 再起動必要 |
| NVS容量 | 約4KB → 設定項目は最小限に |

---

<a id="english"></a>

# Controller Modification Plan: Menu System & Multi-Communication Modes

**Status: ✅ Core Features Complete**

---

## Completion Summary

### Implementation Status

| Phase | Content | Status |
|-------|---------|--------|
| Phase 1 | Menu foundation | ✅ Complete (2025-01-06) |
| Phase 2 | Menu navigation | ✅ Complete (2025-01-06) |
| Phase 3 | Menu items | ✅ Complete |
| Phase 4 | USB HID mode | ✅ Complete (2025-01-07) |
| Phase 5 | WiFi UDP mode | ✅ Complete (added) |
| Phase 6 | Settings persistence | ✅ Complete |
| Phase 7 | Simulator integration | ⏳ Not started |

### Implemented Features

| Feature | Description |
|---------|-------------|
| 3 Communication Modes | ESP-NOW / WiFi UDP / USB HID |
| Menu System | 9 screens, 10 menu items |
| NVS Settings | Comm mode, stick mode, battery warning, deadband, calibration |
| Stick Calibration | Manual calibration with averaging |
| Deadband Adjustment | 0-5% variable |
| Battery Warning | 3.0V-4.0V threshold setting |

## 1. Overview

### Goals

1. **Menu System**: Display menu with M5 button, navigate with sticks
2. **Triple Mode**: ESP-NOW / WiFi UDP / USB HID switching
3. **Extensibility**: Design for future settings and features

## 2. Architecture

### Communication Mode Diagram

```
                    ┌─────────────────────────────────────────┐
                    │        Boot Mode Selection              │
                    │     Read from NVS → One of 3 modes      │
                    └─────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ↓                   ↓                   ↓
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   ESP-NOW Mode  │  │  WiFi UDP Mode  │  │  USB HID Mode   │
│  (Vehicle Ctrl) │  │  (Vehicle Ctrl) │  │   (Simulator)   │
│                 │  │                 │  │                 │
│  TDMA sync      │  │  Auto-connect   │  │  PC recognizes  │
│  10 controllers │  │  to Vehicle AP  │  │  as joystick    │
│  50Hz TX        │  │  50Hz TX        │  │  100Hz TX       │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

## 3. Menu System

### Screen States

| State | Description |
|-------|-------------|
| SCREEN_FLIGHT | Flight screen (normal) |
| SCREEN_MENU | Menu list |
| SCREEN_SETTING | Setting change |
| SCREEN_ABOUT | Version info |
| SCREEN_BATTERY_WARN | Battery warning setting |
| SCREEN_CHANNEL | Channel display |
| SCREEN_MAC | MAC address display |
| SCREEN_CALIBRATION | Calibration |
| SCREEN_STICK_TEST | Stick test |

### Menu Items (Implemented)

- Stick Mode: 2/3
- Comm: ESP-NOW/UDP/USB HID
- Batt Warn: 3.0V-4.0V
- Deadband: 0-5%
- Stick Test
- Calibration
- Channel
- MAC Address
- About
- << Back

## 4. Communication Mode Details

### ESP-NOW Mode (Default)

| Item | Details |
|------|---------|
| Protocol | ESP-NOW + TDMA sync |
| Max Controllers | 10 |
| TX Rate | 50Hz |
| Pairing | M5.BtnA long press |

### WiFi UDP Mode

| Item | Details |
|------|---------|
| Protocol | WiFi STA + UDP |
| Vehicle AP | Auto-detect "StampFly_*" |
| Control Port | 8888 |
| Telemetry Port | 8889 |
| TX Rate | 50Hz |

### USB HID Mode

| Item | Details |
|------|---------|
| Protocol | USB 2.0 HID Joystick |
| VID/PID | 0x303A / 0x8001 |
| Report Size | 6 bytes |
| TX Rate | 100Hz |

## 5. Remaining Tasks

### Phase 7: Simulator Integration (Not Started)

- Update simulator/interfaces/joystick.py
  - New VID/PID support (0x303A:0x8001)
  - Auto-detection
  - read_normalized() implementation
- Test with Genesis simulator

## 6. Constraints

| Item | Details |
|------|---------|
| LCD | Small (128x128) → 6-7 lines max |
| Button | Only M5.BtnA → Must use with sticks |
| USB HID mode switch | Requires restart |
| NVS capacity | ~4KB → Minimize settings |
