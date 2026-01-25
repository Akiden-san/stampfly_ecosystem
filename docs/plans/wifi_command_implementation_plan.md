# WiFi Command Implementation Plan - StampFly High-Level Commands
# WiFi コマンド実装プラン - StampFly 高レベルコマンド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 背景と目的

### 現状
StampFlyは現在、以下の制御方法をサポート：
- **ESP-NOW**: コントローラからの低遅延制御（50Hz）
- **UDP**: WiFi経由の制御入力（コントローラから）
- **WebSocket**: テレメトリ配信（50Hz、read-only）
- **WiFi CLI**: Telnet経由のCLIアクセス（デバッグ・設定用）

### 課題
- **外部システム（ROS、Python等）から高レベルコマンド（離陸、着陸、ジャンプ等）を実行できない**
- コントローラのスティック操作に依存した低レベル制御のみ
- 自律飛行や研究用途に必要な上位コマンド層が未実装

### 目的
1. WiFi経由で高レベルコマンド（takeoff, land, jump等）を実行可能にする
2. 将来のROS統合を見据えた拡張可能なアーキテクチャ
3. プロトコルを Single Source of Truth として管理
4. テストケースとして「ジャンプ」コマンドを実装

---

## 2. アーキテクチャ提案

### 提案1: CLI拡張（短期実装・推奨スタート）

```
外部システム (Python/ROS)
    ↓ TCP/Telnet接続 (WiFi CLI)
WiFi CLI Server (port 23)
    ↓ 文字列コマンド "jump 0.3"
Console (esp_console)
    ↓ コマンド実行
High-Level Command Service
    ↓ 状態遷移・タイマー管理
ControlArbiter → 制御ループ → モーター
```

**利点:**
- ✅ 既存のWiFi CLI/Serial CLI両方で使える
- ✅ 実装が簡単（コマンド追加のみ）
- ✅ 人間がデバッグしやすい（文字列コマンド）
- ✅ 即座に動作確認可能

**欠点:**
- ⚠️ 文字列パース必要（外部システムから）
- ⚠️ リアルタイム性が低い（Telnetプロトコルのオーバーヘッド）
- ⚠️ プロトコル仕様として形式化されていない

**実装範囲:**
- `firmware/vehicle/components/sf_svc_console/commands/cmd_flight.cpp` 新規作成
- `firmware/vehicle/components/sf_svc_flight_command/` 新規コンポーネント

---

### 提案2: プロトコルレベルコマンド（長期・ROS統合時）

```
外部システム (ROS)
    ↓ WebSocket (binary)
WebSocket Server (port 80)
    ↓ バイナリメッセージ (FlightCommandPacket)
Protocol Handler
    ↓ デコード・検証
High-Level Command Service
    ↓ 状態遷移・タイマー管理
ControlArbiter → 制御ループ → モーター
```

**利点:**
- ✅ バイナリフォーマットで効率的
- ✅ `protocol/spec/` を SSOT として管理
- ✅ ROSノードから直接使いやすい
- ✅ リアルタイム性が高い

**欠点:**
- ⚠️ 実装量が多い（プロトコル拡張、コード生成）
- ⚠️ デバッグが難しい（バイナリ）

**実装範囲:**
- `protocol/spec/flight_commands.yaml` 新規作成
- `protocol/generated/` コード生成
- `firmware/vehicle/components/sf_svc_websocket_cmd/` 新規コンポーネント

---

### 提案3: ハイブリッドアプローチ（★推奨★）

**フェーズ1（即時実装）:**
- CLIコマンドとして実装（提案1）
- 内部的に再利用可能なサービス層を作成

**フェーズ2（ROS統合時）:**
- プロトコルメッセージを追加（提案2）
- 同じサービス層を呼び出す

```
┌─────────────────────────────────────────────┐
│  External Interface Layer                   │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │  WiFi CLI    │  │ WebSocket (future)   │ │
│  │  (文字列)     │  │ (バイナリ)            │ │
│  └──────────────┘  └──────────────────────┘ │
└─────────┬───────────────────┬────────────────┘
          │                   │
          ▼                   ▼
┌─────────────────────────────────────────────┐
│  High-Level Command Service (共通実装)       │
│  - FlightCommandExecutor                    │
│  - 状態遷移 (IDLE→TAKEOFF→HOVER→LAND)       │
│  - タイマー管理                              │
│  - 安全チェック (着陸済み、バッテリー等)      │
└─────────┬───────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────┐
│  Control Layer                              │
│  - ControlArbiter                           │
│  - RateController/AttitudeController        │
└─────────────────────────────────────────────┘
```

**利点:**
- ✅ 段階的実装が可能
- ✅ コードの再利用性が高い
- ✅ 早期に動作検証できる
- ✅ 将来の拡張に対応

---

## 3. 実装詳細（フェーズ1: CLI拡張）

### 3.1 新規コンポーネント構成

```
firmware/vehicle/components/
├── sf_svc_flight_command/          # 高レベルコマンドサービス
│   ├── include/
│   │   └── flight_command.hpp      # サービスAPI
│   ├── flight_command.cpp          # コマンド実行エンジン
│   └── CMakeLists.txt
│
└── sf_svc_console/
    └── commands/
        └── cmd_flight.cpp          # CLIコマンド登録
```

### 3.2 FlightCommandService API設計

```cpp
// firmware/vehicle/components/sf_svc_flight_command/include/flight_command.hpp

namespace stampfly {

enum class FlightCommandType {
    NONE = 0,
    TAKEOFF,      // 離陸
    LAND,         // 着陸
    HOVER,        // ホバリング（指定高度で維持）
    JUMP,         // ジャンプ（離陸→ホバリング→着陸）
};

enum class FlightCommandState {
    IDLE,         // コマンド実行中でない
    RUNNING,      // コマンド実行中
    COMPLETED,    // 完了
    FAILED,       // 失敗
};

struct FlightCommandParams {
    float target_altitude;    // 目標高度 [m]
    float duration_s;         // 持続時間 [s]（HOVERやJUMPで使用）
    float climb_rate;         // 上昇速度 [m/s]
    float descent_rate;       // 降下速度 [m/s]
};

class FlightCommandService {
public:
    static FlightCommandService& getInstance();

    esp_err_t init();

    // コマンド実行（非ブロッキング）
    bool executeCommand(FlightCommandType type, const FlightCommandParams& params);

    // 状態確認
    FlightCommandState getState() const;
    bool isRunning() const { return state_ == FlightCommandState::RUNNING; }

    // キャンセル
    void cancel();

    // 定期更新（制御タスクから400Hzで呼ばれる）
    void update(float dt);

    // 安全チェック
    bool canExecute() const;  // 実行可能か（着陸済み、キャリブレーション完了等）

private:
    FlightCommandService() = default;

    FlightCommandState state_ = FlightCommandState::IDLE;
    FlightCommandType current_command_ = FlightCommandType::NONE;
    FlightCommandParams params_;

    // 内部状態（コマンド実行中の段階）
    enum class ExecutionPhase {
        INIT,
        CLIMBING,
        HOVERING,
        DESCENDING,
        DONE,
    };
    ExecutionPhase phase_ = ExecutionPhase::INIT;

    float elapsed_time_ = 0.0f;
    float hover_timer_ = 0.0f;

    // 制御出力を ControlArbiter に送る
    void sendControlInput(float throttle, float roll, float pitch, float yaw);
};

} // namespace stampfly
```

### 3.3 CLIコマンド実装

```cpp
// firmware/vehicle/components/sf_svc_console/commands/cmd_flight.cpp

static int cmd_takeoff(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (!flight.canExecute()) {
        console.print("Error: Cannot execute. Check landing/calibration status.\r\n");
        return 1;
    }

    if (flight.isRunning()) {
        console.print("Error: Another command is running. Use 'flight cancel' first.\r\n");
        return 1;
    }

    float altitude = 0.5f;  // デフォルト 0.5m
    if (argc >= 2) {
        altitude = atof(argv[1]);
        if (altitude < 0.1f || altitude > 2.0f) {
            console.print("Error: Altitude must be 0.1-2.0 [m]\r\n");
            return 1;
        }
    }

    FlightCommandParams params;
    params.target_altitude = altitude;
    params.climb_rate = 0.3f;  // 0.3 m/s

    if (flight.executeCommand(FlightCommandType::TAKEOFF, params)) {
        console.print("Takeoff command started (target: %.2f m)\r\n", altitude);
        return 0;
    } else {
        console.print("Failed to start takeoff command\r\n");
        return 1;
    }
}

static int cmd_jump(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (!flight.canExecute()) {
        console.print("Error: Cannot execute. Check landing/calibration status.\r\n");
        return 1;
    }

    if (flight.isRunning()) {
        console.print("Error: Another command is running.\r\n");
        return 1;
    }

    float altitude = 0.3f;  // デフォルト 0.3m（小さめのジャンプ）
    if (argc >= 2) {
        altitude = atof(argv[1]);
    }

    float hover_duration = 0.5f;  // デフォルト 0.5秒ホバリング
    if (argc >= 3) {
        hover_duration = atof(argv[2]);
    }

    FlightCommandParams params;
    params.target_altitude = altitude;
    params.duration_s = hover_duration;
    params.climb_rate = 0.4f;      // 上昇 0.4 m/s
    params.descent_rate = 0.3f;    // 降下 0.3 m/s

    if (flight.executeCommand(FlightCommandType::JUMP, params)) {
        console.print("Jump command started (alt: %.2f m, hover: %.1f s)\r\n",
                     altitude, hover_duration);
        return 0;
    } else {
        console.print("Failed to start jump command\r\n");
        return 1;
    }
}

void register_flight_commands() {
    const esp_console_cmd_t takeoff_cmd = {
        .command = "takeoff",
        .help = "Takeoff to specified altitude [takeoff <altitude_m>]",
        .hint = NULL,
        .func = &cmd_takeoff,
    };
    esp_console_cmd_register(&takeoff_cmd);

    const esp_console_cmd_t land_cmd = {
        .command = "land",
        .help = "Land the vehicle",
        .hint = NULL,
        .func = &cmd_land,
    };
    esp_console_cmd_register(&land_cmd);

    const esp_console_cmd_t jump_cmd = {
        .command = "jump",
        .help = "Jump (takeoff, hover, land) [jump <altitude_m> <hover_sec>]",
        .hint = NULL,
        .func = &cmd_jump,
    };
    esp_console_cmd_register(&jump_cmd);

    const esp_console_cmd_t flight_status_cmd = {
        .command = "flight",
        .help = "Flight command status [flight status|cancel]",
        .hint = NULL,
        .func = &cmd_flight,
    };
    esp_console_cmd_register(&flight_status_cmd);
}
```

### 3.4 使用例

**WiFi CLI経由:**
```bash
# WiFiに接続
$ telnet 192.168.4.1 23

# ジャンプコマンド実行（高度0.3m、ホバリング0.5秒）
stampfly> jump 0.3 0.5
Jump command started (alt: 0.30 m, hover: 0.5 s)

# 状態確認
stampfly> flight status
Flight command: JUMP
State: RUNNING
Phase: HOVERING
Elapsed: 1.2s

# キャンセル（緊急時）
stampfly> flight cancel
Flight command cancelled
```

**Python/ROS経由（将来）:**
```python
import socket

def send_flight_command(command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("192.168.4.1", 23))
    sock.recv(1024)  # ウェルカムメッセージ
    sock.send(f"{command}\n".encode())
    response = sock.recv(1024).decode()
    sock.close()
    return response

# ジャンプ実行
send_flight_command("jump 0.3 0.5")
```

---

## 4. 実装順序

### Phase 1: 基本実装（1-2日）
1. ✅ `sf_svc_flight_command` コンポーネント作成
2. ✅ `FlightCommandService` 基本クラス実装
3. ✅ JUMP コマンド実装（テストケース）
4. ✅ CLI コマンド登録 (`cmd_flight.cpp`)
5. ✅ 動作確認（WiFi CLI経由でジャンプ）

### Phase 2: コマンド拡張（1日）
6. ✅ TAKEOFF コマンド実装
7. ✅ LAND コマンド実装
8. ✅ HOVER コマンド実装
9. ✅ 安全チェック強化（バッテリー、姿勢等）

### Phase 3: 外部連携準備（1日）
10. ✅ Python サンプルスクリプト作成
11. ✅ ドキュメント整備
12. ✅ 実機テスト

### Phase 4: プロトコル拡張（将来・ROS統合時）
13. `protocol/spec/flight_commands.yaml` 作成
14. WebSocket経由のバイナリコマンド実装
15. ROSノード作成

---

## 5. 安全性考慮事項

### 実行前チェック
- ✅ 着陸検出済み（`LandingHandler::isLanded()`）
- ✅ キャリブレーション完了（`LandingHandler::canArm()`）
- ✅ バッテリー電圧が十分
- ✅ 既存のコマンドが実行中でない

### 実行中の安全機構
- ✅ 高度制限（最大2.0m）
- ✅ タイムアウト（コマンド実行時間の上限）
- ✅ 異常検出（姿勢角、角速度の閾値超過）
- ✅ 緊急停止（`flight cancel` コマンド）
- ✅ 通信タイムアウト（ControlArbiterの既存機構）

### フェイルセーフ
- コマンド実行中にコントローラ入力を検出したら即座にキャンセル
- 異常検出時は自動着陸シーケンス
- モーター出力は既存のControl Allocationで制限

---

## 6. テスト計画

### 単体テスト
- [ ] FlightCommandService::executeCommand() の状態遷移
- [ ] パラメータ範囲チェック
- [ ] 安全チェックロジック

### 統合テスト
- [ ] WiFi CLI経由のコマンド実行
- [ ] Serial CLI経由のコマンド実行
- [ ] コマンド実行中のキャンセル
- [ ] 異常時の自動停止

### 実機テスト
- [ ] ジャンプコマンド（高度0.2m, 0.3m, 0.5m）
- [ ] 離陸→ホバリング→着陸
- [ ] バッテリー低下時の実行拒否
- [ ] 未着陸時の実行拒否

---

## 7. 将来の拡張可能性

### ROS統合
- `ros2_stampfly` パッケージ作成
- WebSocket経由のバイナリプロトコル実装
- トピック: `/stampfly/command/flight`, `/stampfly/status/flight`

### 追加コマンド候補
- `GOTO` - 指定座標への移動（位置制御実装後）
- `CIRCLE` - 円周飛行
- `WAYPOINT` - ウェイポイント飛行
- `EMERGENCY_LAND` - 緊急着陸

### プロトコル拡張
```yaml
# protocol/spec/flight_commands.yaml (将来)
FlightCommandPacket:
  size: 20
  fields:
    - name: header
      type: uint8
      value: 0xAA
    - name: packet_type
      type: uint8
      value: 0x30  # Flight command
    - name: command_id
      type: uint8
      description: "FlightCommandType enum"
    - name: target_altitude
      type: float32
      unit: m
    - name: duration
      type: float32
      unit: s
    - name: climb_rate
      type: float32
      unit: m/s
    - name: checksum
      type: uint8
```

---

## 8. まとめ

### 推奨アプローチ
**ハイブリッド実装（提案3）** を推奨：
1. **短期**: CLIコマンドとして実装（即座に使用可能）
2. **長期**: プロトコルメッセージ追加（ROS統合時）
3. **共通**: 再利用可能なサービス層（FlightCommandService）

### 利点
- ✅ 段階的実装が可能（リスク低減）
- ✅ 早期に動作検証できる
- ✅ プロトコル SSOT の原則に従う
- ✅ 既存のアーキテクチャを活用
- ✅ 将来のROS統合に対応

### 次のステップ
1. このプランをレビュー
2. Phase 1の実装開始（JUMPコマンド）
3. 実機テスト
4. Phase 2以降に進む

---

<a id="english"></a>

## 1. Background and Objectives

### Current Status
StampFly currently supports the following control methods:
- **ESP-NOW**: Low-latency control from controller (50Hz)
- **UDP**: WiFi-based control input (from controller)
- **WebSocket**: Telemetry streaming (50Hz, read-only)
- **WiFi CLI**: CLI access via Telnet (for debugging/configuration)

### Challenges
- **Cannot execute high-level commands (takeoff, land, jump, etc.) from external systems (ROS, Python, etc.)**
- Only low-level control dependent on controller stick input
- No high-level command layer for autonomous flight and research

### Objectives
1. Enable high-level command execution (takeoff, land, jump, etc.) via WiFi
2. Extensible architecture for future ROS integration
3. Manage protocol as Single Source of Truth
4. Implement "jump" command as test case

---

## 2. Architecture Proposals

### Proposal 1: CLI Extension (Short-term, Recommended Start)

```
External System (Python/ROS)
    ↓ TCP/Telnet connection (WiFi CLI)
WiFi CLI Server (port 23)
    ↓ String command "jump 0.3"
Console (esp_console)
    ↓ Command execution
High-Level Command Service
    ↓ State machine & timer management
ControlArbiter → Control Loop → Motors
```

**Advantages:**
- ✅ Works with both WiFi CLI and Serial CLI
- ✅ Simple implementation (just add commands)
- ✅ Easy for humans to debug (string commands)
- ✅ Immediate testing possible

**Disadvantages:**
- ⚠️ String parsing required (from external systems)
- ⚠️ Lower real-time performance (Telnet protocol overhead)
- ⚠️ Not formalized as protocol specification

---

### Proposal 2: Protocol-Level Commands (Long-term, ROS Integration)

```
External System (ROS)
    ↓ WebSocket (binary)
WebSocket Server (port 80)
    ↓ Binary message (FlightCommandPacket)
Protocol Handler
    ↓ Decode & validate
High-Level Command Service
    ↓ State machine & timer management
ControlArbiter → Control Loop → Motors
```

**Advantages:**
- ✅ Efficient binary format
- ✅ Managed as SSOT in `protocol/spec/`
- ✅ Easy to use from ROS nodes
- ✅ High real-time performance

**Disadvantages:**
- ⚠️ More implementation work (protocol extension, code generation)
- ⚠️ Harder to debug (binary)

---

### Proposal 3: Hybrid Approach (★Recommended★)

**Phase 1 (Immediate):**
- Implement as CLI commands (Proposal 1)
- Create reusable service layer internally

**Phase 2 (ROS Integration):**
- Add protocol messages (Proposal 2)
- Call same service layer

```
┌─────────────────────────────────────────────┐
│  External Interface Layer                   │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │  WiFi CLI    │  │ WebSocket (future)   │ │
│  │  (string)    │  │ (binary)             │ │
│  └──────────────┘  └──────────────────────┘ │
└─────────┬───────────────────┬────────────────┘
          │                   │
          ▼                   ▼
┌─────────────────────────────────────────────┐
│  High-Level Command Service (shared impl)   │
│  - FlightCommandExecutor                    │
│  - State machine (IDLE→TAKEOFF→HOVER→LAND)  │
│  - Timer management                         │
│  - Safety checks (landed, battery, etc.)    │
└─────────┬───────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────┐
│  Control Layer                              │
│  - ControlArbiter                           │
│  - RateController/AttitudeController        │
└─────────────────────────────────────────────┘
```

**Advantages:**
- ✅ Incremental implementation possible
- ✅ High code reusability
- ✅ Early validation possible
- ✅ Future-proof

---

## 3. Implementation Details (Phase 1: CLI Extension)

See Japanese section for detailed API design, CLI implementation, usage examples, implementation order, safety considerations, test plan, and future extensibility.

---

## 8. Summary

### Recommended Approach
**Hybrid Implementation (Proposal 3)**:
1. **Short-term**: Implement as CLI commands (immediately usable)
2. **Long-term**: Add protocol messages (for ROS integration)
3. **Common**: Reusable service layer (FlightCommandService)

### Benefits
- ✅ Incremental implementation (risk reduction)
- ✅ Early validation
- ✅ Follows protocol SSOT principle
- ✅ Leverages existing architecture
- ✅ Ready for future ROS integration

### Next Steps
1. Review this plan
2. Start Phase 1 implementation (JUMP command)
3. Real-world testing
4. Proceed to Phase 2+
