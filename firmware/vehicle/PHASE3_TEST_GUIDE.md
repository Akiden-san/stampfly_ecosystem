# Phase 3 Hardware Test - Quick Start Guide
# Phase 3 ハードウェアテスト - クイックスタートガイド

## Quick Test Procedure
## クイックテスト手順

### 1. Flash Firmware
### 1. ファームウェアを書き込む

```bash
cd firmware/vehicle
source ~/esp/esp-idf/export.sh
idf.py flash
```

### 2. Start Monitoring
### 2. モニタリング開始

**Option A: Color-coded test monitor (recommended)**
```bash
idf.py monitor | python3 ../../tools/test_monitor.py
```

**Option B: Standard monitor**
```bash
idf.py monitor
```

### 3. Run Test Scenarios
### 3. テストシナリオ実行

#### Test A: Calibration & Queue
#### テストA: キャリブレーションとキュー

1. **Power on** vehicle
2. **Immediately** send WiFi command (within 2 seconds):
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```
3. **Observe:**
   - Response: "Jump command queued (waiting for calibration)"
   - White LED → Green LED transition
   - Command auto-executes after green LED

**✅ PASS if:** Command waits for calibration, then executes automatically

---

#### Test B: Sequential Commands
#### テストB: 連続コマンド

1. **Wait** for green LED (calibrated)
2. **Send** two commands rapidly:
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```
3. **Observe:**
   - Both commands accepted
   - Execute one after another (not simultaneously)

**✅ PASS if:** Commands execute sequentially

---

#### Test C: LED State Tracking
#### テストC: LED状態追跡

1. **Power on** vehicle
2. **Watch** LED sequence:
   - White LED (2-3 sec) = Calibrating
   - Green LED = Ready

**✅ PASS if:** LED colors match expected states

---

## Expected Log Output
## 期待されるログ出力

### Successful Boot Sequence
### 正常起動シーケンス

```
[SystemStateManager] Initialized
[CommandQueue] CommandQueue initialized (max queue size: 8)
[FlightCommandService] FlightCommandService initialized
[init] LED state callback registered with SystemStateManager
```

### Command During Calibration
### キャリブレーション中のコマンド

```
[CommandQueue] Command enqueued: ID=1, type=4, alt=0.15, queue_size=1
[StateCallback] LED updated for state transition: 1 -> 2 (calibration complete)
[CommandQueue] Starting command: ID=1, type=4, retry=1/3
[CommandQueue] Command completed: ID=1
```

### Sequential Commands
### 連続コマンド

```
[CommandQueue] Command enqueued: ID=1, type=4, alt=0.15, queue_size=1
[CommandQueue] Command enqueued: ID=2, type=4, alt=0.15, queue_size=2
[CommandQueue] Starting command: ID=1, type=4, retry=1/3
[CommandQueue] Command completed: ID=1
[CommandQueue] Starting command: ID=2, type=4, retry=1/3
[CommandQueue] Command completed: ID=2
```

---

## Quick Checklist
## クイックチェックリスト

- [ ] **Test A:** Command queues during calibration → ✅ Waits for green LED
- [ ] **Test B:** Sequential commands execute in order → ✅ No overlap
- [ ] **Test C:** LED colors match states → ✅ White → Green
- [ ] **Performance:** IMU task maintains 400Hz → ✅ No slowdowns
- [ ] **No Errors:** No "queue full" or timeout messages → ✅ Clean logs

**Result:** [ PASS / FAIL ]

---

## Troubleshooting
## トラブルシューティング

### Problem: Command not executing
**Check:**
```bash
# Verify calibration state in logs
grep "calibration complete" logs.txt

# Check queue processing
grep "processQueue" logs.txt
```

### Problem: LED not changing
**Check:**
```bash
# Verify callback registration
grep "LED state callback registered" logs.txt

# Check state transitions
grep "LED updated for state transition" logs.txt
```

### Problem: Queue full immediately
**Solution:** Restart device, issue is likely queue initialization failure

---

## Monitor Usage
## モニター使用方法

The test monitor highlights important logs with colors:

- **GREEN [STATE]:** State transitions
- **BLUE [QUEUE]:** Command enqueued
- **CYAN [START]:** Command started
- **GREEN [DONE]:** Command completed
- **YELLOW [CALIB]:** Calibration events
- **RED [TIMEOUT]:** Timeout errors
- **RED [ERROR]:** Queue full or other errors

Example:
```
[10:30:15.123] [QUEUE] Command enqueued: ID=1, type=4, alt=0.15
[10:30:15.234] [CALIB] Level calibration complete - attitude reference set
[10:30:15.345] [STATE] LED updated for state transition: 1 -> 2
[10:30:15.456] [START] Starting command: ID=1, type=4, retry=1/3
[10:30:17.789] [DONE] Command completed: ID=1
```

---

## Advanced Testing
## 高度なテスト

### Test Command Timeout
### コマンドタイムアウトのテスト

```bash
# 1. Power on vehicle
# 2. Keep vehicle tilted (prevents calibration)
# 3. Send command immediately
python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
# 4. Wait 30+ seconds
# Expected: Command timeout logged
```

### Test Queue Full
### キュー満杯のテスト

```bash
# Send 9 commands rapidly
for i in {1..9}; do
    python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
done
# Expected: 8 accepted, 9th rejected with "queue full"
```

---

## Report Issues
## 問題報告

If tests fail, provide:
1. Full serial log output
2. Test scenario that failed
3. Expected vs actual behavior
4. Firmware version (git commit hash)

Save logs:
```bash
idf.py monitor | tee phase3_test_$(date +%Y%m%d_%H%M%S).log
```
