# Phase 3 Hardware Test Plan
# Phase 3 ハードウェアテスト計画

> **Note:** This test plan validates the SystemStateManager and CommandQueue integration.
> **注意:** このテスト計画はSystemStateManagerとCommandQueueの統合を検証します。

## Test Environment
## テスト環境

- **Device:** StampFly vehicle with ESP32-S3
- **Firmware:** Latest build (Phase 3 complete)
- **Tools:**
  - Serial monitor: `idf.py monitor` or `sf monitor`
  - WiFi CLI: `python3 tools/stampfly_cli.py --ip 192.168.2.19`

## Pre-Test Checklist
## テスト前チェックリスト

- [ ] Firmware built successfully (no errors)
- [ ] Battery charged (> 3.7V per cell)
- [ ] Propellers removed for safety
- [ ] Serial connection established
- [ ] WiFi connection available (if testing WiFi commands)

---

## Test Suite 1: Boot Sequence & Calibration State
## テストスイート1: 起動シーケンスとキャリブレーション状態

### Test 1.1: Power-On Initialization
### テスト1.1: 電源投入時の初期化

**Procedure:**
1. Power on the vehicle
2. Monitor serial output

**Expected Output:**
```
SystemStateManager initialized
LED state callback registered with SystemStateManager
CommandQueue initialized (max queue size: 8)
FlightCommandService initialized
```

**Success Criteria:**
- ✅ SystemStateManager initializes without errors
- ✅ LED callback registration succeeds
- ✅ CommandQueue initializes with max size 8

**Result:** [ PASS / FAIL ]

---

### Test 1.2: Calibration State Transitions
### テスト1.2: キャリブレーション状態遷移

**Procedure:**
1. Keep vehicle on flat surface
2. Observe LED colors
3. Monitor serial logs for calibration state changes

**Expected Sequence:**
1. **White LED (2-3 seconds):** `FlightState::CALIBRATING`
   - Log: `"LED updated for state transition: X -> 1 (calibration)"`
2. **Green LED:** `FlightState::IDLE`, `CalibrationState::COMPLETED`
   - Log: `"Level calibration complete - attitude reference set"`
   - Log: `"LED updated for state transition: 1 -> 2 (completed)"`

**Success Criteria:**
- ✅ White LED appears during calibration
- ✅ Green LED appears after calibration completes
- ✅ State transition callbacks logged correctly

**Result:** [ PASS / FAIL ]

**Notes:**
_____________________________________________________________________

---

## Test Suite 2: Command Queue with Calibration Wait
## テストスイート2: キャリブレーション待機付きコマンドキュー

### Test 2.1: Command During Calibration (Queuing)
### テスト2.1: キャリブレーション中のコマンド（キューイング）

**Scenario:** Send WiFi command before calibration completes

**Procedure:**
1. Power on vehicle
2. **Immediately** (within 1-2 seconds) send WiFi command:
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```
3. Observe response and LED behavior

**Expected Output:**
```
> Jump command queued (waiting for calibration to complete)
>   Target altitude: 0.15 m
>   Watch for GREEN LED before execution starts
```

**Expected Behavior:**
1. Command accepted into queue
2. White LED continues (calibration ongoing)
3. Once calibration completes → Green LED
4. Command automatically starts executing

**Success Criteria:**
- ✅ Command response indicates "queued (waiting for calibration)"
- ✅ No immediate execution during white LED
- ✅ Automatic execution after green LED appears

**Result:** [ PASS / FAIL ]

**Notes:**
_____________________________________________________________________

---

### Test 2.2: Command After Calibration (Immediate Execution)
### テスト2.2: キャリブレーション完了後のコマンド（即座実行）

**Scenario:** Send WiFi command after green LED appears

**Procedure:**
1. Wait for green LED (calibration complete)
2. Send WiFi command:
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```

**Expected Output:**
```
> Jump command enqueued: climb to 0.15 m then descend
```

**Expected Behavior:**
1. Command accepted
2. **Immediate execution** (no wait for calibration)

**Success Criteria:**
- ✅ Response indicates "enqueued" (not "queued")
- ✅ No calibration wait message

**Result:** [ PASS / FAIL ]

---

### Test 2.3: Consecutive Commands (Sequential Execution)
### テスト2.3: 連続コマンド（順次実行）

**Scenario:** Send multiple commands to test queue processing

**Procedure:**
1. Ensure vehicle is calibrated (green LED)
2. Send first command:
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```
3. Immediately send second command:
   ```bash
   python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
   ```
4. Monitor serial logs for queue processing

**Expected Log Output:**
```
Command enqueued: ID=1, type=4, alt=0.15, queue_size=1
Command enqueued: ID=2, type=4, alt=0.15, queue_size=2
Starting command: ID=1, type=4, retry=1/3
[... command 1 executes ...]
Command completed: ID=1
Starting command: ID=2, type=4, retry=1/3
[... command 2 executes ...]
Command completed: ID=2
```

**Success Criteria:**
- ✅ Both commands accepted into queue
- ✅ Commands execute sequentially (not in parallel)
- ✅ Second command waits for first to complete

**Result:** [ PASS / FAIL ]

**Notes:**
_____________________________________________________________________

---

## Test Suite 3: LED Event-Driven Updates
## テストスイート3: イベント駆動LED更新

### Test 3.1: Calibration LED Transitions
### テスト3.1: キャリブレーションLED遷移

**Procedure:**
1. Power on vehicle
2. Observe LED sequence during calibration

**Expected LED Sequence:**
1. **White:** Calibration in progress
2. **Green:** Calibration complete, ready to arm

**Expected Log:**
```
LED updated for state transition: 0 -> 1 (INIT -> CALIBRATING)
LED updated for state transition: 1 -> 2 (CALIBRATING -> IDLE)
```

**Success Criteria:**
- ✅ LED color matches FlightState
- ✅ State transition callbacks logged
- ✅ No manual LED update calls in logs

**Result:** [ PASS / FAIL ]

---

### Test 3.2: Flight State LED Updates
### テスト3.2: フライト状態LED更新

**Procedure:**
1. After calibration (green LED)
2. Send ARM command (if available) or trigger state change
3. Observe LED changes

**Expected Behavior:**
- LED updates automatically via callback
- No polling or manual updates

**Success Criteria:**
- ✅ LED changes reflect FlightState changes
- ✅ Callback-based updates logged

**Result:** [ PASS / FAIL ]

---

## Test Suite 4: Control Source Tracking
## テストスイート4: 制御ソース追跡

### Test 4.1: WiFi Control Source Registration
### テスト4.1: WiFi制御ソース登録

**Procedure:**
1. Send WiFi command via WebSocket
2. Check SystemStateManager logs for control source update

**Expected Log:**
```
[FlightCommandService] executeCommand called (type=4, alt=0.15)
[ControlArbiter] Received WebSocket control input
```

**Success Criteria:**
- ✅ ControlArbiter updates SystemStateManager with WEBSOCKET source
- ✅ Active control source tracked correctly

**Result:** [ PASS / FAIL ]

---

## Test Suite 5: Timeout & Error Handling
## テストスイート5: タイムアウトとエラー処理

### Test 5.1: Queue Timeout
### テスト5.1: キュータイムアウト

**Scenario:** Command waits too long for calibration

**Procedure:**
1. Power on vehicle
2. **Prevent calibration** (e.g., tilt vehicle, keep it moving)
3. Send WiFi command immediately
4. Wait 30+ seconds

**Expected Output:**
```
Command timeout: ID=1 (enqueued 30000 ms ago)
Command 1 timeout after 30s (preconditions not met)
```

**Success Criteria:**
- ✅ Command times out after 30 seconds
- ✅ Timeout logged with command ID
- ✅ Queue remains functional after timeout

**Result:** [ PASS / FAIL ]

**Notes:**
_____________________________________________________________________

---

### Test 5.2: Queue Full Scenario
### テスト5.2: キュー満杯シナリオ

**Procedure:**
1. Send 9 commands rapidly (queue size = 8)
2. Observe 9th command rejection

**Expected Output:**
```
Failed to enqueue jump command (queue full)
```

**Success Criteria:**
- ✅ 8 commands accepted
- ✅ 9th command rejected with "queue full" message

**Result:** [ PASS / FAIL ]

---

## Test Suite 6: Performance & Real-Time Constraints
## テストスイート6: パフォーマンスとリアルタイム制約

### Test 6.1: Control Loop Performance
### テスト6.1: 制御ループパフォーマンス

**Procedure:**
1. Monitor IMU task logs during command execution
2. Check for 400Hz maintenance

**Expected Log (every 10 seconds):**
```
IMUTask alive: loop=4000, read_fails=0, stack_free=XXXX
```

**Success Criteria:**
- ✅ IMU loop counter increments by ~4000 every 10s (400Hz maintained)
- ✅ No significant loop slowdowns

**Result:** [ PASS / FAIL ]

---

### Test 6.2: Queue Processing Rate
### テスト6.2: キュー処理速度

**Procedure:**
1. Send command during calibration
2. Measure time from calibration complete to command start

**Expected Timing:**
- Calibration complete → Green LED
- Within 100ms → Command execution starts (10Hz queue processing)

**Success Criteria:**
- ✅ Command starts within 100-200ms of calibration completion

**Result:** [ PASS / FAIL ]

---

## Test Summary
## テストサマリー

| Test Suite | Tests | Passed | Failed | Notes |
|------------|-------|--------|--------|-------|
| Boot Sequence | 2 | | | |
| Command Queue | 3 | | | |
| LED Updates | 2 | | | |
| Control Source | 1 | | | |
| Timeout/Error | 2 | | | |
| Performance | 2 | | | |
| **Total** | **12** | | | |

**Overall Result:** [ PASS / FAIL ]

---

## Troubleshooting Guide
## トラブルシューティングガイド

### Issue: Command not executing after calibration
**Symptoms:** Green LED on, but queued command doesn't start

**Debugging Steps:**
1. Check serial logs for `processQueue()` calls
2. Verify `isReadyToExecute()` returns true
3. Check `ReadinessFlags::CALIBRATED` is set

**Solution:** Verify LandingHandler sync is calling `SystemStateManager::setReady()`

---

### Issue: LED not changing color
**Symptoms:** LED stuck on one color despite state changes

**Debugging Steps:**
1. Check for "LED updated for state transition" logs
2. Verify callback registration in init.cpp
3. Check LEDManager initialization

**Solution:** Ensure `subscribeStateChange()` was called successfully

---

### Issue: Commands always rejected
**Symptoms:** "Failed to enqueue" message for all commands

**Debugging Steps:**
1. Check queue initialization logs
2. Verify `CommandQueue::getInstance().init()` was called
3. Check for queue overflow

**Solution:** Restart device, ensure initialization order correct

---

## Test Execution Log
## テスト実行ログ

**Date:** ___________________
**Tester:** ___________________
**Firmware Version:** ___________________
**Device ID:** ___________________

**Notes:**
_____________________________________________________________________
_____________________________________________________________________
_____________________________________________________________________
_____________________________________________________________________
