# VPython カメラ Yaw 追従問題

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 状態

- **解決済み**（2026-03-09）

## 2. 原因と修正

問題は2つあった：

### 問題1: カメラが派生プロパティを使用していた

`scene.camera.pos` / `scene.camera.axis` は VPython の派生プロパティで、セッター内部で `scene.center` を上書きする依存関係ループが存在した。

**修正:** プライマリプロパティ (`scene.center`, `scene.forward`, `scene.range`) のみ使用するように変更。

### 問題2: euler[2][0] が step_fast で正しく更新されなかった（根本原因）

| データソース | yaw 値 | 状態 |
|-------------|--------|------|
| `drone.body.euler[2][0]` | 常に 0（まれに正しい値） | 不正 |
| `drone.body.DCM[1,0], DCM[0,0]` から `atan2` | 正しく変化 | 正常 |

DCM はドローンの描画に使われており、ドローンは正しく回転していた。しかしカメラは `euler[2][0]` を使っていたため追従しなかった。

**修正:** カメラの yaw を `euler[2][0]` ではなく DCM から直接計算：
```python
direction = atan2(drone.body.DCM[1,0], drone.body.DCM[0,0])
```

### 残課題

`euler[2][0]` が `step_fast()` で正しく更新されない根本原因は未調査。`step_fast()` 内の 484 行目で `self.euler[2][0]` に書き込んでいるが、レンダリング時に 0 に戻っている。ACRO ログとカメラで異なるタイミングで値が異なるケースも確認された。

---

<a id="english"></a>

## 1. Status

- **Resolved** (2026-03-09)

## 2. Cause and Fix

Two issues were found:

### Issue 1: Camera used derived properties

`scene.camera.pos` / `scene.camera.axis` are VPython derived properties with a dependency loop in their setters that corrupts `scene.center`.

**Fix:** Use only primary properties (`scene.center`, `scene.forward`, `scene.range`).

### Issue 2: euler[2][0] not reliably updated in step_fast (root cause)

| Data source | Yaw value | Status |
|-------------|-----------|--------|
| `drone.body.euler[2][0]` | Always 0 (rarely correct) | Broken |
| `atan2(drone.body.DCM[1,0], DCM[0,0])` | Correctly changing | Working |

DCM was used for drone rendering and showed correct rotation. But the camera used `euler[2][0]`, which stayed at 0.

**Fix:** Compute camera yaw directly from DCM:
```python
direction = atan2(drone.body.DCM[1,0], drone.body.DCM[0,0])
```

### Remaining issue

Root cause of why `euler[2][0]` is not properly updated in `step_fast()` is uninvestigated. Line 484 writes to `self.euler[2][0]`, but the value reverts to 0 by rendering time.
