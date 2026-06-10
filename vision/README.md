# `vision/` — 視覺定位模組

> 起手日:2026-06-10。**目前是 stub**(只有 layer 規則 + package marker),等 M0c milestone close + `phase5-panel` CV code 移植才有業務邏輯。

## 一句話

讀 M0b/M0c 標定 artifact → 跑 CV 推論 → 把像素座標換到手臂 base 座標 → (可選) 走 controller 安全到位。

## 詳細計劃

**`docs/VISION_DEVELOPMENT_PLAN.md`** 是這個模組的設計 doc:為什麼新開、依賴方向、資料夾結構、跟 `robot_core/` / `viz/` / `phase5-panel` 接點、開發節奏 V1-V8、未拍板決策。先讀那份。

## 開發紀律

`vision/CLAUDE.md` 是 layer 規則。修任何子模組前要看一遍依賴方向、單位約定、測試紀律。

## 跟既有模組關係(速查)

```
                       vision/  ← 視覺定位(本模組)
                         ├── 讀 config/{camera_intrinsics,hand_eye,safety,robot}.json
                         ├── (可選)上接 robot_core.controller.move_to(...)
                         └── 跟 robot_core 解耦的 detector(外部 anomalib / phase5-panel)
                            ↑ 只能往下相依
                       robot_core/  ← 手臂控制堆疊
                       viz/         ← UI ws 後端(同事 web 從這邊接)
                       web/         ← 既有 calib/handeye operator 工具(不擴張)
```

## 子目錄(到位後的結構,目前全空)

| 子目錄 | 角色 | 何時填 |
|---|---|---|
| `calibration/` | 讀 `config/camera_intrinsics.json` + `config/hand_eye.json` 包成 typed dataclass | V2(M0c close 後)|
| `pipeline/` | CV pipeline(`camera_source.py` / `detector.py` / `output.py`)| V4 之後(phase5-panel 移植時)|
| `positioning/` | (image_xy, depth_hint) + intrinsics + hand_eye → base 座標 | V6 |
| `integration/` | end-to-end orchestrator(拍 → 偵測 → 換算 → move_to)| V7 |
| `scripts/` | M0d `handeye_verify.py` + `pick_demo.py` 等互動工具 | V3 / V8 |

## 目前能做什麼

```python
import vision  # OK(stub)
import vision.calibration  # OK(空 package)
```

業務邏輯**還沒有**。等 M0c milestone close → 從 V2 `vision/calibration/intrinsics.py` 跟 `hand_eye.py` 開始填。

## 開發節奏(快查)

V1 ✅ stub + planning doc(本 PR)
V2 ⏳ artifact loader(等 M0c close)
V3 ⏳ M0d handeye_verify
V4 ⏳ pipeline 骨架(camera_source + detector interface + Detection schema)
V5 ⏳ phase5-panel CV code 移植進 detector
V6 ⏳ positioning (pixel → base)
V7 ⏳ integration runner
V8 ⏳ end-to-end demo = P10 終局訊號
