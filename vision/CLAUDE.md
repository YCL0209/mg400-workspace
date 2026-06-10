# `vision/` 層規則(子層,補充根 CLAUDE.md)

`vision/` 是視覺定位模組:讀 M0b/M0c artifact → 跑 CV 推論 → 把像素座標換到 base 座標 → (可選) 走 controller 移動手臂。**詳細開發計劃見 `docs/VISION_DEVELOPMENT_PLAN.md`。**

## 依賴與分層

- `vision/` 在 `robot_core/` 之上、跟 `viz/` 平行。
- **可以 import**:
  - `robot_core.kinematics`(座標換算)
  - 未來的 `robot_core.controller`(motion API)
  - `config/*.json` artifact(camera_intrinsics、hand_eye、safety、robot)
  - 外部:OpenCV、numpy、anomalib、PatchCore、phase5-panel 移植過來的 CV code
- **不可 import**:
  - `viz/`(viz 是 server、不是 lib)
  - `web/`(純前端)
  - 同事 web(他們是 ws client、不會被反向 import)
- `robot_core/` 跟 `viz/` **不可** import `vision/`。若 `viz/` 要送 detection,改成「viz/ 啟動時注入 vision 的 callable」、不直接相依。

## 寫法紀律

- **artifact loader 嚴格**:`vision/calibration/{intrinsics,hand_eye}.py` 是 `viz/calib_artifact.py` / `viz/handeye_artifact.py` 的對偶——schema 對齊 PHASE2 design §8.1.4 / §8.2.5。schema 漂移要在這層 fail-fast。
- **detection schema 是 contract**:`vision/pipeline/output.py` 的 `Detection` dataclass 被 positioning + integration 共用,改它要動三處,加新欄位優先用 `Optional`。
- **單位統一**:跟 robot_core 對齊——**mm / 度**。OpenCV 內部 m → 在 `vision/calibration/hand_eye.py` 讀檔時就轉 mm(cv2 寫入時轉 m)。
- **detector 解耦**:`vision/pipeline/detector.py` 不認識手臂、不認識 base 座標、不認識 hand_eye;它的輸入是 RGB frame、輸出是「像素座標 + class + confidence」(或 mask / bbox)。**把手臂相關依賴推到上面 `positioning/` + `integration/`**。
- **CV pipeline 純函式 / 可離線**:`detector.py` 給定固定 model + frame 應該 deterministic;測試友善;不耦合相機。`camera_source.py` 是唯一接相機的地方。

## 並行模型

- CV 推論本身**可同步**(per-frame、單張、CPU/GPU bound 不是 I/O bound)。
- 跟 `viz/` 整合若要走 async:由 `viz/` 那邊 `asyncio.to_thread(detector.predict, frame)` 包,不要在 `vision/` 自己長 async。
- 端對端 `pipeline_runner.py` 可同步 orchestrator(拍 → 偵測 → 換算 → move_to);若 motion API 是 async,在那一行 await 即可。

## 測試紀律

- `vision/calibration/` 跟 `vision/positioning/` 是純計算層、**全 unit-testable**:固定 K / hand_eye / detection,assert base 座標出來對的(差 < 1mm)。
- `vision/pipeline/detector.py` 用 mock detector(回固定 detection)做 wiring test;真模型 inference 放 integration test、Mac 上 skip / Win 上 run。
- `vision/integration/` + `scripts/` 是 end-to-end 層、**不寫單元測試**;靠 V3 (`handeye_verify.py`) + V8 (`pick_demo.py`) 做硬體 smoke。

## 跟 phase5-panel 的關係

`phase5-panel` 是另一個 repo,使用者既有的 AOI 專案(PatchCore / anomalib)。`vision/pipeline/` 是「把 phase5-panel 的 CV code 拉進來、跟 robot_core 解耦、輸出標準 detection schema」的工作。移植目標:`phase5-panel` 留作 AOI 模型訓練 + 獨立檢驗;**production 推論走 `vision/`**。

## 給後續 sub-PR 的提醒

- 移植 phase5-panel code 進 `vision/pipeline/` 時:先在這份 CLAUDE.md 確認你的 import 圖沒違反分層;不確定就提到設計 doc 跟人對。
- 加新外部依賴(anomalib / 特定 CV 模型)時:寫進 `requirements.txt` + 標 Python 版本約束(同 finding 26 教訓:**3.11.9** 是專案目前唯一 Python)。
- 加新 artifact 檔(例如 `config/detection_classes.json`)時:在 `docs/VISION_DEVELOPMENT_PLAN.md` 開一條 schema section,跟 §8.1.4 / §8.2.5 同等地位。

---

**這層還沒任何 production 業務邏輯**(2026-06-10 起手 stub)。下一個 PR 等 M0c milestone close 跟使用者把 phase5-panel code 搬過來。
