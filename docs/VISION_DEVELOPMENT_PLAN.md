# 視覺定位開發計劃(`vision/` 模組)

> 狀態:設計文件 + 資料夾骨架(本回合只定架構;CV pipeline / scripts 尚未實作)。
> 啟動日:2026-06-10。
> 承接:`docs/PHASE2_COORDINATE_INTERFACE_DESIGN.md` §8 的 M0d / 段 3。
> 相關 PR:M0c-2 (#39) 提供 `config/hand_eye.json` artifact;M0c-3 (#38) 提供 solver 跟 artifact schema。

---

## 1. 為什麼新開 `vision/` 資料夾

2026-06-10 兩個分工決策同時拍板:

1. **前端外包給同事**:UI scene + control panel 由同事用他自己的 web 框架實作。我們既有的 `web/`(calib.html / handeye.html)留作 M0c reality check 收尾 + 未來 M0d verify 工具,**不再擴張**。
2. **使用者主場 = 運動學 + 視覺定位**:`robot_core/kinematics/`(已有)繼續歸 robot_core;新的「**視覺定位**」工作獨立成 `vision/` 模組(資料夾平行於 `robot_core/` / `viz/` / `web/`)。

`vision/` 從一開始就**獨立、可單獨 import、可被同事 web 走後端 RPC 或 stdout 接走**。比把視覺塞進 `robot_core/` 乾淨(robot_core 是「手臂控制」、不該長視覺依賴),也比塞進 `viz/` 乾淨(viz 是「給 UI 看的展示層」、不是計算層)。

---

## 2. 模組邊界與依賴方向

```
        vision/              ← 視覺定位(上層,新增)
        ├── 讀 config/*.json artifact(camera_intrinsics / hand_eye)
        ├── (可選)上接 controller.move_to(...)
        ├── (可選)被 viz/server.py import 來吐 detections 給 ws
        └── 跟 robot_core/ 解耦的 detector(獨立 import 外部 anomalib / cv2)
                          ↑
                      robot_core/    ← 手臂控制堆疊(已有,不動)
                          ↑
                      transport / state / protocol / safety / kinematics / scripts
```

**規則**(對應 CLAUDE.md「分層只能由上往下」原則):

- `vision/` **可以 import**:`robot_core.kinematics`(座標換算)、`robot_core.controller`(未來 motion API)、`config/*.json`(artifact 讀寫)、外部 OpenCV / anomalib / phase5-panel
- `vision/` **不可 import**:`viz/`(viz 不是 lib、是 server)、`web/`(純前端)
- `robot_core/` 跟 `viz/` **不可 import `vision/`**(若 viz/ 要送 detection,改成 viz/ 啟動時注入 vision 的 callable;不直接相依)
- `vision/` **不可 import 同事的 web**(同事的 web 是 client、走 ws 連我們後端;不會反向 import)

---

## 3. 目標資料夾結構(到位後)

```
vision/                                  ← 模組根
├── CLAUDE.md                            ← layer 規則(內容:本檔 §2 + §5 摘要)
├── README.md                            ← 模組 quickstart + 依賴方向
├── __init__.py                          ← 空,標記 Python package
├── calibration/                         ← 標定 artifact loader
│   ├── __init__.py
│   ├── hand_eye.py                      ← 讀 config/hand_eye.json,提供 T_tcp_cam 4x4 + helpers
│   └── intrinsics.py                    ← 讀 config/camera_intrinsics.json,提供 K / dist
├── pipeline/                            ← CV pipeline(從 phase5-panel 移植過來)
│   ├── __init__.py
│   ├── camera_source.py                 ← DeltaCamera frame iterator(包 robot_core/camera/)
│   ├── detector.py                      ← AOI / PatchCore / anomalib 推論層
│   └── output.py                        ← Detection schema (dataclass)
├── positioning/                         ← 視覺 → base 座標換算
│   ├── __init__.py
│   ├── transform.py                     ← (image_xy, depth_hint, hand_eye, intrinsics) → base 座標
│   └── target.py                        ← target pose 生成(z_plane / TCP offset / safety margin)
├── integration/                         ← 視覺 → controller.move_to 整合
│   ├── __init__.py
│   └── pipeline_runner.py               ← end-to-end orchestrator
└── scripts/                             ← 互動 / 驗收工具
    ├── __init__.py
    ├── handeye_verify.py                ← M0d:已知物投影 vs 實放 sanity check
    └── pick_demo.py                     ← end-to-end:拍 → 偵測 → 換算 → move_to
```

> 本 PR 只先建 `CLAUDE.md` + `README.md` + 空的 `__init__.py`、subdir stub。子模組內容等使用者把 `phase5-panel` 的 CV code 搬過來再 PR-by-PR 填。

---

## 4. 跟既有模組的接點

### 4.1 對 `robot_core/`

- `vision/calibration/intrinsics.py` 跟 `vision/calibration/hand_eye.py` 是 `viz/calib_artifact.py` / `viz/handeye_artifact.py` **的對偶**:後者寫 artifact、前者讀 artifact。schema 對齊 §8.1.4 / §8.2.5。
- `vision/positioning/transform.py` 會用 `robot_core/kinematics`(段 2.1 `transform.py` 上線後)的 4×4 同質變換 utility(待寫,目前是 `vision/` 跟 `robot_core/kinematics/transform.py` 兩邊**共用** transformer 工具的好契機)。
- `vision/integration/pipeline_runner.py` 會走 controller(段 1.3 `T10` 上線後)的 `move_to(...)` API。

### 4.2 對 `viz/` 跟同事 web

- 同事 web 是 client、走 ws 連 `viz/server.py`。我們**不改**現有 viz/ schema。
- 如果同事 web 想顯示 detections,**兩條路擇一**:
  - (a) viz/ 啟動時 import `vision/integration/pipeline_runner.py`、把 detections 推進 `/ws/state` 的 `detections` 欄(§5 (b))
  - (b) `vision/` 跑獨立 process、走自己的 ws 端點直連同事 web,viz/ 不知情
- 第一輪先走 **(a)**(reuse 既有 ws schema、最低 friction),等同事 web 真用起來再評估是否拆。

### 4.3 對 `phase5-panel`

- `phase5-panel` 是另一個 repo,使用者既有的 AOI 專案(PatchCore / anomalib)。
- `vision/pipeline/` 是「**把 phase5-panel 的 CV code 拉進來、跟 robot_core 解耦、輸出標準 detection schema**」的工作(段 3.0 規劃)。
- 移植目標:`phase5-panel` 留作 AOI 模型訓練 + 獨立檢驗;**production 推論走 `vision/`**。

---

## 5. 開發節奏(依優先序)

| # | 工作 | 依賴 | 工時 | 性質 |
|---|---|---|---|---|
| V1 | `vision/CLAUDE.md` + `README.md` + 骨架 stub(**本 PR**) | — | ~30 分 | 純文件 |
| V2 | `vision/calibration/intrinsics.py` + `hand_eye.py`(loader + typed dataclass) | M0c reality check close(`config/hand_eye.json` 落地) | ~1 小時 | 離線,Mac 可寫 |
| V3 | `vision/scripts/handeye_verify.py`(= M0d) | V2 + Win 端有板可放 | ~1-2 小時 + Win 驗 | 軟體離線 + Win 一次性驗 |
| V4 | `vision/pipeline/` 骨架:`camera_source.py` + `detector.py` interface + `output.py` schema | V2 | ~2 小時 | 離線,先定 protocol |
| V5 | `phase5-panel` CV code 移植進 `pipeline/detector.py` | V4 + 看 phase5-panel code 結構 | ~未定 | 跨 repo 整合 |
| V6 | `vision/positioning/transform.py`(pixel → base 座標) | V2 + `robot_core/kinematics/transform.py`(段 2.1)| ~2-3 小時 | 純離線數學 |
| V7 | `vision/integration/pipeline_runner.py` + `scripts/pick_demo.py` | V5 + V6 + 段 1.3 controller | ~3-4 小時 | 半離線半 Win |
| V8 | end-to-end demo:放物 → 視覺辨識 → 手臂安全到位(P10 終局) | V7 + 同事 web 接好 | 跑 5 次無 alarm = 達成 | Win |

V1-V3 跟 M0c reality check 的關係:**V1 現在就做**(本 PR);V2/V3 卡在 M0c milestone close 後(`config/hand_eye.json` 要在)。

---

## 6. 還沒拍板的決策(等具體需求再決)

| 決策 | 候選 | 何時決 |
|---|---|---|
| **CV 整合形式** | (a) `pipeline/detector.py` 直接 import phase5-panel 的 code、(b) subprocess + IPC、(c) 包成 git submodule | V5 開工前 |
| **Detection schema** | bounding box / segmentation mask / class+confidence / keypoint | V4 定 schema 時、看 AOI 用途 |
| **z_plane(目標平面高度)來源** | 設定常數 / 拍照高度推 / 標定時記錄 | V6 transform 工作中決 |
| **同事 web ↔ vision detections 接點** | (a) viz/ 啟動 import vision + push ws、(b) vision/ 跑獨立 ws process | V7 看同事 web 結構決 |

---

## 7. 跟使用者三段策略的對應

PHASE2 §8 把 M-milestone 拆 M0a/b/c/d + M1/M2/M3;NEXT_TASKS.md 三段策略把整體拆段 1/2/3。`vision/` 是 **段 3 視覺整合**的本體。

```
段 1 手臂控制堆疊 (99% 完成)
    └─ Phase 6 controller T10 ← 段 1.3,要寫 (vision/V7 依賴)

段 2 座標系 UI 介面 (M0c 收尾中)
    ├─ M0a/M0b/M0c-0/1/3 ✅
    ├─ M0c-2 PR #39 ⏳
    ├─ M0c reality check 🔒 Win 端
    ├─ M0d → 由 vision/V3 (vision/scripts/handeye_verify.py) 接管
    ├─ M1 ✅
    ├─ M2 FOV polygon ← 段 2,仍由 viz/ 後端做,不歸 vision/
    └─ M3 phase5-panel detection 接 viz/ ← 由 vision/ V5+V7 提供 detection,viz/ 推 ws

段 3 視覺整合 = vision/ 模組
    ├─ V2 calibration loader
    ├─ V3 handeye_verify (= M0d)
    ├─ V4-V5 pipeline + phase5-panel 移植
    ├─ V6 positioning (pixel → base)
    ├─ V7 integration runner
    └─ V8 end-to-end demo = P10 終局
```

**前端**:
- 既有 `web/` 留作 M0c calib/handeye operator 工具,不擴張
- 段 2 scene panel 由**同事 web** 接 `viz/server.py` ws,我們不寫前端
- M2 FOV polygon 跟 M3 AOI 疊圖在同事 web 端 render;**我們的責任只到 viz/ 後端把資料推出來**

---

## 8. 本 PR 範圍(V1)

- 新建 `vision/` 資料夾 + `CLAUDE.md` + `README.md` + 各 subdir 的空 `__init__.py`(讓 Python package 結構就位、使用者把 phase5-panel code 搬過來時不必再建)
- 本檔 `docs/VISION_DEVELOPMENT_PLAN.md`
- **不動** robot_core / viz / web / tests
- **不寫** 任何業務邏輯(loader / detector / transform / integration 全留空)

merge 後使用者:
1. 把 `phase5-panel` 的 CV code 搬進 `vision/pipeline/`(自己 PR-by-PR 控制節奏)
2. 等 M0c milestone close(PR #39 Win smoke + rms<2mm + artifact)→ V2 解鎖
