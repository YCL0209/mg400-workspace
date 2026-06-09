# 階段二設計:MG400 巡檢坐標系視覺化介面

**狀態**:設計文件(本回合只定架構,尚未實作)。
**目標讀者**:開發 agent / Claude Code,依此分期實作。
**承接**:[`COORDINATE_INTERFACE_DESIGN_NOTES.md`](COORDINATE_INTERFACE_DESIGN_NOTES.md) 的 7 條設計概念。
**協議權威**:[`OFFICIAL_TCP_COMMAND_REFERENCE.md`](OFFICIAL_TCP_COMMAND_REFERENCE.md)、[`OFFICIAL_COORDINATE_SYSTEM_SPEC.md`](OFFICIAL_COORDINATE_SYSTEM_SPEC.md)。

---

## 1. 目標與範圍

做一個**俯視 XY 坐標系介面**,即時呈現:
1. 機械臂的**環形可工作範圍** + 座標格(base 原點、mm 刻度)。
2. 手臂**即時位姿**(TCP 點 + r 朝向,可選簡易連桿)。
3. **eye-in-hand 相機目前拍到的區域(FOV footprint)**,以多邊形框出,隨手臂移動而動。
4. (後續)疊上 AOI 偵測結果(OK / NG / Suspicious)。

**用途 = 巡檢**(手臂帶相機拍配電盤跑 AOI),**非取放**。介面是「看手臂在哪、相機看哪、哪格判 NG」的監看視圖。

### 關鍵前提(務必先讀)
- CV 專案 `phase5-panel` 實為**配電盤 AOI 異常檢測**(PatchCore/anomalib)。其 ArUco(`DICT_4X4_50`, ID 0)
  只用於 `findHomography` **影像對齊**,**與機械臂座標無關**;且**無相機內參、無手眼標定、無手臂通訊**。
  → 要把「相機看哪」畫進機械臂座標,**內參 + 手眼標定是缺的前置(本設計 Step 0)**。
- 相機:台達 DeltaCamera(DMV-SDK)。實機型號 `DMV-CC1M6GM075`(GigE,**monochrome** Mono8),
  解析度 **1440×1080**(M0a smoke 實測確認;舊版本本檔誤寫 1280×960,已修正)。
  Wrapper 把 Mono8 broadcast 成三通道 RGB(R=G=B),下游模型一律拿到 `(H,W,3) uint8`,
  跟彩色相機相容;但 ChArUco/AOI 實際只用一個通道,無資訊損失。

---

## 2. 整體架構(三層)

```
┌─ 標定 artifact 層(JSON,離線一次性) ────────────────────────┐
│  config/camera_intrinsics.json   (K, dist, 解析度)            │
│  config/hand_eye.json            (T_flange←cam 或 T_tcp←cam)  │
└──────────────────────────────────────────────────────────────┘
                │ 載入
┌─ Python WebSocket 後端(mg400-workspace,新模組 viz/) ───────┐
│  • 30004 即時位姿  ← AsyncFeedbackStream(重用)               │
│  • 工作範圍幾何    ← SafetyBounds(重用)                      │
│  • FK(驗證/備援) ← forward_kinematics(重用)                │
│  • 每幀算 FOV footprint(本層新邏輯)                          │
│  • 以 ws 推 JSON 給前端                                       │
└──────────────────────────────────────────────────────────────┘
                │ WebSocket(JSON)
┌─ Three.js / WebGL 前端(正交俯視,看 −Z) ───────────────────┐
│  畫:環形工作範圍 / 座標格軸 / 即時手臂 / FOV 多邊形 / 偵測框   │
│  前端只收「絕對 base 座標」,不做任何 transform(概念④)       │
└──────────────────────────────────────────────────────────────┘
```

**職責切割(概念④)**:所有座標換算在後端做完,前端只渲染後端送來的 base 座標。出錯易二分:算錯(後端)vs 畫錯(前端)。

---

## 3. Step 0 — 標定(前置,必做)

沒有這兩個 artifact,FOV 框只能粗估;要畫準必須先做。兩者**互相獨立**(概念③:對應點+求解器+存檔)。

### 3.1 相機內參標定
- 方法:**ChArUco** 板(棋盤格 + ArUco markers)多角度拍 → `cv2.aruco.calibrateCameraCharuco` →
  `cameraMatrix K` + 畸變 `dist`。ChArUco 比純棋盤格穩在角點識別(partial-occlusion 仍可用)。
- 存:`config/camera_intrinsics.json`(含 `image_width/height`、`K`(3×3)、`dist`、板規格、RMS 殘差等)。
  完整 schema 見 §8.1。
- 採點透過**自家瀏覽器 UI**(viz/ + web/)即時看 ChArUco overlay,順手調焦/光圈,不需開 DMV Studio。
- 一次性;換鏡頭/變焦才重做。鎖鏡頭環是 M0b 前置。
- 詳細實作見 **§8.1 M0b 細節**。

### 3.2 手眼標定(eye-in-hand)
- 目標:相機相對手臂的外參。建議標到 **flange**(對齊 FK 輸出)或 **TCP**(對齊 30004 `.pose`);**全程擇一**。
  本設計**主用 TCP**(因 30004 `.pose` 直接給 TCP 位姿,免跑 FK)→ 解 `T_tcp←cam`。
- 流程:在 N 個(≥10)不同手臂位姿下,各拍一張標定板(棋盤/ArUco),同時記錄當下 TCP 位姿:
  - 手臂位姿:`FeedbackFrame.pose`(30004)或 `get_pose`(29999)。
  - 板對相機的位姿:`cv2.solvePnP`(用 3.1 的 K)。
  - 解:`cv2.calibrateHandEye(R_gripper2base, t_..., R_target2cam, t_..., method=PARK/TSAI)` → `T_tcp←cam`。
- 存:`config/hand_eye.json`(`R`(3×3)、`t`(3)、標定到 TCP/flange 的註記、樣本數、殘差)。
- 互動採集腳本放 `robot_core/scripts/handeye_calib.py`,沿用既有 `collect_pairs.py` / `feedback_test.py` 模式。

> ⚠️ 4 軸 MG400 只有 J1+J4 的單一 yaw 自由度,姿態變化受限;手眼採樣要盡量在 XY 平面散開 + 改變 r,確保解穩定。

---

## 4. FOV footprint 數學(核心)

巡檢時相機看到工作平面上一塊矩形;把它畫進 base 座標:

1. **取即時相機位姿**:`T_base←tcp`(來自 30004 `.pose` 的 `{x,y,z,r}`,4 軸只有 yaw r)
   → `T_base←cam = T_base←tcp · T_tcp←cam`(手眼外參)。
2. **影像四角反投影**:角點 `(0,0),(W,0),(W,H),(0,H)` 經 `K⁻¹` → 相機座標系射線(可選先 `undistort`)。
3. **與工作平面相交**:工作平面 `z = z_plane`(配電盤面,設定值或由拍照高度推)。每條射線在 `T_base←cam` 下
   與該平面求交 → 得 4 個 base 座標點 `(x_i, y_i, z_plane)`。
4. **連成多邊形** = FOV 框,投影到俯視 XY 畫上去。

- 巡檢通常在**固定拍照位姿**拍照:靜止時框靜止;手臂移動時(eye-in-hand)框跟著動。
- **目視驗證**:把已知尺寸物(或 ArUco 板)放工作平面,FOV 框的位置/大小若與實際吻合,代表內參+手眼標定正確 —— 這是最直接的標定 sanity check。

---

## 5. 資料模型 / WebSocket 訊息 schema(草案)

JSON,單位 mm / 度,位姿一律 `{x,y,z,r}`(概念⑦)。

**(a) 連線時推一次:`workspace`(靜態幾何)**
```json
{
  "type": "workspace",
  "annulus_inner_mm": 123.83,
  "annulus_outer_mm": 440.0,
  "z_min_mm": <from safety>, "z_max_mm": <from safety>,
  "j1_range_deg": [-160, 160],
  "j1_rear_dead_zone_deg": <from safety>,
  "origin": [0, 0],
  "grid_step_mm": 50
}
```

**(b) 即時逐幀推:`state`(~8ms / 可降頻)**
```json
{
  "type": "state",
  "pose":   {"x":..,"y":..,"z":..,"r":..},
  "joints": [j1,j2,j3,j4],
  "fov_polygon": [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
  "flags": {"enabled": true, "error": false},
  "detections": [
    {"id": 1, "name":"Main Breaker", "x":.., "y":.., "r":.., "status":"NG"}
  ]
}
```
- `pose`/`joints`/`flags` ← `FeedbackFrame.pose` / `.joints` / `.is_enabled` / `.has_error`。
- `fov_polygon` ← §4 計算。
- `detections` ← M3 由 phase5-panel 推入(M1/M2 先空陣列)。

---

## 6. 前端渲染規格(Three.js / WebGL,正交俯視)

- **相機**:`OrthographicCamera` 看 −Z,+X 朝前、+Y 朝左(對齊 base:原點在 J1 軸,+X 前、+Z 上)。
- **環形工作範圍**:內徑 123.83 / 外徑 440 的環,扣掉 J1 後方死區扇形(`j1_rear_dead_zone_deg`)與 J1 角度範圍 → 真實可達扇環。
- **座標格 + 軸**:mm 刻度格線、X/Y 軸、base 原點標記、半徑刻度。
- **即時手臂**:最低限度畫 TCP 點 + r 朝向箭頭;可選用 `joints` + FK 畫簡易兩連桿俯視。
- **FOV 多邊形**:`fov_polygon` 連線填半透明,標「相機視野」。
- **偵測框**:`detections` 以帶 r 朝向的小框畫在其 base 座標,OK 綠 / NG 紅 / Suspicious 橘(對齊 phase5-panel 配色)。
- 前端**不做任何座標換算**,純畫後端送來的點。

---

## 7. 重用對照表(指向真實 API)

| 需要的能力 | mg400-workspace 既有 | 簽名 / 用法 | 動作 |
|---|---|---|---|
| 30004 即時位姿/關節串流 | `robot_core/transport/feedback_stream.py` | `AsyncFeedbackStream(host, port)` async context manager,逐幀產出 `FeedbackFrame` | **重用** |
| 單幀解析 / 欄位 | `robot_core/transport/feedback.py` | `parse_feedback(bytes)`、`FeedbackFrame.pose -> (x,y,z,r)`(TCP)、`.joints`、`.is_enabled`、`.has_error` | **重用** |
| FK(驗證/備援) | `robot_core/kinematics/forward.py` | `forward_kinematics(j1,j2,j3,j4) -> (x,y,z,r)`(**flange centre**) | **重用** |
| 工作範圍幾何 | `robot_core/safety/bounds.py` + `config/safety.json` | `SafetyBounds.load()` → `annulus_inner_mm/annulus_outer_mm/z_*/j1_rear_dead_zone_deg/joint_ranges_deg/coupling` | **重用** |
| 主動查詢位姿(備援) | `robot_core/protocol/builders.py` | `get_pose` / `get_angle`(29999) | **重用** |
| 連線設定 | `config/robot.json` | `ip 192.168.1.6`、`feedback 30004` | **重用** |
| ws 後端 / FOV 計算 / 前端 | — | — | **全新**(`viz/` 後端 + 前端目錄) |
| 相機內參 / 手眼標定腳本 | — | `cv2.calibrateCamera` / `cv2.calibrateHandEye` | **全新**(`robot_core/scripts/handeye_calib.py`) |

> 注意:FK 給 **flange centre**,30004 `.pose` 給 **TCP**;手眼標定到哪個基準就全程用哪個,別混(本設計用 TCP)。

---

## 8. 建置里程碑(分期,逐步落地)

| 里程碑 | 內容 | 產出 | 狀態 |
|---|---|---|---|
| **M0a** | DeltaCamera adapter + multi-camera 選擇 + smoke | `robot_core/camera/`、`scripts/{list_,}camera_smoke.py`、camera_serial 進 config | ✅ merged `5a2eb9d` |
| **M0b** | 相機內參標定(瀏覽器式 live capture + ChArUco) | `config/camera_intrinsics.json`、`viz/calib_*`、ChArUco 板列印腳本 | ⏳ planned (§8.1) |
| **M0c** | 手眼校正(eye-in-hand,跟手臂同 session) | `config/hand_eye.json`、`viz/handeye_*` + `web/handeye.html` | ⏳ planned (§8.2) |
| **M0d** | sanity verify(已知物投影 vs 實放) | `scripts/handeye_verify.py` | 🔒 等 M0c |
| **M1** | ws 後端骨架 + 前端畫「靜態工作範圍 + 座標格」(不接相機) | `viz/` 後端 + 前端;推 `workspace` 訊息 | ✅ merged `6241111` |
| **M2** | 接 30004 即時位姿 + 算/畫 FOV 框 | 推 `state`(pose/joints/fov_polygon) | 🔒 等 M0d artifact |
| **M3** | phase5-panel 推 AOI 結果疊框 | phase5-panel 加一個輸出 hook → 後端 `detections` | 🔒 等 M2 + hook |

每個里程碑各自 **plan → build → 驗證**,不一次全建(符合 step-by-step)。

---

## 8.1 M0b 細節:瀏覽器式 live capture + ChArUco

**目標**:從 `DeltaCamera` 抓單張 raw frame、跑 ChArUco 偵測、累積 ≥20 不同視角採樣、解 K 矩陣 +
畸變、寫進 `config/camera_intrinsics.json`。M0c / M2 都依賴這個 artifact。

**為什麼走瀏覽器(不走 cv2.imshow)**:整個 M0b 同時讓 viz/ 順手獲得「相機 stream + ChArUco overlay」
能力,M2 / M3 順著用。調焦距 / 光圈也用同一個 UI,操作員不必在 DMV Studio + Python script 之間切。

### 8.1.1 ChArUco 板規格

| 參數 | 值 | 理由 |
|---|---|---|
| 方格數 | **7 × 10**(可調) | 角點 54 個夠 K 求解、A4 容納合理 |
| 方格邊長 | **20 mm** | 7×10 = 140×200mm 板 + 20mm 白邊 = 180×240mm 紙、A4 印得下 |
| Marker 邊長 | **15 mm**(≈ 0.75 × 方格) | cv2.aruco 常用比例 |
| 字典 | **DICT_4X4_50** | 跟 phase5-panel 同字典避免衝突 |
| 紙張 | **A4 厚紙板**(實際大小列印) | 家用印表機 / 公司影印機都印得了、不用跑 A3 影印店 |

> 為何 A4 而非 A3：1440×1080 相機 + 30cm 工作距離下，20mm 方格約佔 75 px，cv2
> 角點偵測綽綽有餘；角點數一樣 54 個，K 矩陣解出來品質跟 A3 幾乎沒差。
> 真的需要遠距離 (>50cm) 校正時再上 A2 印表機改 spec。

**Single source of truth**:`robot_core/calibration/charuco.py` 定義 board factory,列印腳本、偵測腳本、
solver、artifact JSON 寫的 board metadata 全部從這拿。

### 8.1.2 ws 訊息 schema 擴充(§5 補充)

**(c) Live calib frame 推送(M0b)**

```json
{
  "type": "calib_frame",
  "jpeg_b64": "/9j/4AAQ...",
  "timestamp_ms": 12345,
  "detection": {
    "charuco_corners_found": 23,
    "charuco_corners_total": 54,
    "board_visible": true,
    "marker_ids": [0, 1, 2, ...]
  },
  "captures": {"collected": 8, "target": 20}
}
```
- frame: ~5–10 fps base64 JPEG over JSON ws message。M2 高吞吐情境再考慮 binary frame。
- detection: 偵測到的 ChArUco 角點數、ArUco marker IDs。前端疊文字 overlay。

**(d) Client → backend 動作**

```json
{"action": "capture" | "discard" | "reset" | "solve"}
```
- `capture`: snapshot 當前 frame + detection 進 sample buffer
- `discard`: 退最後一筆
- `reset`: 清空
- `solve`: 跑 `cv2.aruco.calibrateCameraCharuco` 解、回 `calib_result`

**(e) Solve 結果**

```json
{
  "type": "calib_result",
  "success": true,
  "rms_px": 0.45,
  "n_views": 23,
  "K": [[fx,0,cx],[0,fy,cy],[0,0,1]],
  "dist": [k1,k2,p1,p2,k3]
}
```
- success=false 時帶 `error` 欄位說明(視角不足、解出 NaN 等)
- 成功則 backend 同步寫 `config/camera_intrinsics.json`(schema §8.1.4)

### 8.1.3 4 sub-PR 拆分

每個獨立可 merge、各自驗收:

| sub-PR | 內容 | 環境 |
|---|---|---|
| **M0b-1** | ChArUco 規格 + 列印腳本:`robot_core/calibration/charuco.py`、`scripts/charuco_print.py`、`opencv-contrib-python` 進 requirements | 純離線(Mac OK) |
| **M0b-2** | viz/ 後端 live stream:`viz/calib_session.py` 包 DeltaCamera 連續模式 + 偵測、`viz/server.py` 加 `/ws/calib`、推 `calib_frame` | Win(實機相機) |
| **M0b-3** | web/ 前端 live view:`web/src/calib.js` decode JPEG + 疊 overlay + 按鍵 binding(SPACE/ESC/R/D) | Win(連背端) |
| **M0b-4** | Solver + artifact:`solve()` 呼叫 `cv2.aruco.calibrateCameraCharuco`、寫 `config/camera_intrinsics.json`、tests 完整 | Win(實機跑 20+ 採點) |

### 8.1.4 `config/camera_intrinsics.json` schema

```json
{
  "K": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist": [k1, k2, p1, p2, k3],
  "image_width": 1440,
  "image_height": 1080,
  "rms_px": 0.45,
  "n_views": 23,
  "board": {
    "squares_x": 7,
    "squares_y": 10,
    "square_size_mm": 30.0,
    "marker_size_mm": 22.0,
    "dictionary": "DICT_4X4_50"
  },
  "camera_serial": "C1M6GM0W24460005",
  "captured_at": "2026-06-05T22:00:00",
  "tool_version": "M0b-v1"
}
```

驗收 acceptance:`rms_px < 1.0` 算 OK;< 0.5 是優秀;>= 1.0 採點不夠 / 太集中,重採。

### 8.1.5 焦距 / 光圈調整 SOP(M0b-3 完成後)

1. Win 端起 viz + 瀏覽器、進 `/ws/calib` view
2. 鏡頭前放 ChArUco 板
3. 觀察 overlay「corners: X/54 visible」+ 邊緣銳利度
4. **物理調焦距**(轉鏡頭環)直到 corners 數最大 + 邊緣最銳
5. **物理調光圈**直到曝光合理(直方圖偏中,不過亮過暗)
6. **鎖住鏡頭環**(止動螺絲 / 鎖環)
7. 開始 M0b-4 採點循環 — **步驟 6 之後不要再碰鏡頭,碰了校正就要重來**

### 8.1.6 採點品質基準

- 至少 20 個視角(target=20,buffer 提示)
- 涵蓋畫面**各角度**:左上、右下、正中、近、遠、傾斜
- ChArUco 板在畫面中**佔比 > 30%**(太小角點偵測不穩)
- 每張 corners_found ≥ 30(≥ 50% 總數)
- 完成後一張畫面看 RMS + 重投影誤差分布,大於 1.0 就重來

---

## 8.2 M0c 細節:eye-in-hand 手眼校正

**目標**:在 ≥15 個手臂 pose 下,各拍一張 ChArUco 板、同時記錄當下 TCP 位姿,解出
**`T_tcp←cam`**(相機相對 TCP 的剛性外參 4×4 同質變換),寫進 `config/hand_eye.json`。
M2 FOV 反投影、M3 AOI 結果換算 base 座標,都吃這個 artifact。

**為什麼接著 §8.1 走 viz/web**:M0b 把 viz/ 升級為「相機 stream + ChArUco overlay + sample
buffer + solve」一條龍。M0c **直接擴充 calib.html 加 hand-eye mode**,沿用同一條 ws 管道、同一份
ChArUco detector、同一個 SPACE/ESC/R 鍵盤 binding,只多一條「同步抓 30004 TCP pose」的支線。
操作員的肌肉記憶從 M0b 帶下來。

**前置(M0c 開工前必須就位)**:
1. `config/camera_intrinsics.json` 已寫(M0b-4 完成,rms_px < 1.0)
2. 手臂 enable 且 workbench 三條 connected log 都看到(finding 17)
3. 鏡頭環鎖住,**不曾被碰過**(M0b 之後鏡頭一動 K 就失效)
4. ChArUco 板**剛性固定不動**(本設計選桌面平放,見 §8.2.6)

### 8.2.1 演算法

**核心呼叫**:
```python
R_tcp_cam, t_tcp_cam = cv2.calibrateHandEye(
    R_gripper2base=[...],   # 每個 pose 的 R_base←tcp (從 FK / 30004 pose 算)
    t_gripper2base=[...],   # 每個 pose 的 t_base←tcp
    R_target2cam=[...],     # 每個 pose 的 R_cam←board (從 estimatePoseCharucoBoard)
    t_target2cam=[...],     # 每個 pose 的 t_cam←board
    method=cv2.CALIB_HAND_EYE_PARK,
)
```

**每次採點兩條資料同時抓**:
- **(a) 手臂 pose**:`FeedbackFrame.pose` (30004,4 軸 TCP `{x,y,z,r}`) → 轉成 4×4 同質變換
  `T_base←tcp`(z 軸 yaw = r,roll/pitch = 0,因 4 軸末端恆鉛直)
- **(b) 板對相機 pose**:`cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids,
  board, K, dist, None, None)` → rvec/tvec → `T_cam←board`

`method=PARK` 為預設(SDK 認同;TSAI 為備援若 PARK 解品質差時試)。

**Residual sanity check**(solve 後跑):
對每個樣本 i: `T_base←board_pred = T_base←tcp[i] · T_tcp←cam · T_cam←board[i]`。
理論上 `T_base←board_pred` 對所有 i 應該重合(板在 base 系下不動)。
取所有 i 的 `t_base←board_pred` 平均當「真值」、算每樣本到平均的距離 → rms = `rms_residual_mm`。
**< 2mm = OK**;> 5mm = 採點品質差(姿態散布不夠、板搖動、TCP feedback 跟 PnP 不同步)→ 重採。

### 8.2.2 MG400 4 軸採點難點

MG400 只有 J1+J4 的 yaw 自由度 r,**roll/pitch 永遠是 0**——`cv2.calibrateHandEye` 數學上要求
gripper 姿態變化有足夠 rank,否則解奇異。實務需要在以下三個維度散開,才能解出穩定的
`T_tcp←cam`:

| 維度 | 採樣要求 | 為什麼 |
|---|---|---|
| **XY base 位置** | 至少 9 個 XY 區塊(類似 3×3 棋盤)各 1 點 | 不同 xy 看到的板透視不同,給 t 提供解 |
| **r (J1+J4) 變化** | 至少 3 種 r 組合(e.g. r = -60° / 0° / +60°) | 解 R 必需的姿態變化,沒有就會奇異 |
| **J1/J4 各自分散** | 不要全部用同一個 J1 + 變 J4 來湊 r | 4 軸 r = J1+J4 各種拆法物理姿態不同,加 redundancy |

**鏡頭朝向假設**(依實際治具修正本節):相機 mount 在手臂法蘭面、鏡頭**朝向 法蘭 +Y(從上看是手臂前方右側)
或 +X(直前方)**。採點時要保證 **相機畫面框得到整塊板**——工作距離 30cm(M0b 鎖鏡頭時設定),
所以每個 pose 的 (x,y,z) 都要讓相機光軸落在板上、距離板 30cm 左右。

**z 安全**:目前 `config/safety.json` `z_max=116mm`(保守,finding 28 待寫:實機可到 195mm)。
桌面平放方案會把板「墊高至 ~30cm 工作距離」,z=116mm 對 30cm 工作距離夠用(板墊高就好)。

### 8.2.3 ws 訊息 schema 擴充

`/ws/handeye` endpoint(獨立於 `/ws/calib` 但共享 viz server),沿 §8.1.2 pattern:

**(f) Live handeye frame 推送**(類似 `calib_frame`、多 arm 段)

```json
{
  "type": "handeye_frame",
  "jpeg_b64": "/9j/4AAQ...",
  "timestamp_ms": 12345,
  "detection": {
    "charuco_corners_found": 38,
    "charuco_corners_total": 54,
    "board_visible": true,
    "board_pose": {"tx_mm": 12.3, "ty_mm": -4.5, "tz_mm": 312.7}
  },
  "arm": {
    "available": true,
    "pose": {"x": 230.0, "y": 0.0, "z": 60.0, "r": -45.0},
    "joints": [-0.01, 5.21, 32.40, -44.99],
    "mode": 5,
    "enabled": true,
    "has_error": false
  },
  "captures": {"collected": 8, "target": 15}
}
```

`arm.available=false` 時 backend 沒接上 30004(可能手臂沒開、或 transport 還沒連),前端顯示
「ARM: OFFLINE」、disable SPACE。`mode != 5`(running / error / disabled)時顯示警告但不擋
SPACE——操作員的責任。

**(g) Client → backend action**(同 calib action 名稱,獨立 buffer)

```json
{"action": "capture" | "discard" | "reset" | "solve"}
```

`capture` 時 backend 同時 snap 當前 jpeg-decoded frame + 抓 `RobotState.snapshot()` 配對成
一筆 `HandeyeSample`。若 `arm.available=false`,server 回 `handeye_error`(不入 buffer)。

**(h) Solve 結果**

```json
{
  "type": "handeye_result",
  "success": true,
  "n_samples": 18,
  "method": "CALIB_HAND_EYE_PARK",
  "rms_residual_mm": 1.42,
  "R": [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]],
  "t": [tx, ty, tz],
  "artifact_path": "config/hand_eye.json"
}
```

`success=false` 時帶 `error` 欄位(樣本不足、PARK 解奇異、rms 過大等);**不寫 artifact**。
跟 M0b-4 同坑:`rms_residual_mm` 缺值時整欄省略,不要送 `NaN`(JSON.parse 會炸,finding 27 教訓)。

### 8.2.4 4 sub-PR 拆分

每個獨立可 merge、各自驗收。仿 M0b 切法:

| sub-PR | 內容 | 環境 |
|---|---|---|
| **M0c-0** | §8.2 設計 doc(本 PR) | 純離線(Mac OK) |
| **M0c-1** | viz/ backend:`viz/handeye_session.py`(沿 `CalibSession` pattern,加 arm pose 抓取 hook 但**先 mock 接口**,arm 訊息回 `available=false`)、`viz/server.py` 加 `/ws/handeye`、`viz/messages.py` 加 handeye_* 型別;web/:`web/handeye.html` + `web/src/handeye.js`(沿 `calib.js` 但加 arm 狀態欄)| Mac 離線可寫測完(無 cv2 環境降級) |
| **M0c-2** | arm pose 接線:`HandeyeSession` 加 `arm_state: RobotStateMonitor` 參數,從 `RobotState.snapshot()` 抓 pose+joints+mode→ `arm.available=true`;`viz/server.py` 啟動時建立 transport + AsyncFeedbackStream + RobotState(沿 §7 重用清單)| Win(實機手臂 + 相機) |
| **M0c-3** | solver + artifact:`HandeyeSession.solve()` 跑 `cv2.calibrateHandEye(PARK)` + residual sanity → `viz/handeye_artifact.py` 寫 `config/hand_eye.json`、tests 完整 | Win(實機跑 15+ 採點) |

**M0c-1 / M0c-2 的切割理由**:Mac 端沒手臂,但有 cv2 + 板,M0c-1 把採點 UI 寫完(arm 段先 stub)
可離線跑 unit tests + Mac 上預演前端、減低 Win 端 debug 時間。M0c-2 在 Win 端接上 arm,只動
backend 接線。

### 8.2.5 `config/hand_eye.json` schema

```json
{
  "T_tcp_cam": {
    "R": [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]],
    "t": [tx_mm, ty_mm, tz_mm]
  },
  "frame": "tcp",
  "method": "CALIB_HAND_EYE_PARK",
  "n_samples": 18,
  "rms_residual_mm": 1.42,
  "intrinsics_file": "config/camera_intrinsics.json",
  "intrinsics_rms_px": 0.894,
  "camera_serial": "C1M6GM0W24460005",
  "captured_at": "2026-06-10T22:15:00",
  "tool_version": "M0c-v1"
}
```

**設計選擇**:
- `frame: "tcp"`(對齊 30004 `.pose`,免跑 FK)——若未來改 flange,改這欄位 + 採點時改抓 FK pose
- `t` 單位 mm(全專案統一,與 `kinematics`/`safety` 一致;cv2 內部用 m,artifact write 時 ×1000)
- `intrinsics_file` 是相對 repo root 的路徑;`intrinsics_rms_px` 是冗餘記錄(快速看「外參品質
  + 內參品質一起記在 artifact 中」),`config/camera_intrinsics.json` 還是真理來源
- `tool_version` 標 M0c-v1;未來改 PARK→TSAI、或重採 schema 改動時版本號 +1

### 8.2.6 採點 SOP

**前置**:
1. 控制器啟動 + 設定 `TCP/二次開發模式`(finding 11)
2. workbench 起、`status` 看 mode=5、Δ30004<0.1mm
3. ChArUco 板**平放桌面**(可用書本/泡棉墊高至 ~30cm 工作距離),用膠帶/重物**固定不晃**
4. 瀏覽器開 `http://<win-ip>:8000/handeye.html`,看 `ARM: ONLINE`、ChArUco overlay

**採點循環**(每個 pose):
1. workbench `move_l <x> <y> <z> <r> <speed>` 移到大致目標
2. (可選)物理 unlock 進拖曳示教,手動微調姿態讓 board 在畫面中**佔比 > 30%**
3. 退拖曳 → workbench `enable`(若退使能)→ 等手臂停穩
4. 瀏覽器看 `corners: X/54` ≥ 30 + `mode=5 enabled=Y err=N` 確認
5. **SPACE** → 抓 frame + arm pose 配對入 buffer(計數 +1)
6. ESC/R/D 用法同 M0b(reset / discard last / 退出)

**solve**(採滿 ≥15):
7. 按 web UI 的 **Solve** 按鈕(或送 `{"action": "solve"}`)
8. backend 跑 `calibrateHandEye(PARK)` → 算 residual_mm → 寫 `config/hand_eye.json`
9. UI 顯示 `n_samples / rms_residual_mm / artifact_path`
10. **rms_residual_mm < 2mm** = OK;否則重採(改善姿態散布、檢查板固定、確認 arm 在 SPACE 時 stable)

**收尾**(同 workbench SOP 收尾 14-17):save 不需要(handeye 用 artifact)、disable、`q`。

### 8.2.7 採點品質基準

- **數量**:≥15 sample(`target_views = 15`,buffer 提示)
- **XY 散布**:9 宮格各 ≥ 1 sample(對 base XY 平面,以板中心為原點 ±150mm 範圍切 3×3)
- **r 變化**:至少 3 個不同 r 組(以 ±60° 為跨度)、同 r 不可超過 8 sample
- **每張 corners_found ≥ 30**(≥ 50% 總角點,跟 M0b 同)
- **arm stable**:SPACE 觸發時 `mode==5 && error==False`,UI 在 unstable 時顯示 ⚠ 但不擋
- **rms_residual_mm**:< 2mm = OK、< 1mm = 優秀、≥ 5mm 重採

### 8.2.8 設計決策一覽(本 PR 拍板)

| 決策 | 選擇 | 理由 |
|---|---|---|
| 採點介面 | 擴充 calib.html / `viz/` 加 hand-eye mode | 沿用 M0b live stream + ChArUco overlay 基礎,操作員 SOP 一致;新增 endpoint + handeye.html、不動 calib.html |
| 手臂移動方式 | workbench 手動 `move_l` 定點 + 拖曳微調 | T8/T9 已驗 motion path 穩;操作員可即時看畫面調姿態、最大彈性 |
| 板固定方式 | 桌面平放(書本/泡棉墊高至工作距離) | 簡單、剛性最好;z=116mm safety 上限對 30cm 工作距離夠用 |
| 採點觸發 | 操作員手動 SPACE | 跟 M0b SOP 同;留最後一道人工 sanity(arm stable + corners 數) |

未拍板,留 M0c-1 plan 時決(不影響 §8.2 設計):
- 採點數 buffer 上限(15 / 20 / 30?)
- web/handeye.html 是否分離成獨立檔、或加 calib.html 的 mode switcher
- arm 段 ws 推送頻率(跟 calib frame 同 5-10 fps,或單獨抽?)

### 8.2.9 sanity verify hooks(銜接 M0d)

M0c 收尾不做 verify(那是 M0d 範圍),但 §8.2 落地後 M0d 需要這條資料才能跑:

```python
# M0d 偽碼:已知物投影 vs 實放
T_tcp_cam = load_hand_eye("config/hand_eye.json")  # ← M0c artifact
T_base_tcp = pose_to_matrix(get_pose())            # ← 當下 TCP pose
T_base_cam = T_base_tcp @ T_tcp_cam
# 拍一張 ChArUco 板放已知 base 座標,projectPoints 板角點 → 期待與
# 影像中的偵測角點 < 5mm error
```

→ M0d 額外只需 `scripts/handeye_verify.py`,不再動 viz/。

---

## 9. 與 phase5-panel 的邊界

- `phase5-panel` 保持**獨立 AOI 系統**,不引入機械臂依賴、不被塞坐標系介面進它的 cv2 視窗。
- 唯一接點在 **M3**:它多一個「把判定結果(`results` 內 `is_ng`/`suspicious`)+ 當前拍攝格位往外推」的輸出 hook
  (其 `專案總覽.md` 已預留 PLC/控制端接口)。傳輸建議走後端的 ws/socket,格式對齊 §5 `detections`。
- 兩專案鬆耦合:phase5-panel 只「產生事件」,座標換算與渲染都在 mg400-workspace 端。

---

## 10. 設計原則回扣(對應 NOTES 7 概念)

- **②逐指令顯式綁 User/Tool**:後端送運動/查詢時顯式帶坐標系;介面顯示時標明是哪個 frame。
- **③標定=對應點+求解器+artifact**:內參、手眼各自成 JSON artifact,可版本化、可追溯。
- **④轉換在後端、前端只收絕對座標**:見 §2/§6。
- **①可組合變換鏈**:`T_base←tcp · T_tcp←cam`(再到平面交點)就是鏈式組合;換相機只重標 `T_tcp←cam`。
- **⑤點位模板化**:偵測點 = 視覺給 x,y,r + 固定 z_plane。
- **⑦ID 而非順序、統一單位/位姿**:detections 用 ID;全程 `{x,y,z,r}`、mm/度。

---

## 附:開放項(實作前再決定)
- 工作平面 `z_plane` 來源:設定常數、由拍照高度推、或標定時記錄?
- ws 推送頻率:8ms 原速 or 降頻(如 30–60Hz)以省前端負擔?
- 前端是否需要側視/3D 切換,或本期只做俯視?
- 相機畸變是否需在 FOV 反投影前 `undistort`(廣角鏡才明顯)?
