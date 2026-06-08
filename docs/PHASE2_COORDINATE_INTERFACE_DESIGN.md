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
- 方法:棋盤格(chessboard)多角度拍 → `cv2.calibrateCamera` → `cameraMatrix K` + 畸變 `dist`。
- 存:`config/camera_intrinsics.json`(含 `image_width/height`、`K`(3×3)、`dist`、標定日期/RMS 殘差)。
- 一次性;換鏡頭/變焦才重做。

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

| 里程碑 | 內容 | 產出 | 可單獨驗證 |
|---|---|---|---|
| **M0** | Step 0 標定:內參 + 手眼腳本與 artifact | `config/camera_intrinsics.json`、`config/hand_eye.json`、`scripts/handeye_calib.py` | FOV 框目視吻合已知物 |
| **M1** | ws 後端骨架 + 前端畫「靜態工作範圍 + 座標格」(不接相機) | `viz/` 後端 + 前端;推 `workspace` 訊息 | 瀏覽器看到正確環形+格線 |
| **M2** | 接 30004 即時位姿 + 算/畫 FOV 框 | 推 `state`(pose/joints/fov_polygon) | 手臂動→介面手臂與框跟著動 |
| **M3** | phase5-panel 推 AOI 結果疊框 | phase5-panel 加一個輸出 hook → 後端 `detections` | 某格判 NG→介面該格變紅 |

每個里程碑各自 **plan → build → 驗證**,不一次全建(符合 step-by-step)。

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
