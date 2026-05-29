# MG400 二次開發 — 進度交接記錄

> 用途:給新對話/新 session 快速接手的單一事實來源。
> 最後更新:2026-05-29(第一次硬體 session 完成,Phase 2b v1 採集完畢,SDK doc 入手並驗證 protocol 設計)

---

## 專案目標

把 MG400 機械手臂打造成「可擴展的 AI 機器人軟體平台」(非一次性腳本)。

**範圍 / 終局**:做到 **P10(視覺偵測 → 手眼校正 → 手臂安全到位)** 為止;**不做** LLM agent、Web UI、語音、多裝置。

**當前焦點**:**先攻完里程碑一**(完整且安全的控制堆疊),視覺/手眼之後再展開。

開發者背景:軟體/AI 出身,非傳統機器人工程師。Mac 開發、Windows 接手臂測試,GitHub 同步。

Repo: https://github.com/YCL0209/mg400-workspace
手臂 IP: 192.168.1.6(LAN1),四埠:29999 dashboard / 30003 move / 30004 feedback(8ms 推 1440-byte 幀)/ 30005 (200ms) / 30006 (50ms)。

**手邊有的原廠資料**:`Dobot TCP/IP 遠程控制接口文檔(4 軸,2024-04-19)` PDF——對應控制器版本 1.7.0.0。**全部協定都對得上**,我們的 protocol 層設計沒問題。

---

## 路線圖與狀態

### 里程碑一:完整且安全的控制堆疊(進行中)

| Phase | 內容 | 狀態 | PR |
|---|---|---|---|
| 0 | connect/enable/狀態讀/disable + 安全基線 | ✅ | (合於 #1) |
| 1 | async feedback + 事件驅動 RobotState(deadband) | ✅ | #1 |
| 2c 採資料 | q_actual + collect_pairs(10 筆校驗配對) | ✅ | #2 |
| 2a FK | forward kinematics + 校驗介面 + `fit_config` 擬合器 | ✅ | #3 |
| 2 IK | analytic inverse kinematics(閉式解 ≤2 解) | ✅ | #4 |
| 3 protocol | builders / responses / DashboardClient + MoveClient | ✅ | #5 |
| 4 safety | gate.py + bounds(placeholder)+ 6 條拒絕路徑 | ✅ | #6 |
| 2b 採集腳本 | probe_limits.py + calibrate_bounds.py(離線) | ✅ | #7 |
| workbench v1 | scripts 層整合 REPL(status/live/dashboard/sing?/mark) | ✅ | #8 |
| **2b 實採 + v1 bounds** | 13 點實採 → calibrate_bounds → safety.json v1 | ✅ | #9(待開) |
| **3.1 protocol 補完** | ResetRobot/Continue/StartDrag/StopDrag/EmergencyStop/GetErrorID/Sync/GetAngle/GetPose | 未開始 | 離線可寫 |
| **3.2 enable 授權** | reverse-engineer DobotStudio 的 pre-enable 序列(Wireshark) | 未開始 | 要手臂 + 抓封包 |
| **2b v2 (coupling)** | calibrate_bounds 升級非線性擬合 + 補採 5~8 個 coupling 點 | 未開始 | 要手臂 |
| transport framing fix | `_read_frame` 的 `_pending` 在每次 request 開頭清掉 | 未開始 | 離線可寫 |
| 5 motion 原語 | MovJ/MovL 穿過 safety,事件驅動到位(不用 sleep) | 未開始 | 要手臂(且需 3.1 / 3.2 解完) |
| demo 切片 | `move_to(寫死安全座標)` → 回原位,整條串通驗證 | 未開始 | 要手臂 |
| 6 controller | 狀態機 + 任務佇列 + `move_to(pose)` API + 工具 TCP offset | 未開始 | 要手臂 |

### 里程碑二:智慧層(範圍內)
- Phase 9 視覺管線(camera → OpenCV/YOLO,**與機器人執行分離**的獨立 pipeline)
- Phase 10 手眼校正(像素 → 底座座標,接到 `controller.move_to`)

P7 API server / P8 MongoDB 視需要再加;P11 AI agent / 里程碑三 砍掉。

---

## 已鎖定的架構決策(別重新討論,除非有新理由)

### 分層與依賴方向

- **分層**:transport(認 socket 不認機器人)→ state → 上層。純函式(framing/parse)與 I/O 分離。
- **並行模型**:feedback/state 串流走 async;dashboard/move 的 request-response 可同步。分幀與解析寫成 I/O-agnostic 純函式,sync↔async 是薄 wrapper 替換。
- **kinematics 正交於主鏈**:純函式、零 I/O、不 import transport/state 或任何上下層;只依賴 `config/kinematics.json`。safety 用它判工作範圍/奇異點,controller 用它做 pose↔joint 換算。
- **safety 只判斷不執行**:依賴 kinematics + state snapshot(duck-typed),禁止 import protocol/transport。回 `SafetyDecision`,不丟例外。E-stop **不受閘門管**,走獨立通道。
- **reference/ 是協定字典,非架構範本**:原廠 code 唯讀、只查協定,禁止沿用其風格。**SDK PDF 也放在 reference/ 下作 protocol 真理之源**(本地保存,不入 git)。
- **測試框架**:stdlib unittest(零依賴;Windows 跑 Python 3.14.4 + numpy 2.4.4)。
- **RobotState 通知**:edge-triggered callback 訂閱(`subscribe(cb)→unsub`),浮點欄位用 deadband 容差比較。

### workbench(scripts 層整合工具,單一操作入口)

**workbench 不是某個 phase 的拋棄式腳本,是隨平台長大、最後變成日常操作手臂入口的單一終端 CLI。**

- 每完成一個 phase = workbench 多一組指令(verb)
- workbench 本身**零業務邏輯**,每個指令路由到對應 layer
- 同一個工具從手感校正期到自動化期,肌肉記憶不浪費

**workbench v1 現有指令**(Phase 2b 完):
- 被動讀:`status`、`live`
- dashboard 控制:`enable`、`disable`、`clear`、`mode`、`version`(⚠ enable 因 firmware 授權問題目前要走 DobotStudio,workbench enable 永遠回 -1,見 finding 11)
- kinematics 即時讀數:`status` / `live` 自動算 FK pose、與 30004 報的 pose 差(對帳)、距禁區距離
- 奇異點查詢:`sing?`
- 採點:`mark <label>`、`save`
- 離開:`q`

**Phase 5 之後 workbench 將自動獲得**:`jog`、`move`、`start_drag` / `stop_drag`(走 protocol StartDrag,替代物理 unlock 鈕)。

### Git / PR 紀律

- **永遠走 feature branch + PR**,不直接 commit 到 main
- **「Claude Code 說推上去了」≠「真的在 origin」**:push 後另一台 `git branch -a` + `git log --all --oneline | grep` 確認
- PR 描述標明對應路線圖的哪個 Phase
- **outputs/ 在 .gitignore**,但 `outputs/limits_*.json` 要破例(用 `git add -f` 或加 `!outputs/limits_*.json` 例外)

---

## 已完成 layers 速查

```
robot_core/
├── transport/          ✅ socket + FramedConnection + AsyncFeedbackStream
│   ├── connection.py   ⚠ _read_frame 在 firmware 送兩 ; 的錯誤回應時錯位,待修
│   ├── framing.py      (純函式分幀,按單 ; 切。doc 確認此為規範)
│   └── feedback.py     
├── state/              ✅ RobotState + RobotStateSnapshot + RobotStateMonitor
├── kinematics/         ✅ FK + IK + 校驗(實測 Δ30004=0.00mm,100% 對齊實機)
├── protocol/           ✅ 指令拼接 + 回應解析 + 薄 client
│   ├── builders.py     ⚠ 待補 ResetRobot/Continue/StartDrag/StopDrag/EmergencyStop/GetErrorID/Sync/GetAngle/GetPose
│   ├── responses.py    
│   └── client.py       
├── safety/             ✅ 閘門 + bounds(v1 實測,coupling 待 v2)
│   ├── gate.py         (6 條拒絕路徑,E-stop 不受閘門管)
│   ├── bounds.py       
│   └── calibrate_bounds.py  ⚠ v1 不處理非線性 coupling,留 polygon 空
└── scripts/            
    └── workbench.py    ✅ (Phase 2b 操作主入口)
```

`config/`: `robot.json`、`kinematics.json`、`safety.json`(**v1 實測**,coupling 留空)、`calibration_pairs.json`(10 筆)。

`outputs/`(本地)`limits_2026*.json`(13 點實採)+ `calibrated_bounds_*.json`。已 `git add -f` push 到分支 `phase-2b-real-limits`。

---

## 實機驗證得到的重要發現

### 1. r = J1 + J4(10 筆鐵證確認,不 wrap)

末端 pose 的 `r` = J1 + J4,兩係數皆 +1、無 offset。FK 算 r 時**直接相加,不可取模**。

### 2. 解耦平行四連桿模型(FK 殘差 max 0.003mm,實機 Δ30004=0.00mm)

不可套 serial DH。
- `ρ = R0 + L1·sin(J2) + L2·cos(J3)`
- `z = Z0 + L1·cos(J2) − L2·sin(J3)`
- L1=174.21mm、L2=175.07mm、R0=109.50mm、Z0=−53.00mm

### 3. IK 是 FK 的精確反函式

閉式解 ≤2 解;`FK∘IK` 還原 <1e-6;500 筆 property test 通過。

### 4. 座標系(由 factory point 錨定)

factory(J1=0, J2=0, J3≈60, J4≈0)時 pose=(197.2, ~0, -30.3) mm。原點在底座內,+X 正前方,法蘭面為 TCP 量測點。**J2=0 是後臂朝正上方鉛直**(不是橫的)。

### 5. J3 實測上限受 J2 耦合(平行四連桿)

J2≈0 時 jog J3 到 60 即報錯,但規格寫 J3: -25~105。實機可動範圍更小且 J2/J3 互相牽制。Phase 2b v1 採了 5 個 coupling 點,呈非線性 envelope(見 finding 13)。

### 6. 機構/安全事實

- MG400 平行四連桿 4 軸,末端工具恆鉛直
- J2/J3 有煞車,J1/J4 無煞車。沒使能時 J2/J3 鎖死、只 J1 可手轉
- 四軸皆絕對編碼器,斷電記得住位置(免回原點)
- 規格:伸距 440mm、重複精度 ±0.05mm、負載 500g(max 750g)
- 關節範圍(理論):J1 ±160、J2 -25~85、J3 -25~105、J4 ±180。J1 ±160 → 後方 40° 死角

### 7. 錯誤訊號三條通道

- **手臂本體**:LED 變紅、J2/J3 煞車咬上、拖曳示教瞬間脫開(最快)
- **30004 feedback**:`robot_mode → 9`、`error_status` 設旗標、`snapshot.has_error=True`
- **29999 dashboard**:指令回 `ErrorID != 0`;`GetErrorID()` 主動問

### 8. TCP server 多 client OK

workbench + DobotStudio 並存正常,port 不會被「佔走」。

### 9. **第一次硬體 session 完成(Phase 2b v1):13 點實採**

- 7 個外緣伸展點(8 方位扣後方死角)
- 5 個 J2/J3 coupling 點(J2 從 -15 到 +64)
- 1 個 z_floor 點(正前方桌面參考)
- 全部 `mode=5 err=False`,品質乾淨
- commit `a609b65` 在分支 `phase-2b-real-limits`

### 10. **「unlock」物理按鈕的行為(實機驗證,B 型 latched)**

手臂上一顆 latch 按鈕(按一次鎖、再按一次解鎖)。**狀態取決於 servo 是否使能**:
- **未使能時按 unlock**:純機械釋放 J2/J3 煞車——但會把控制器搞到**「全拒絕模式」**(下次接 dashboard 任何指令都會回 -1,連 RobotMode 也是)
- **已使能時按 unlock**:**啟動重力補償**,進入拖曳示教模式(等同 RobotMode 6 BACKDRIVE)

⚠ **規則**:這顆鈕**只能在「已使能」狀態下按**。session 結束前**一定要再按一次跳出**。

### 11. **🚨 dashboard 的 -1 之謎(本次 session 最大發現)**

**DobotStudio Pro 必要,目前不能繞**:
- 我們 `EnableRobot()` 各種 signature 都試過,全部回 `-1,{},`
- 連 read-only `RobotMode()`、`GetErrorID()`、`PowerOn()` 也回 `-1`
- 控制器 power cycle 兩次,救不回

**根本原因(推測)**:DobotStudio 按 enable 時送的不只是 `EnableRobot()`,還有一個**未文件化的「遠端控制授權」指令**。沒這道授權,所有 dashboard 指令都被拒。一旦 DobotStudio 按過 enable,控制器進入「remote authorized」狀態,workbench 接著看 `en=Y` 並能做事。

**目前 workaround**:DobotStudio 開著 → 點 enable → workbench 接手。**Phase 3.2 要 Wireshark 抓 DobotStudio enable 那一刻的 TCP 對話**。

**SDK 錯誤碼表(第 68 頁)印證**:`-1` = 「沒有獲取成功 / 命令接收失敗 / 執行失敗」,**generic 拒絕**——不是 -10000(命令不存在)、不是 -20000(參數錯)、不是 -3xxxx/-4xxxx(類型/範圍錯)。格式都對,純粹是狀態拒絕。

### 12. **韌體在拒絕狀態下會送兩個 `;` 的非標準回應**

```
b'-1,{},;EnableRobot(0.5,0,0,0,0);'
b'-1,{},;PowerOn();'
```

SDK doc(第 5 頁)規範**單 `;` 結尾**:`ErrorID,{value},FuncName(params);`。這支韌體在「remote unauthorized」狀態下把回應拆成兩段。**我們 framer 按單 `;` 切是規範正確的**,但 transport `_read_frame` 把第二個碎片留在 `_pending`,下次 `request()` 拿到上一筆尾巴。修法:`request()` 開頭 `self._pending.clear()` + `self._rx_buffer = b""`。

### 13. **Phase 2b v1 採集發現:coupling envelope 是非線性的**

```
J2     J3max
-14.7  44.5   ← 邊界
 -7.4  50.4
+14.1  55.3
+29.3  55.5   ← J3 上限約飽和在 55~56
+63.9  35.7   ← 其實是 Z 桌面擋住,不是 coupling
```

前 4 點呈「上升後飽和」的曲線(不是直線),第 5 點掉下來因為高 J2 時末端會撞桌面。`calibrate_bounds v1` 的演算法(只用 `J3 > 50` 的點線性擬合)處理不了這種形狀,擬合放棄,coupling polygon 留空。

**現況**:safety v1 沒 coupling 多邊形這層保護,只有「J3 ≤ 77.3 per-axis」這種粗粒度限制。Phase 5 送 MovL 中了真實 coupling 違反時,**控制器自己會 alarm 擋下**——不會撞硬體,但會多 alarm-clear-retry 循環。Phase 2b v2 要解這個。

### 14. SDK doc 確認我們 protocol 設計正確

讀完 PDF(2024-04-19 版,控制器 1.7.0.0)後:
- builders/responses 格式 100% 對齊 doc 規範
- EnableRobot 全部簽名(0/1/4 參數)我們都對
- 回應格式 `ErrorID,{value},FuncName(params);` 對齊
- 30004 feedback 1440-byte 結構對齊
- RobotMode 11 個狀態值現在有完整字典:1 INIT / 2 BRAKE_OPEN / 3 POWER_STATUS (本體未上電) / 4 DISABLED / **5 ENABLE** / **6 BACKDRIVE (拖曳)** / 7 RUNNING / 8 RECORDING / **9 ERROR** / 10 PAUSE / 11 JOG

doc 還揭露我們沒用的指令(Phase 3.1 要補):`ResetRobot`、`Continue`、`StartDrag`、`StopDrag`、`EmergencyStop`、`GetErrorID`、`Sync`、`GetAngle`、`GetPose`,還有後面 Phase 5/6 motion 規劃要選用的 `MovJ`、`MovL`、`MovJIO`、`MovLIO`、`Arc`、`Circle`、`MoveJog`、`Sync`、`SyncAll`、`RelMov*`、`SetPayLoad`、`SpeedFactor`、`SpeedJ/L`、`AccJ/L`、`CP`、`User`、`Tool`、`SetCollisionLevel`、`SetUser`、`SetTool`、`CalcUser`、`CalcTool` 等。

### 15. **z_floor 是「裸 flange baseline」,治具用 offset 相加**

採到的 z_floor = -197mm 是裸法蘭面到桌面的距離。未來裝治具:
```
z_min_effective = z_floor_flange + tool.z_offset_mm
```
**不用每換治具就重採整套工作範圍**——這是 Phase 6 controller 的 TCP/Tool 架構要扛的事。

---

## FK 校驗資料(10 筆真實配對,法蘭中心,mm/deg)

格式:J=(j1,j2,j3,j4) 度 → pose=(x,y,z mm, r 度)。來源 30004 feedback,全部 `is_enabled=false`。已 commit 進 `config/calibration_pairs.json`。

```
A1 factory  J=(-0.007, -0.021, 59.903,   2.681)  pose=(197.229,  -0.023, -30.260,   2.674)
A2 onlyJ4   J=(-0.015, -0.020, 59.903, -70.019)  pose=(197.230,  -0.051, -30.260, -70.033)
B1 J=(-0.070,   6.499, 12.529, 159.755)  pose=(300.120,   -0.365,  82.109, 159.685)
B2 J=(-0.011, -15.363, 40.637, 159.752)  pose=(196.198,   -0.038,   0.965, 159.741)
B3 J=(48.427,  46.154, 19.430, 159.752)  pose=(265.586,  299.425,   9.440, 208.180)
B4 J=(-54.113, 56.777, 16.554, 159.749)  pose=(247.985, -342.741,  -7.433, 105.637)
B5 J=(-54.538,-27.940, 27.310, 159.752)  pose=(106.421, -149.406,  20.579, 105.214)
B6 J=(-2.304,  12.932, 65.225, 159.752)  pose=(221.673,   -8.920, -42.168, 157.448)
B7 J=(87.310,  -1.396, 38.689, 159.760)  pose=( 11.352,  241.641,  11.721, 247.071)
B8 J=(-85.001,  3.588, 41.160, 159.758)  pose=( 21.975, -251.250,   5.641,  74.756)
```

**實機驗證**:第一次硬體 session `status` 顯示 Δ30004=0.00mm。模型完美對齊實機。

---

## Phase 2b v1 採集結果(13 點)

`outputs/limits_2026*.json` 三檔,已 push 到 `phase-2b-real-limits`:

```
# 外緣(7 點):
outer_front       J=(  -1.0, +61.2,  +7.3, -106.2)
outer_leftt45     J=( +41.6, +61.9, +10.0, -106.1)   ⚠ typo label,值正確
outer_left90      J=( +90.3, +48.9, +12.5, -106.1)
outer_left135     J=(+157.0, +52.1,  +0.1, -106.1)
outer_right45     J=( -49.3, +61.1, +14.8, -105.8)
outer_right90     J=( -87.8, +62.4,  +6.7, -105.7)
outer_right135    J=(-159.9, +64.2,  +4.4, -105.7)

# J2/J3 耦合(5 點):
coup_j2_-10       J=(  +2.7, -14.7, +44.5, -105.7)
coup_j2_0         J=(  +2.6,  -7.4, +50.4, -105.7)
coup_j2_20        J=(  +1.2, +14.1, +55.3, -105.7)
coup_j2_40        J=(  +1.2, +29.3, +55.5, -105.7)
coup_j2_70        J=(  +1.4, +63.9, +35.7, -105.7)   ⚠ 實際是 z_floor 不是 coupling

# Z 桌面(1 點):
floor_0           J=(  -1.7, +82.8, +77.3,  +81.2)   ρ≈320, z≈-197mm
```

`calibrate_bounds` 跑出來 → 已寫進 `config/safety.json` v1:
- 工作環:**inner 123.8mm, outer 440mm**
- z 範圍:**[−197, +116]mm**(z 上限沒實採,observed)
- J1: **[−159.9, +157.0]**(實採);J2/J3 上限實採,下限用 spec;J4 完全沒實採,用 spec ±180
- coupling: **`[]`**(v2 待處理)

---

## 下一步(優先順序)

### 🟢 P0:離線可做(下次 session 前)
1. **safety.json v1 寫入 `config/`**(`outputs/safety_v1.json` review 後合進 main)
2. **transport framing fix**(1 行修法 + 補測試模擬「過讀+殘留」情境)
3. **.gitignore 加 `!outputs/limits_*.json` 例外**
4. **PROGRESS.md 入 repo**(本檔)

### 🟡 P1:Phase 3.1 protocol 補完(離線)
5. 新增 builders:`ResetRobot`、`Continue`、`StartDrag`、`StopDrag`、`EmergencyStop`、`GetErrorID`(回 structured tuple)、`Sync`、`GetAngle`、`GetPose`
6. 配套測試 + DashboardClient 對外介面更新

### 🟠 P2:要手臂的 session
7. **Phase 3.2 enable 授權 RE**:Wireshark 抓 DobotStudio enable 那 1~2 秒的 TCP,反推授權序列,實作 `enable_session()`
8. **Phase 2b v2**:
   - 升級 `calibrate_bounds`:處理非線性 / 分段 coupling envelope、含異常點偵測(z_floor 假冒 coupling)
   - 補採 6~8 個 coupling 點(避開 z_floor 影響,在中等 z 高度採)
   - 補採 J3 / J2 / J4 真實下限
   - safety.json v2(含 coupling polygon)
9. **Phase 5 motion 原語**:`MovJ` / `MovL` 穿過 safety,事件驅動到位(走 Sync(),不要 sleep)
10. **demo 切片**:`move_to(寫死安全座標) → 回原位`

### 🔴 P3:Phase 6+
11. **controller**:狀態機 + 任務佇列 + `move_to(pose)` API + **TCP/tool offset 架構**
12. **Phase 9/10**:視覺管線 + 手眼校正

---

## 跨平台/協作提醒

- Mac 改 → commit → push → Windows pull → 比對 commit 碼一致 才測
- 確認「Claude Code 說修好了」後,自己 `git log --all --oneline | grep` 驗證
- Windows asyncio 坑:互動輸入用 `asyncio.to_thread(input)`
- 跑腳本一律 `python -m robot_core.scripts.xxx`
- **不直接 commit 到 main**,永遠走 feature branch + PR
- TCP 多 client 沒問題
- **outputs/ 在 .gitignore**:採資料要 `git add -f`,日後 .gitignore 加 `!outputs/limits_*.json`
- PowerShell vs zsh:zsh 預設不把 `#` 當 inline 註解
- **Claude Code 在 Mac 上安裝**:不要 `npm install -g`(Claude Code 2.1.113+ 在 macOS optional dep bug),用 native installer `curl -fsSL https://claude.ai/install.sh | bash`,確認 `~/.local/bin/claude` 在 PATH 第一位

## 跟手臂工作的 SOP

**啟動序列**(每次接手臂必做):
1. 控制器電源開啟,等 LED 穩定(30~60 秒)
2. **DobotStudio Pro 開,按 Enable**(綠燈、J1234 全使能)。**這步不能省**,workbench enable 目前無效
3. 終端機跑 `python -m robot_core.scripts.workbench`
4. **`status`,確認 Δ30004 < 0.1mm**——go/no-go 訊號

**進入拖曳示教**(採點 / 教學):
5. 物理按 unlock 鈕一次(B 型 latch,卡住「拖曳模式」位置)
6. 雙手都可以拉手臂

**採點循環**:
7. 拖到目標位置 → `status` 確認 `err=N` → `mark <label>`
8. 每 5~10 點 `save` 一次

**越界恢復**(會發生很多次):
9. 鬆開示教手
10. **回 DobotStudio**:點 ClearError → 點 Enable(若退使能)
11. workbench `status` 確認 `en=Y err=N`
12. 物理按 unlock 鈕重新進拖曳
13. 繼續

**收尾**(順序很重要):
14. workbench `save` 確保所有點存檔
15. 物理上**再按一次 unlock 鈕**跳出拖曳模式(漏了下次會莫名 -1)
16. DobotStudio 點 Disable
17. workbench `q`

⚠ **絕對禁止**:
- 通電中、未使能時按 unlock 鈕(會把控制器搞到「全拒絕模式」,只能 power cycle 救)
- 拖曳示教中送任何 dashboard / motion 指令
- 不戴護目鏡 / 不確認周圍淨空就 enable

---

**Phase 2b v1 完成 = 第一個真的能擋下違規動作的 safety 層上線。** 後面 Phase 5 motion 終於有底氣送 MovL 了。
