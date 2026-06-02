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
| **2b 實採 + v1 bounds** | 13 點實採 → calibrate_bounds → safety.json v1 | ✅ | #9 |
| **3.1 protocol 補完** | ResetRobot/Continue/StartDrag/StopDrag/EmergencyStop/GetErrorID/Sync/GetAngle/GetPose | ✅ | #10 |
| **3.2 enable 授權** | DobotStudio → 設定 → 遠程設置 → `TCP/二次開發模式`(跨重開機保留) | ✅ | 控制器設定,無程式 |
| **2b v2 algo (T7A)** | calibrate_bounds 升級(piecewise envelope / spec-fallback / z_floor masquerade filter) | ✅ | #12 |
| **2b v2 polygon (T7B)** | 重採 coupling 點(push-to-alarm 協定)→ 過 deploy gate → 寫進 config | 未開始 | 要手臂 |
| transport framing fix | `_read_frame` 的 `_pending` 在每次 request 開頭清掉 | ✅ | #9 |
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

### 11. **dashboard 的 -1 之謎(已解,2026-06-01)**

**解答**:控制器有一個「遠程控制模式」設定,**不在這個模式 → 29999/30003 所有外部指令一律 -1**(連 read-only `RobotMode()`/`GetErrorID()`/`PowerOn()` 都拒)。預設不在這模式,所以一台從未被 DobotStudio 設過的控制器,fork-style client(含我們的 workbench)會徹底卡住。

**設定位置**:DobotStudio Pro → **設定** → **遠程設置** → 改成 **`TCP/二次開發模式`**。

**持久性**:**跨控制器電源週期保留**(2026-06-01 實機驗證:設定後 power-cycle 控制器、不開 DobotStudio,workbench `EnableRobot()` 仍回 `0,{},EnableRobot()`)。**設一次,永久有效**。

**對 workbench 的影響(零程式改動)**:設過之後 workbench 自己能 enable;**DobotStudio runtime 不必開、不必按 Enable**。只在首次裝機那一次需要打開 DobotStudio 設這個模式。

**為什麼不是 22000?**(順帶副發現):DobotStudio 自己用的是私有的 **22000** 二進位通道控制手臂,**不走** 29999 公開文字 API(實機 Wireshark 抓包確認,DobotStudio 按 Enable 時 29999 上零封包)。原本以為要抓「DobotStudio 在 29999 多送了什麼授權指令」是空集合 —— 它根本不送 29999。授權就是這個模式設定的事,跟 22000 私有協定無關。

**SDK 錯誤碼表(第 68 頁)印證**:`-1` = 「沒有獲取成功 / 命令接收失敗 / 執行失敗」,**generic 拒絕**——不是 -10000(命令不存在)、不是 -20000(參數錯)、不是 -3xxxx/-4xxxx(類型/範圍錯)。格式都對,純粹是狀態拒絕,正好對應「不在遠程控制模式」的拒絕路徑。

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

### 16. **🚨 `EnableRobot()` 不是 idempotent,雙 enable 把 dashboard 整段卸載(2026-06-01 實機驗證)** ⚠ [待重新調查 — 見 finding 17]

**現象**(workbench 連續兩次 `enable`,reproducible):
```
mg400> enable
Received: 0,{},EnableRobot()         ← 1st call,成功使能
mg400> enable
Received: -10000,{},                  ← 2nd call,失敗
mg400> mode
Received: -10000,{},                  ← 連 read-only RobotMode 也拒
mg400> status
[mode=5 en=Y err=N] ...               ← feedback 30004 仍正常
```

**機制**:對已使能(`en=Y`)的控制器再 call `EnableRobot()` → 韌體回 `-10000` 並**把整個 dashboard 接收器卸載**;所有後續 dashboard 指令(query、set、控制全部)都 `-10000`。Feedback 30004 不受影響,所以從 `status` 看 controller state machine 還活著,只是 29999 dashboard interface 被踢掉。

**注意 `-10000` 是被韌體過載使用**:PDF p68 寫 `-10000 = 命令不存在`,但實際上韌體也用它表達「指令存在但當前狀態拒絕」(此案就是已使能 + 又 enable)。所以看到 -10000 不代表指令真的不存在,要交叉比對。

**復原**:dashboard 死透,沒有 TCP 端的救法(PDF 整本 11 + 12 + 9 + 8 + ... 個指令全部掃過,沒有 reset session 之類)。只能:
- DobotStudio Pro 按 Disable → Enable(走它的 22000 私有通道,可能踢回 dashboard mode)
- 失敗就 **power-cycle 控制器**

**修法(已 commit)**:`workbench.cmd_enable()` 加 pre-check:先讀 feedback snapshot,若 `is_enabled` 為 True 就 short-circuit 不送指令,印「Already enabled — skipping」。任何呼叫 `EnableRobot()` 的程式碼都要遵守這條,否則一次無心多叫就會把 controller 搞死。

**對舊認知的修正**:這次 session 早先以為「物理 unlock 進/退觸發全拒絕」(寫進這份 plan 的執行進度),其實**很可能**真正的觸發是某個地方多叫了一次 EnableRobot,而非 unlock 本身。前一次 Phase 2b 13 點採集 unlock 用了很多次都沒事,反證 unlock alone 不是元兇。需要日後做一次受控實驗(只按 unlock 不下任何 dashboard 指令)才能洗清 unlock 的嫌疑。

**2026-06-01 後續修正(finding 17 出現後)**:再評估這條 finding。用戶觀察到 reference fork 的 demo software 連續按 enable 都沒事 + 不需 pre-check,跟這條 finding 的「雙 enable 必死」相矛盾。後續實機證據(finding 17)指向**真正的觸發是 workbench 漏開 30003 → dashboard interface 沒掛載**,而不是雙 enable。pre-check 暫保留無害,但「雙 enable 必死」的因果鏈待 finding 17 修法上線後重新驗證:
- 若 workbench 修成三 port 後,連續兩次 enable 不再 -10000 → finding 16 假設**錯**,真正觸發是 3-port 缺
- 若仍會 -10000(但只限第二次)→ finding 16 真的存在,跟 finding 17 是兩條獨立 trap

### 17. **🚨 dashboard interface mount 要求三 port 全連,缺 30003 就全 -10000(2026-06-01 實機驗證)**

過去誤判為 finding 16 的隨機 -10000、需 power-cycle 才復原,真正元兇是 workbench 連線模型錯:**只開了 29999(dashboard)+ 30004(feedback),沒開 30003(move)**。控制器要求 client 連完三個埠才把 dashboard 指令解析器掛載;少一個 → 所有 dashboard 指令當「命令不存在」拒掉,回 -10000。

**實機證據**(兩支 ad-hoc raw-socket 對照):
```python
# outputs/test_dashboard_only_idle.py — 只開 29999
sock.connect((ip, 29999)); time.sleep(1); sock.sendall(b"RobotMode();")
# Reply: b'-10000,{},;RobotMode();'        ❌

# outputs/test_three_socket.py — 連 29999 + 30003 + 30004
dash.connect(29999); move.connect(30003); feed.connect(30004); time.sleep(1)
dash.sendall(b"RobotMode();")
# Reply: b'0,{4},RobotMode();'              ✓
```

**為什麼以前沒看到**:reference fork 的 `ui.py` / `PythonExample.py` / `main.py` 三個 demo 一開始就連三個埠,所以 demo 跑得通 → 我們以為「dashboard 直接送指令就行」。workbench v1 設計時把 30003 留到「之後做 motion 才開」→ 撞上這條沒文件化的握手要求。

**修法(commit `7bc1aa7`)**:`workbench.main_async` 啟動時也建立一條 `FramedConnection(move_port)` 包成 `MoveClient`,純粹維持 socket 開著,不送任何 motion 指令。`Workbench.__init__` 加可選 `move` 參數承接。連線失敗 log warning + 繼續(避免 workbench 起不來),但 dashboard 可能仍 -10000(這時就要看 log 找 move 連線到底成不成)。

**注意**:這不一定否定 finding 16(雙 enable trap)。可能兩條都存在(獨立 trap),也可能只有 17 是真的、16 的 -10000 從頭到尾都是 3-port 缺造成的。修完上手臂驗證後回頭改寫 finding 16。

### 18. **📜 線上 byte 格式以 demo 為準:送端帶 `;` 是 code-vs-doc 漂移(2026-06-01)**

**現象**:三埠 mount 成功(finding 17 修法上線)後實機跑 workbench `enable` → 仍回 `-10000,{},`。三埠連好、不是 finding 17、也沒雙 enable(finding 16),新症狀。

**對比審計發現**(`COMPARISON_REPORT.md`,worktree branch `worktree-comparison-report`):我們 `transport/connection.py:223` 在送指令時 `+ self._terminator`,送線是 `EnableRobot();`,**但 reference fork 的 `send_data` 從不加 `;`**(送 `EnableRobot()`)。CLAUDE.md「`;` 規則」段也明文「送不加 `;`」——也就是 **code 早就漂離 doc**,只是沒人盯。Finding 17 的 ad-hoc probe 剛好也帶 `;` 而通了,所以這個分歧躲過早期偵測。

**原則(寫進 CLAUDE.md 第一規則)**:**reference demo 是「真的能操控硬體」的證明,凡屬線上 byte 格式以 demo 為準**——demo 跑通了多年,我們是新人。發現我們 code 跟 demo 線上格式不一致,**先假設我們錯**,回頭對齊 demo,再回頭證明 demo 錯(要有實機證據)。

**修法(commit `3d623c1`)**:`connection.py:223` 去掉 `+ self._terminator` → 送 `b"EnableRobot()"`。Docstring 補對齊 demo 的說明。`test_framing.py` 對應 assertion 從 `b"EnableRobot();"` → `b"EnableRobot()"`。175 unit tests 仍全綠。

**收端 `_terminator` 不動**:`extract_frames` 仍按 `;` 切框(line 238);PR #9 雙 `;` 殘留清除也不受影響——那都是「收端」邏輯,協定真理。

**待實機確認**(尚未驗):Windows pull 後 workbench `mode` 應該回 `0,{...},RobotMode();` 而非 -10000。**如果仍 -10000 → `;` 不是元兇**,要回頭查 dashboard 模式設定(finding 11)/ 雙 enable 殘留(finding 16)/ 控制器 power-cycle。

**Lesson**:CLAUDE.md 的協定規則段是寫給未來的自己看的——code 漂離 doc 沒人發現,因為「能跑」沒人盯。下次新增一條 wire-format 規則時,在 PR 加一條 unit test 把「送出 byte 字面值」釘住,讓漂移在 review 時可見。

### 19. **🎯 J2/J3 coupling 是單一線性約束 `J3 − J2 ≤ 60°`(2026-06-01 T7B 確證)**

T7B 採點 5 個 J2 樣本（−10、−5、0、+5、+20）用「push J3 直到 controller alarm」協定。**4/4 alarm trigger 完全擬合單一線性約束**：

| J2 | last stable J3 | controller rejected J3 | **J3 − J2** |
|---|---|---|---|
| −10 | 49 | 50 | **60** |
| −5  | 54 | 55 | **60** |
| 0   | 59 | 60 | **60** |
| +5  | 64 | 65 | **60** |
| +20 | 77 | (per-axis cap 77.3 擋住) | 57（一致，無 alarm） |

**物理意義**：MG400 平行四連桿——當前臂跟後臂的相對開合角度 `J3 − J2 > 60°`，連桿撞到機械摺疊上限。原廠 SDK doc 沒記載這條約束。

**Deploy（commit 待補）**：
- `config/safety.json` 的 `j2_j3_coupling`:
  ```json
  [{"j2_coeff": -1.0, "j3_coeff": 1.0, "max_value": 59.95, "label": "j3_minus_j2_le_60"}]
  ```
  `max_value = 59.95` = 已知可達（J3-J2=59）跟 controller 拒絕（J3-J2=60）的中點，留 0.05° margin 給 feedback 浮動。Factory pose J3=59.903 < 59.95，accepted ✓。
- `joint_ranges_deg.J3` 上限從 77.3° 放回 spec **105°**（有 polygon 保護，per-axis 粗鎖可以拆）
- `_coupling_note.status` → `FITTED_BY_T7B`

**Deploy gate 三條 sanity 全綠**（離線跑）：
1. Factory pose (J=(-0.007, -0.021, 59.903, 2.681)) → polygon **approved** ✓
2. T7B 9 個採點 mark（5 stable + 4 alarm-tagged）→ **全 approved** ✓
3. Alarm 觸發 target（J3 = J2 + 60 at 4 個 J2）→ **全 rejected** ✓

**為什麼 T7A 的 piecewise envelope 演算法沒派上用場**：T7A 預期資料有 noise（operator 主觀邊界、masquerading z_floor 點），所以做分段 rising-flat 擬合。但 T7B「push 到 controller alarm」資料太乾淨——4 個 alarm trigger 完全在直線上——polygon 直接手寫一條更精準。**T7A 的演算法保留**，未來如果其他 robot 或新限制需要再用。

**Lesson**：實機驗證的 coupling 資料**比 SDK doc 還細**（doc 給 J2/J3 各自範圍但沒給相對關係）；只要採點協定對（直推到 alarm，不是 operator 心理界線），少量樣本就能釘住線性物理約束。

**沒採到的 J2 範圍**（待補）：
- `J2 ∈ {-15, -20}`：probe_start 卡 forbidden 起點（J3=46 起點 > J2+60 = 45 / 40 boundary）
- `J2 ∈ {+10, +15, +30, +40}`：per-axis cap 77.3° 卡住（本 deploy 後 cap 升到 105° 就沒這個問題）

下次 session 可選擇性補採 +30 / +40 驗證線性是否延伸到大 J2，沒採也不會有保護缺口（polygon 已涵蓋全範圍）。

### 20. **📋 官方 SDK PDF 全面審計 — 6 大類差異（2026-06-02）**

第三方 agent 對齊《TCP/IP 远程控制接口文档（4軸）_20240419》逐條審計，產出 `docs/OFFICIAL_VS_PROJECT_DIFF.md`。**權威優先序立規**：官方 PDF > demo wire format > 我們本地文件（CLAUDE.md 第一規則段升級）。

**A. 核心協定已對齊 ✅**：三埠、回應格式、`;` 規則（commit 3d623c1 修法後）、feedback 1440-byte magic、RobotMode 字典、SpeedFactor 值域、笛卡爾 `{x,y,z,r}`、指令大小寫不敏感、錯誤碼 -1/-10000/-20000/-3xxx/-4xxx。

**B. 缺口（按優先序）**：
- **B6**（最優先）：feedback 1440-byte 欄位**未逐欄對官方 offset 表**。實機 magic + FK 對齊 ⇒ 我們用到的欄位 offset 對；但官方 `ToolVectorActual` 是 6 分量（x,y,z,Rx,Ry,Rz），4 軸機型的 `r` 對應哪個分量待確認。建議做一次完整 dtype↔官方表核對 + unit test 釘住偏移。
- **B1**：`EnableRobot(load, cx, cy, cz)` 支援 0/1/4 簽名，我們只有 0。500g 偏心負載需要這個做動力學補償。
- **B2**：`MovL/MovJ(...)` 缺 `SpeedL/AccL/User/Tool/CP` 可選 kwargs。目前只能吃全局速率，無法逐指令客製。
- **B3**：`JointMovJ(...)` 缺 `SpeedJ/AccJ/CP`。
- **B4**：缺指令——`User/Tool/SetUser/SetTool/CalcUser/CalcTool`（座標系，待 Phase 3.2）、`PositiveSolution/InverseSolution`（控制器算 FK/IK 供交叉驗證）、`Arc/Circle/MovLIO/MovJIO`（待 Phase 5 motion）、`DO` 等 IO（待 controller phase）。
- **B5**（無影響）：feedback 30005（200ms）/ 30006（可配置）兩埠未用，目前 30004 足夠。

**C. 文件對齊**：
- C1：worktree `COMPARISON_REPORT.md`（已搬到 `docs/REFERENCE_AUDIT_2026-06-01.md`）的 `;` 段已過時——`;` 修法早就 ship 在 commit `3d623c1`。已加 stale-warning header 註明。
- C2：CLAUDE.md `;` 規則段現與官方一致，無需改。
- C3：findings 16/17 對 `-10000` 的「過載使用」註記跟官方「命令不存在」定義不衝突——標明這是我們實機補出的擴充。

**Lesson**：原廠 SDK PDF 是 contract，韌體是 Dobot 寫的——我們是 client。再強的實機觀察也不能凌駕 PDF（除非 PDF 漏記某條真實限制，如 finding 19 的 J3−J2 coupling）。CLAUDE.md 升級的三級權威排序就是把這條紀律寫進規範。

**Next action**：NEXT_TASKS 加 T13-T18 把 B 系列拆成可執行 task，按優先序排程。

### 21. **⚙️ EnableRobot load 參數實機驗證:phantom load → 馬達嗡聲;對應 load → J4 剛性增強(2026-06-02)**

H3 硬體驗證（PR #21 / B1 簽名）實測 `EnableRobot(load, cx, cy, cz)` 三變體：

**0 參數**（`enable`，裸法蘭）：✅ controller 接受、無動力學補償、安靜。

**1 參數，load=0.5kg、無實際負載**（`enable 0.5`）：
- controller 接受 → `Received: 0,{},EnableRobot(0.500000)`
- **馬達持續嗡聲**（1b + 2b：連續、enable 後一直響到 disable）
- 機制：宣告 0.5kg 重力 → 動力學補償多算 0.5kg → 馬達持續輸出對抗虛擬重力的小扭矩 → phantom load whine

**1 參數，load=0.75kg、有實際對應負載**（`enable 0.75` + 約 0.75kg 法蘭負載）：
- ✅ controller 接受、**無嗡聲**（補償對齊現實）
- ✅ **J4 剛性明顯變硬**（手感）——controller 預期有負載 → 抬高 J4 servo hold gain → 對抗預期的重力扭矩 → 維持位置的剛性增強

**4 參數**（`enable 0.5 0 0 30`）：**workbench parser bug**——`cmd_enable` 用 `args.split()[0]` 只取第一個值，後 3 個（cx, cy, cz）被丟掉。PR #21 builder/client 簽名沒問題，只是 workbench verb 包裝不完整。本 PR 修。

**這條 finding 證明了**：
1. **動力學補償是真的有效運作**——load 宣告直接改 servo 增益策略，不是僅當 metadata
2. **參數對齊現實很重要**——錯的 load 宣告造成 motor 持續 phantom-fight，長期不好（額外熱、磨耗）
3. **`r` 軸（J4）對 load 補償特別敏感**——可作為「load 宣告是否生效」的快速感官檢查

**對 phase 5/6 motion 的影響**：抓取場景必須**每換工件就 disable + EnableRobot(load, cx, cy, cz) 重設**正確負載 + 質心，否則 MovL/MovJ 動態軌跡計算會偏差（補償用錯重力導致 overshoot / 抖動）。Phase 6 controller `move_to(...)` API 之前要有 `set_payload(kg, cx, cy, cz)` 方法統一管理。

### 22. **🚧 韌體 1.7.0.0 不讓你「per-call 讀某 frame 下的 pose」(2026-06-02 H5)**

`docs/OFFICIAL_COORDINATE_SYSTEM_SPEC.md` 第 4.7 節 PDF 範例：
```
GetPose(User=0,Tool=0)     立即指令，可選 User/Tool 索引
GetPose(User=1,Tool=0)     →  回該 User=1 系下的 pose
```
**實機驗證該寫法不被 1.7.0.0 韌體支援**（TCP/二次開發 模式，公開 29999 通道）：

| 嘗試 | 結果 |
|---|---|
| `GetPose()` 無參數 | ✅ 回全局 active frame 下的 pose |
| `GetPose(User=1,Tool=0)` keyword 語法 | ❌ `error_id=-30001`（型別錯）|
| `GetPose(1,0)` 位置式 | ⚠ controller 接受、回 `0,{x,y,z,r}`，但**忽略 args** — 回的還是 base pose |
| `User(1)` on 29999 / 30003 | ✅ 兩個埠都回 `0,{},User(1)` 接受，但**只影響未來 motion 指令的 frame**，不影響後續 `GetPose()` 回傳 |
| DobotStudio UI 切 User=1 顯示 X=190 | ✅ 透過私有 22000 通道，public TCP 無等價接口 |

**Spec 4.1 重讀**：「User(index) 設定全局用戶坐標系。**運動指令未指定 User 時用此全局值**」——這條只說「給 motion」，沒承諾「給 query」。我們之前以為 User() 是全局切換、所有 query 跟著走，**是誤讀**。

**修法**（commit 待補）：
- `builders.get_pose()` 移除 user/tool 參數，留無參數版本 + docstring 警告
- `client.DashboardClient.get_pose()` 同步移除 kwargs，回 `PoseResult(user_index=None, tool_index=None)`
- 對應 unit test 更新

**對段 2 UI 的影響**（明確化、無架構變動）：
- UI 要顯示「pose 在 User=1 系下」**必須 client-side 算**
- 步驟：拿 `get_pose()` 回的 base pose → 套用 SetUser 時記住的 User=1 偏移（4×4 同質變換）→ 顯示
- 這正是段 2.1 `kinematics/transform.py` 的工作 —— 之前計劃就有，這條只是把「為什麼非要這個模組不可」釘死
- 副作用：client 端必須**自己記住 SetUser/SetTool 寫過什麼**（韌體沒 GetUser/GetTool 接口可讀回）→ 用 local cache（例如 `config/user_frames.json`）保存

**Lesson**：spec PDF 的範例可能描述「設計意圖」而非「實作現狀」。即使是官方 SDK doc，**對舊韌體版本要實機驗才能信**。1.7.0.0 是我們目前的真相。未來韌體升版可能補上 query-side User/Tool，到時再回頭測。

### 23. **🔄 DobotStudio Pro 校正 User/Tool 後，TCP/二次開發 模式可能被切走(2026-06-02 重現 finding 11 變種)**

H5 流程中需要開 DobotStudio Pro 校正 User=1 frame。校完關掉後，回 TCP 端跑 dashboard 指令全部 `-1` —— 跟 finding 11 一樣的「全拒絕模式」。回 DobotStudio 設定 → 遠程設置 → 重設 `TCP/二次開發模式` 才復原。

**機制推測**（未驗）：DobotStudio Pro 進入校正面板可能自動把控制器切到 DobotStudio-active 模式（私有 22000 通道控制權）→ 關掉 DobotStudio 後**控制器不會自動 revert** 回 TCP/二次開發模式 → 須手動再切回。

**SOP 修正**（finding 11 補強）：
- ✅ 「設定一次跨 power cycle 保留」**仍對**——指該模式設定本身持久
- ❌ 但「設過後 DobotStudio 不必開」**有例外**——**每次用 DobotStudio 做任何校正後，記得回設定→遠程設置確認 TCP/二次開發 還在**
- ✅ Workflow 建議：DobotStudio 校正 → **退出前先回設定面板把模式設回 TCP/二次開發** → 關 DobotStudio → 才繼續用 public TCP

未來 UI（段 2）可以加一個「校正模式 ↔ 控制模式」切換按鈕，封裝這個 toggle，避免 operator 忘記。

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

> **首次裝機(一次性,5 秒)**:DobotStudio Pro → **設定** → **遠程設置** → 改成 **`TCP/二次開發模式`**。
> 此設定**跨控制器電源週期保留**,設過一次就永久有效。後續 session 全程**不需要打開 DobotStudio**(詳見 finding 11)。

**啟動序列**(每次接手臂必做):
1. 控制器電源開啟,等 LED 穩定(30~60 秒)
2. 終端機跑 `python -m robot_core.scripts.workbench`
3. **檢視 log 三條 connected**:`Dashboard connected at .:29999`、`Move channel connected at .:30003`、`Feedback stream started at .:30004`。缺任何一條 → dashboard 會全 -10000(finding 17),先排線重試
4. `mg400>` 打 `enable`(綠燈、J1234 全使能)
5. **`status`,確認 Δ30004 < 0.1mm**——go/no-go 訊號

**進入拖曳示教**(採點 / 教學):
5. 物理按 unlock 鈕一次(B 型 latch,卡住「拖曳模式」位置)
6. 雙手都可以拉手臂

**採點循環**:
7. 拖到目標位置 → `status` 確認 `err=N` → `mark <label>`
8. 每 5~10 點 `save` 一次

**越界恢復**(會發生很多次):
9. 鬆開示教手
10. workbench `clear` → `enable`(若退使能)
11. workbench `status` 確認 `en=Y err=N`
12. 物理按 unlock 鈕重新進拖曳
13. 繼續

**收尾**(順序很重要):
14. workbench `save` 確保所有點存檔
15. 物理上**再按一次 unlock 鈕**跳出拖曳模式(漏了下次會莫名 -1)
16. workbench `disable`(退使能)
17. workbench `q`

⚠ **絕對禁止**:
- 通電中、未使能時按 unlock 鈕(會把控制器搞到「全拒絕模式」,只能 power cycle 救)
- 拖曳示教中送任何 dashboard / motion 指令
- 不戴護目鏡 / 不確認周圍淨空就 enable

---

**Phase 2b v1 完成 = 第一個真的能擋下違規動作的 safety 層上線。** 後面 Phase 5 motion 終於有底氣送 MovL 了。
