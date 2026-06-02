# Next Tasks — Phase 2b v1 完成後路線

> 截至 2026-05-29:Phase 2b v1 採集完畢、safety bounds v1 落地。下面是按優先順序的待辦,**離線可寫的擺前面**,要手臂的擺後面。

---

## 🟢 P0:離線可做、下次接手臂前要先就位

### T1. safety.json v1 寫入 config/(2 分鐘)

把 `outputs/safety_v1.json` 內容寫到 `config/safety.json`(覆蓋現有 placeholder)。

```bash
git checkout -b phase-2b-safety-v1
cp outputs/safety_v1.json config/safety.json
# review diff
git diff config/safety.json
# 跑 safety 既有測試確認沒打破
python -m unittest discover -s tests -v
git add config/safety.json
git commit -m "Phase 2b v1: write empirical safety bounds (annulus/z/joints), coupling pending v2"
git push -u origin phase-2b-safety-v1
```

PR #9 標題建議:`Phase 2b v1: safety bounds from real arm calibration`

⚠ **驗收**:既有 safety gate 測試應該全綠(只是 bounds 數值換了,行為沒變)。如果某些測試硬編碼了 placeholder 數值,把它們改成讀新 safety.json 而不是改測試斷言。

---

### T2. transport framing fix(1 行修法 + 補測試,30 分鐘)

**Bug**:`robot_core/transport/connection.py` 的 `_read_frame` 在 firmware 送多個 `;` 分段時,把過讀的 frame 留在 `_pending` queue。下次 `request()` 不送指令直接拿 `_pending` 的第一個,結果**所有 request 都錯位**。

**修法**(在 `request()` 開頭加 2 行):

```python
def request(self, message: str, *, timeout_s: Optional[float] = None) -> str:
    sock = self._require_socket()
    # 排乾上一次 request 過讀的殘留(這支 firmware 在拒絕回應時會
    # 送兩個 ;-結尾的 frame,第二個是 echo,不是給下一次 request 用的)
    self._pending.clear()
    self._rx_buffer = b""
    self.send(message.encode("utf-8") + self._terminator)
    return self._read_frame(sock, timeout_s)
```

**測試 `tests/test_connection.py` 補一個情境**:
- 模擬一個 mock socket 在 `recv()` 連續吐 `b'-1,{},;EnableRobot();'`(兩個 `;`)
- 確認第一次 `request('EnableRobot()')` 拿到 `-1,{},`(第一個 frame)
- 確認第二次 `request('RobotMode()')` 不會拿到 `'EnableRobot()'`(殘留)而是真的等下一次 socket 回應
- 也順便驗 single-`;` 標準回應行為不變

提示詞給 Claude Code:

> 修 `robot_core/transport/connection.py` 的 `request()`,在開頭清掉 `self._pending` 跟 `self._rx_buffer`,排除上一次過讀的殘留。背景:MG400 firmware 在拒絕指令的回應會送兩個 `;`-結尾片段(`-1,{},;EnableRobot();`),我們的 framer 按單 `;` 切會產生 2 個 frame,第二個被留在 `_pending` 給下一次 request 拿,造成所有後續 request 錯位。
>
> 同時在 `tests/test_connection.py` 補測試:用 mock socket 模擬連續吐多個 `;`-結尾 frame 的情境,驗證 `request()` 每次拿到的是「**這次** request 真正的第一個回應」,不是上一次的尾巴。
>
> 不要動 framing.py(它按單 `;` 切是符合 SDK doc 規範的,不是 bug 的根源)。

---

### T3. .gitignore 加 limits_*.json 例外(1 分鐘)

`outputs/` 在 .gitignore 是對的(中間檔不入庫),但 `limits_*.json` 是採集真理,應該例外:

```
# .gitignore 加這行(在 outputs/ 那行下面)
!outputs/limits_*.json
```

之後 `git add outputs/limits_xxxx.json` 不需要 `-f` 了。

---

### T4. PROGRESS.md 入 repo(2 分鐘)

把 `outputs/PROGRESS.md` 覆蓋掉現有的:

```bash
cp outputs/PROGRESS.md PROGRESS.md
git add PROGRESS.md
git commit -m "Update PROGRESS.md after Phase 2b v1 hardware session"
```

可跟 T1 / T3 一起一個 PR 推。

---

## 🟡 P1:Phase 3.1 protocol 補完(離線,~3-4 小時)

### T5. 加 9 個 builder + responses + DashboardClient 方法

照 SDK doc 第 7-33 頁的規格,在 `robot_core/protocol/builders.py` 加:

| 指令 | 簽名 | 回應結構 | 何時用 |
|---|---|---|---|
| `ResetRobot()` | 無參 | `0,{},ResetRobot();` | 停手臂+清隊列 |
| `Continue()` | 無參 | `0,{},Continue();` | ClearError 之後恢復隊列必要 |
| `StartDrag()` | 無參 | `0,{},StartDrag();` | 軟體進拖曳模式(替代 unlock 鈕) |
| `StopDrag()` | 無參 | `0,{},StopDrag();` | 軟體出拖曳模式 |
| `EmergencyStop()` | 無參 | `0,{},EmergencyStop();` | 軟體急停 |
| `GetErrorID()` | 無參 | `0,{[[id,..],[id],[id],[id],[id],[id],[id]]},GetErrorID();` | 詳細錯誤碼,要 parser |
| `Sync()` | 無參 | `0,{},Sync();` | block 直到隊列清完(Phase 5 motion 必用) |
| `GetAngle()` | 無參 | `0,{J1,J2,J3,J4},GetAngle();` | 同步查詢關節 |
| `GetPose(User=0,Tool=0)` | 可選 User/Tool | `0,{X,Y,Z,R},GetPose();` | 同步查詢笛卡爾 |

**注意 `GetErrorID()` 的 parser 比較特殊**:回應裡的 value 是巢狀 list `[[控制器], [servo1], [servo2], [servo3], [servo4]]`(只用前 5 個,4 軸不用 servo5/6)。要寫專屬 parser,不能套通用 `{...}` 邏輯。

**測試**:每個指令一組 unit test,涵蓋
- 成功路徑(`0,{},...;` parse 正確)
- 錯誤路徑(`-1,{},...;` 回 `error_id=-1` 不丟例外)
- builder 產出字串對齊 doc 規範(`StartDrag()`、`Sync()` 等)

提示詞給 Claude Code:

> 根據 `reference/TCP_IP遠程控制接口文檔_4軸_20240419_cn.pdf` 第 7-33 頁,在 `robot_core/protocol/builders.py` 加 9 個指令的 builder:`ResetRobot`、`Continue`、`StartDrag`、`StopDrag`、`EmergencyStop`、`GetErrorID`、`Sync`、`GetAngle`、`GetPose`。
>
> 規範:
> - builder 是純函式,輸入 Python 參數、輸出 bytes(命令字串 + 不含 `;` 因為 transport 會加)
> - 簽名對齊 doc 規範(`GetPose(User=0, Tool=0)` 兩個可選關鍵字參數)
> - 加靜態 validation(`GetPose` 的 User/Tool 必須是 int)
> - `GetErrorID()` 需要在 `responses.py` 加專屬 parser(回應的 value 是巢狀 list,不是單一值或單一字典)
>
> 然後在 `DashboardClient` 加對應方法,呼叫 builder + parse 回應 + 回 strongly typed dataclass(例如 `GetErrorIDResult` 含 `controller_errors: list[int]`, `servo_errors: list[list[int]]`)。
>
> 測試:每個指令 happy path + error path + 字串對齊 doc 範例。
>
> 注意 `Continue()` 跟 `Pause()` 不一樣(Continue 是 Pause 的對應,也是 ClearError 之後恢復隊列必要的後續步驟)。

---

## 🟠 P2:要手臂的 session(優先順序內排)

### T6. Phase 3.2 enable 授權 ✅(2026-06-01,無程式改動)

**結論**:控制器設定 `遠程設置 → TCP/二次開發模式` 是 29999/30003 外部控制的前置開關。不在這模式 → 所有 dashboard 指令一律 -1;切過去後 workbench 自行 enable,**跨控制器電源週期保留**,DobotStudio runtime 不必開。

**設定位置**:DobotStudio Pro → 設定 → 遠程設置 → 改 `TCP/二次開發模式`。

過程中順手排除的假設(留給未來省事):29999 換 30003 送 EnableRobot 一樣 -1(授權閘門是整機的,非 per-port);DobotStudio 私有 22000 通道**不是**授權來源(它根本不送 29999),授權純粹是這個模式設定的事。

詳見 PROGRESS finding 11(已解)。

---

### T7A. calibrate_bounds 演算法升級 ✅ #12(2026-06-01)

`select_coupling_points`(label-based)+ `detect_z_floor` + `filter_masquerading_points`(J2 cutoff 50° + z proximity 30mm)+ `fit_piecewise_envelope`(2-segment rising → flat)。`derive_joint_ranges` 自動 spec-fallback;`derive_j1_dead_zone` 去 cap;`compute_workspace_limits` 修為用觀察 extremes。v1.5 config 部署完成,**coupling polygon 故意留空**待 T7B。

**重要 lesson(別重蹈)**:v1 採的 coupling 點是 operator 主觀邊界(看快到就停),不是 controller 真 alarm 邊界。用這資料擬合會把 J3 ≤ 52 約束在 J2=0,**直接擋掉 factory pose J3≈60**(known-safe)。Deploy 會把 Phase 5 motion 全部鎖死。所以 T7B 採點協定必須改。

---

### T7B. 重採 coupling 點 + 部署 polygon(~1 小時要手臂)

**採點協定:從「operator judgment」改成「push 到 controller 真 alarm 才停」。**

**前置(每次 session 開始,都要過才動手)**:
- 控制器設定為 `TCP/二次開發模式`(finding 11)。
- workbench 啟動後 log 三條都要看到 "connected":`Dashboard connected at .:29999`、`Move channel connected at .:30003`、`Feedback stream started at .:30004`。少任何一條 → dashboard 指令會全 -10000(finding 17),先排線、重試,不要直接開始採點。
- 送指令一律**不加** `;`(會被算進 frame terminator,雙 `;` 觸發 framer 切錯);收回應由 transport 框,別自己拼。

**SOP**(workbench 在手臂邊):
1. workbench `status` 確認 `mode=5 en=Y err=N`、Δ30004 < 0.1mm。
2. 拖到**中等 z 高度**(避桌面 masquerading)。
3. **每個 J2 點**(grid 建議 J2 ∈ `{-20, -15, -10, -5, 0, +5, +10, +15, +20, +30, +40}`):
   - 拖到 (J1=0, J2=該值, J3=安全起點如 30, J4=0)、`status` 確認穩定
   - **1° 步進往上推 J3**(workbench 暫無 jog 指令 — 可用一次性 Python script,或事先在 workbench 加 `jog j3 +1`)
   - 每步後讀 `status`:`mode == 9` 或 `err=Y` → **立刻停**
   - **退回上一個穩定 J3**,`mark coup_j2_<value>`
   - `clear` → `enable`(T6 後 workbench 自己能做)→ 下一點
4. (可選)J3 下緣:同樣手法往負方向。
5. `save`。

**Deploy gate(必過才能 PR 寫 config)**:
```
.venv/bin/python -m robot_core.safety.calibrate_bounds outputs/limits_<T7B>.json
# 拿到新 polygon 後,sanity-test 必須兩條都過:
#   ✓ factory pose FK(0,0,60,0) → approved (Phase 5 起手不會被鎖)
#   ✓ 採點中每個「alarm 前最後一筆穩定 J3」→ approved (在邊界內側)
#   ✗ 採點中每個「alarm 觸發那一筆」→ rejected (邊界外側)
```

三條 sanity 全過,**才能** PR 把 polygon 寫進 `config/safety.json`(`j2_j3_coupling: []` → 兩條或更多 CouplingConstraint;`_coupling_note.status: DEFERRED_TO_T7B` → `FITTED_BY_T7B`;`joint_ranges_deg.J3` 上限可從手動 77.3° 改回 spec 105°,因為現在有 polygon 保護)。

任何 sanity 沒過 → **不 deploy**,重新檢視 polygon 演算法或採點協定。

---

### T8. Phase 5 motion 原語(~3-4 小時,需 T6 / T7 先穩)

加 `MovJ` / `MovL` 的 motion client + safety gate 整合 + 事件驅動到位:

```python
async def move_l(target: Pose) -> None:
    # 1. IK 求解
    joint_solutions = inverse_kinematics(target, config)
    # 2. safety gate 檢查(每個解)
    decision = gate.evaluate(target, state.snapshot(), bounds)
    if not decision.allowed: raise SafetyViolation(decision)
    # 3. 送 MovL via 30003
    client.movl(target)
    # 4. 事件驅動到位(訂閱 RobotState,等 robot_mode 從 7 RUNNING 回到 5 ENABLE)
    await state.wait_for(lambda s: s.robot_mode == 5, timeout=10.0)
```

⚠ **不要用 sleep 等到位**,用 `RobotState` 事件訂閱。Phase 1 寫的 deadband + edge-triggered callback 就是給這用的。

⚠ **送 MovL 前一定要 `Sync()` 等前序隊列清空**(否則立即指令會跟運動指令搶執行序)。

---

### T9. demo 切片(~30 分鐘,要手臂)

驗證整條串通:寫死兩個安全座標(例如 factory pose + 偏前方 50mm),`move_to(pose_a)` → `move_to(pose_b)` → 回 pose_a,跑 5 次無 alarm。

這是「Phase 5 完整收尾」訊號——之後才能放心做 Phase 6 controller。

---

## 🔴 P3:Phase 6+(後續)

### T10. Controller(狀態機 + 任務佇列 + TCP/tool offset)

詳見 PROGRESS.md「workbench」段——controller 出來後 workbench 變 thin CLI over controller。

### T11. Phase 9/10(視覺 + 手眼校正)

里程碑二範圍。Phase 5/6 穩了之後再展開。

---

## 🆕 P2.5:官方 SDK 審計補齊(2026-06-02 加入,源自 `docs/OFFICIAL_VS_PROJECT_DIFF.md`)

權威 = 官方 PDF。下列 task 按 DIFF 自己的優先序排;依賴 Phase 5/6 進度決定何時動。**不現在做**——T8/T9 motion 原語先穩了再回頭補。

### T13. B6:feedback 1440-byte 欄位逐欄對官方表(離線,~2 小時)

**Why**:現在實機 magic + FK 對齊 ⇒ 我們用的欄位 offset **是對的**,但**整個 1440 byte 沒逐欄核**過官方表(PDF 行 2836-3700 給精確 byte offset)。特別:官方 `ToolVectorActual` 是 **6 分量** (X,Y,Z,Rx,Ry,Rz),4 軸機型的 `r` 對應哪個分量我們**沒交叉驗證**過(workbench Δ 只比對 x,y,z 三軸)。

**Scope**:
- 對 `robot_core/transport/feedback.py` 的 numpy dtype 跟 PDF 表逐欄核對(reserved/padding 也要對)
- 釘死 4 軸 `r` 來自哪個分量 + 加 unit test 寫死該欄位 offset
- 對 RobotMode、EnableStatus、ErrorStatus、QActual、ToolVectorActual 等核心欄位寫 unit test 釘住 byte offset
- 若發現我們解析錯的欄位:修 + 加 finding

**Verify**:unittest 全綠 + workbench `live` 看到的 `r` 跟 30004 報的對應分量一致(差 < 0.1°)

**順序**:T8 motion 原語上線前**強烈建議**做完——若 motion 命令吃 r 但我們讀的是錯欄位,Phase 5 demo 會神祕失敗。

### T14. B1:`EnableRobot(load, cx, cy, cz)` 支援 0/1/4 簽名(離線,~30 分鐘)

**Why**:官方 `EnableRobot(load, centerX, centerY, centerZ)` 支援 0 / 1 / 4 參數;我們只有 0。500g + 偏心 ≤40mm 抓取場景需要動力學補償。

**Scope**:
- `builders.py` `enable_robot()` → `enable_robot(load=None, cx=None, cy=None, cz=None)`,套官方值域(load double kg、cx/cy/cz double -500~500 mm)
- `client.py` `DashboardClient.enable_robot()` 同步
- workbench `cmd_enable()` 可選帶 load(`enable 0.5` 或 `enable 0.5 0 0 30` 之類)
- Test:happy 0/1/4 簽名 + 各值域邊界

**順序**:Phase 5 前;或者抓取場景出現再做。

### T15. B2 + B3:`MovL/MovJ/JointMovJ` 補可選 kwargs(離線,~1 小時)

**Why**:官方 `MovL(X,Y,Z,R, User=, Tool=, SpeedL=, AccL=, CP=)`、`MovJ` 同(`SpeedJ`/`AccJ`)、`JointMovJ(J1,J2,J3,J4, SpeedJ=, AccJ=, CP=)`。我們只有 4 位置參數。

**Scope**:
- builders 三個 mov_* 函式加 optional kwargs,validate + 拼成完整字串
- client 同步
- workbench `probe_start`/`jog` 可選帶 speed(替代 SpeedFactor 全局)
- Tests:有沒有 kwargs 都通

**順序**:T8 motion 原語起跑時順手做,讓 Phase 5 demo 可以逐指令客製。

### T16. Phase 3.2 / B4-coords:8 條座標系指令(離線,~3-4 小時)

**Why**:`docs/OFFICIAL_COORDINATE_SYSTEM_SPEC.md` 已備好。10 條官方座標系指令只實 2 條(GetPose、GetAngle)。**UI 座標圖開發前必須補完**。

**Scope**(8 條):
- `User(index)` / `Tool(index)` — 隊列,設全局
- `SetUser(index, table)` / `SetTool(index, table)` — 立即,寫入指定槽
- `CalcUser(index, dir, table)` / `CalcTool(index, dir, table)` — 立即,算變換後值
- `PositiveSolution(J1..J4, User, Tool)` / `InverseSolution(X,Y,Z,R,..., isJointNear, JointNear)` — 立即,控制器算 FK/IK
- 升級 `PoseResult` 帶 `user_index` / `tool_index` 欄位(否則 UI 拿到 pose 不知道是哪個系下)
- Tests:每個指令 happy + 邊界

**順序**:Phase 5 motion 收尾後、Phase 6 controller 之前。Phase 6 controller 的 TCP/Tool offset 架構會基於這 8 條。

**前置**:T13(feedback `r` 釘死)、T15(mov_* User/Tool 參數)——T16 補的 User()/Tool() 指令會跟 motion 指令的 User/Tool 參數互動。

### T17. B4-剩餘:Arc / Circle / MovLIO / MovJIO / DO 等(離線,~大,分批做)

**Why**:官方有,我們無。`Arc` `Circle` 是 Phase 5/6 motion 需要;`MovLIO` `MovJIO` 是動中觸發 IO;`DO` 等是 controller phase 的 IO 控制。

**Scope**:依需求逐條補,不一次補完。每條照 T15/T16 的 pattern(builder + client + test)。

**順序**:Phase 5 demo 跑通後,看哪些指令實際需要再補。**避免 YAGNI**。

### T18. B5 註記:30005 / 30006 feedback 埠(無動作,僅記錄)

官方除 30004 外另有 30005(200ms)、30006(可配置)。**目前 30004 已足夠**,記錄供未來低頻需求參考(例如 UI 不需要 125 Hz、用 30005 5 Hz 就好,可省 CPU)。

**不排 task**,只在 PROGRESS finding 20 留註記。

---

## 🚨 下次硬體 session checklist(源自 2026-06-02 merge 的 4 個 PR，全部離線測試綠但實機未跑)

照「最便宜安全 → 較進階」順序，開機 + enable 後依序跑。每項通過再進下一項。

### H1. PR #20 (B6) — feedback `r` 欄位對位 ⭐ 先做

**改了什麼**:`FeedbackFrame.pose.r` 釘死 = `tool_vector_actual[3]`(6 分量中的 Rx)。離線靠 demo + PDF 推斷釘住。

**驗法**:
```
mg400> live
```
然後手動轉 J1 或 J4(unlock 拖曳 / DobotStudio jog)。觀察 workbench 印的 `r`:

- ✅ `r` 跟 30004 報的同步變、跟 `r = J1 + J4`(finding 1)一致 → pin 對
- ❌ `r` 不動或方向反 → 試 `tool_vector_actual[4]`(Ry)或 `[5]`(Rz),改 `robot_core/transport/feedback.py`,重啟 workbench 再驗

**最便宜的測試**(無運動指令,純讀)。

### H2. PR #22 (Phase 3.2) — FK/IK 控制器交叉驗 ⭐ 同樣便宜

**改了什麼**:加 8 個座標系指令含 `PositiveSolution`(控制器算 FK)/ `InverseSolution`(控制器算 IK)。

**驗法**(ad-hoc Python,要手臂 enable 但**不會動**):
```python
from robot_core.transport import FramedConnection
from robot_core.protocol import DashboardClient
from robot_core.kinematics import forward_kinematics, inverse_kinematics

conn = FramedConnection("192.168.1.6", 29999); conn.connect()
d = DashboardClient(conn)

# (a) 控制器 FK vs 我們 FK
print("controller FK:", d.positive_solution(0, 0, 60, 0, user=0, tool=0))
print("our FK:       ", forward_kinematics(0, 0, 60, 0))
# 預期差 < 0.1mm / 0.1°

# (b) 控制器 IK vs 我們 IK (用 factory pose)
print("controller IK:", d.inverse_solution(197.2, 0, -30.3, 0, user=0, tool=0))
print("our IK:       ", inverse_kinematics(197.2, 0, -30.3, 0))
# 控制器應回 J≈(0, 0, 60, 0)
```

- ✅ 兩組差 < 0.1mm / 0.1° → 我們的 FK/IK 模型(finding 2/3)補強為「控制器同意」
- ❌ 差 > 1mm / 1° → 某個 corner 偏掉,需要重新擬合 / 抓 bug

`get_pose(user=1, tool=2)` 也可順便試 — 若 User=1 / Tool=2 未校過,應回 `ErrorID=-1`(預期)。

### H3. PR #21 (B1) — `EnableRobot(load, cx, cy, cz)`

**改了什麼**:`enable` workbench verb 接受 0/1/4 參數簽名。

**驗法**(disable → 重 enable 三次):
```
mg400> disable
mg400> enable                    # 0 參數
mg400> disable
mg400> enable 0.5                # 1 參數,0.5kg 負載
mg400> disable
mg400> enable 0.5 0 0 30         # 4 參數,0.5kg + cz=30mm 偏心
```

- ✅ 三組都印 `Received: 0,{},EnableRobot(...);` → 動力學補償啟用
- ❌ 4 參數那組 `-30000` → `cx/cy/cz` 必須在 `[-500, 500]` mm(spec)

**注意**:你目前抓取場景如果有用治具,確認治具實際質心偏移代入 cx/cy/cz。**未來抓物**:每換工件就 disable → enable 4 參數重設。

### H4. PR #21 (B2+B3) — `MovL/MovJ/JointMovJ` 可選 kwargs

**改了什麼**:每個 mov_* 可選 `SpeedL/AccL/SpeedJ/AccJ/CP/User/Tool`。

**驗法**:**目前 workbench 沒 motion verb 暴露這些參數**——要等 T8 (Phase 5 motion 原語) 上線 verb 才好驗。短期可寫一次性 Python:
```python
from robot_core.protocol import MoveClient
from robot_core.transport import FramedConnection
conn = FramedConnection("192.168.1.6", 30003); conn.connect()
m = MoveClient(conn)
# 從目前位置往同位置 mov_j(零位移),帶 SpeedJ 測接受度
# (要先讀 status 確認當前 joints)
print(m.joint_mov_j(0, 0, 60, 0, speed_j=20, acc_j=20, cp=50))
```
- ✅ 回 `0,{},JointMovJ(...);` 且手臂沒亂動 → 參數接受
- ❌ `-30000` → kwargs 名字或值域不對,看回應字串對照 PDF 第 7-13 頁

**建議延後**:跟 T8 Phase 5 motion 一起做更省事。

### H5. PR #22 — `User()` / `Tool()` 切換指令(可選)

如果你之後要做 UI 座標圖、或用治具(Tool 系)、或多工作平面(User 系),驗:
```
mg400> # 先在 DobotStudio 標定 User=1 跟 Tool=1
mg400> # 然後 ad-hoc:
>>> d.user(1)   # 切到 User 系 1
>>> d.tool(1)   # 切到 Tool 系 1
>>> d.get_pose()  # 看 pose 在新系下的座標
```
這組沒標定就跳過——controller 會回 `-1` 表示索引未配置。

---

## 預估工時

- P0(T1-T4):**1 小時**(全離線)
- P1(T5):**3-4 小時**(全離線)
- P2(T6):**30 分鐘**(要手臂)
- P2(T7A 離線 + T7B 手臂):**2.5 小時**
- P2(T8):**3-4 小時**(要手臂)
- P2(T9):**30 分鐘**(要手臂)

**合計到 Phase 5 完整收尾:約 11 小時**,跨 3-4 次 session。

---

## 一個重要的提醒

`config/safety.json` v1 已經比 placeholder 安全很多,但 **coupling polygon 是空的**——意味著 Phase 5 第一次送 MovL 時,某些違反真實 coupling 的目標**會通過 safety 檢查、但被控制器 alarm 擋下**。

這不會撞硬體,但會讓 demo 切片(T9)看起來像「我們的 safety 有 bug」。**那不是 bug,是 v1 已知缺口**。等 T7 完成 coupling polygon 後,這層保護才會完整。

所以建議順序:**T1-T4 → T6(enable 授權)→ T7(coupling) → T8(motion) → T9(demo)**。把 coupling 處理在 motion 之前,demo 跑起來才會乾淨。
