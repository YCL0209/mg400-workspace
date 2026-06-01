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

**SOP**(workbench 在手臂邊):
1. DobotStudio 確認模式為 `TCP/二次開發模式`(per finding 11)。
2. workbench `enable` → `status` 確認 `mode=5 en=Y err=N`、Δ30004 < 0.1mm。
3. 拖到**中等 z 高度**(避桌面 masquerading)。
4. **每個 J2 點**(grid 建議 J2 ∈ `{-20, -15, -10, -5, 0, +5, +10, +15, +20, +30, +40}`):
   - 拖到 (J1=0, J2=該值, J3=安全起點如 30, J4=0)、`status` 確認穩定
   - **1° 步進往上推 J3**(workbench 暫無 jog 指令 — 可用一次性 Python script,或事先在 workbench 加 `jog j3 +1`)
   - 每步後讀 `status`:`mode == 9` 或 `err=Y` → **立刻停**
   - **退回上一個穩定 J3**,`mark coup_j2_<value>`
   - `clear` → `enable`(T6 後 workbench 自己能做)→ 下一點
5. (可選)J3 下緣:同樣手法往負方向。
6. `save`。

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
