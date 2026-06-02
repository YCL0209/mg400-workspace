# 專案對比報告：`robot_core/` vs reference fork

> 角色：專案對比師 · 產出日期：2026-06-01 · **狀態：歷史紀錄（部分結論已過時，見下方標註）**
>
> 標的：本專案重寫實作 `robot_core/` ⟷ Dobot 原廠 `reference/TCP-IP-4Axis-Python/`
> 重點：CLAUDE.md 列的 **8 大反模式是否避開** + **協定事實是否忠實重現**，並依使用者指示
> **最優先盯緊 `;` 語法**。
>
> ---
>
> ### ⚠ 重要更新（2026-06-02 補註）
>
> 本報告當時的「送端帶 `;`」分歧**已在 commit `3d623c1` 修法上線**（[T7B PR #18](https://github.com/YCL0209/mg400-workspace/pull/18) 合入 main）。下方 §三送端段、§四建議修法表的「現況」欄、§五驗證方式都是當時的歷史快照——不再反映 main 的真實狀態。
>
> **想看當前真實狀況請參考**：
> - `docs/OFFICIAL_VS_PROJECT_DIFF.md` 的 §A 第 3 列「送指令不帶 `;` — ✅ 已對齊」
> - `PROGRESS.md` finding 18（送 `;` 修法的 lesson）
>
> 為何保留這份報告：當初 8 大反模式對照表（§一）仍有歷史 audit 價值；以及 finding 18 的觸發溯源（`;` 是怎麼被發現要修的）需要這份報告當證據。
>
> ---

---

## TL;DR

- **8 大反模式：全數避開 ✅**,各有對應的乾淨反制實作與測試。
- **協定事實：忠實重現 ✅**(三埠、1440-byte、magic number、回應格式、錯誤碼字典)。
- **`;` 語法**：**收端我們完勝 demo**(demo 不分幀,我們按 `;` 切框);**送端有一處與 demo
  不一致**——我們送線帶 `;`,demo 不帶。原則「線上格式對齊 demo」 → 建議送端改為不帶 `;`
  (詳見第三節 + 建議修法)。

---

## 一、8 大反模式對照(reference 證據 → 我們的反制實作)

| # | 反模式 (reference 證據) | 我們的反制 (檔案:行) | 判定 |
|---|---|---|---|
| 1 | 連線寫死 `__init__`、無重連 (`dobot_api.py:106-122`) | `TcpConnection` 連線在顯式 `connect()`,`max_retries`+指數退避 (`transport/connection.py:83-119`) | ✅ 避開 |
| 2 | `recv(1024)` 假設一收一則、不分幀 (`dobot_api.py:147` `wait_reply`) | 純函式 `extract_frames` 按 `;` 跨 recv 邊界重組 (`transport/framing.py:18-53`);`FramedConnection` 持緩衝 (`connection.py:226-244`) | ✅ 避開 |
| 3 | 手工拼字串+零驗證+4 個複製貼上 bug (`dobot_api.py:374-498`) | builders 純函式集中拼接+靜態驗證 (NaN/inf/範圍) (`protocol/builders.py:47-193`);bug 全未沿用 | ✅ 避開 |
| 4 | module 級全域可變狀態+單一 Lock (`main.py:8-12`) | `RobotState` 不可變快照+邊緣觸發訂閱、無全域、無鎖 (`state/robot_state.py:46-199`) | ✅ 避開 |
| 5 | busy-wait `while True: sleep(0.001)` (`main.py:39,59`;`ui.py:414`) | producer/consumer + `asyncio`,排空到最新幀 (`state/monitor.py:49-135`);事件通知非輪詢 | ✅ 避開 |
| 6 | 傳輸層 `import tkinter`、API 直操 widget (`dobot_api.py:3,128-132`) | transport/protocol/state 僅依 stdlib+下層,零 UI import;`logging` 不用 `print`(CLI 例外) | ✅ 避開 |
| 7 | 靠 `__del__` 釋放 socket (`dobot_api.py:175-176`) | context manager + 冪等 `close()`,docstring 明禁 `__del__` (`connection.py:19,121-128`) | ✅ 避開 |
| 8 | 純同步、無 async (全檔用 thread) | request-response 同步、feedback/state 串流 async (`feedback_stream.py`、`monitor.py`),分幀純函式可共用 | ✅ 避開 |

**結論：8 項反模式全數避開。** reference 的 4 個原檔 bug(`SetHoldRegs` if/else 反、
`ToolDI` 送成 `DI`、`DOGroup` 從不送、零範圍驗證)在我們的 builders 中均不存在。

---

## 二、協定事實忠實度

| 協定事實 | reference 出處 | 我們的實作 | 一致? |
|---|---|---|---|
| 三埠 29999 / 30003 / 30004 | `main.py:17-19` | `config/robot.json` ports 區塊 | ✅ |
| feedback 1440 bytes | `ui.py:419` | `FEEDBACK_FRAME_SIZE=1440` (`feedback.py:36`) | ✅ |
| magic `0x123456789abcdef` | `ui.py:429` | `TEST_VALUE_MAGIC` 驗框 (`feedback.py:37,186-199`) | ✅ |
| 回應格式 `ErrorID,{val},Func();` | `wait_reply` 原樣回傳 | `extract_frames` 切框後解析 | ✅ 我們更正確 |
| 錯誤碼字典 | `files/alarm_*.json` | (資料可搬 `config/`,程式不沿用) | ✅ |

---

## 三、🔎 重點盯緊：`;` 送/收不對稱

### 收端 — 我們完勝 reference
- reference `wait_reply` (`dobot_api.py:141-157`)：`recv(1024)` 整包 decode 回傳,**完全不把
  `;` 當終結符**,假設一收一則 → 雙 `;` 拒絕回應或多筆串接會錯亂。**這正是反模式 2。**
- 我們 `extract_frames` (`framing.py:44-52`)：`buffer.split(b";")`,最後一段當 remainder 跨
  recv 保留,正確按 `;` 切框。✅
- 韌體在 remote-unauthorized 會送**雙 `;`**(`b"-1,{},;EnableRobot();"`,PROGRESS finding 12),
  第 2 段是 echo 非下一筆回應。`FramedConnection.request()` 每次送前 `_pending.clear()` +
  `_rx_buffer=b""` 已修(`connection.py:217-222`,PR #9)。✅

### 送端 — ⚠ 與 demo 不一致(唯一需決議處)

| 來源 | 送線上是否帶 `;` | 證據 |
|---|---|---|
| reference `send_data` | **不帶** | `dobot_api.py:134-138` 送 raw string;`dobot_api.py` 無任何指令字串含 `;`;builders 產 `"RobotMode()"`、`"MovL({:f},...)"` |
| CLAUDE.md 硬體真理 | **不帶**(明文「送不加 `;`」) | CLAUDE.md「`;` 規則」段 |
| PROGRESS finding 17 實機探針 | **帶 `;`** → 回 `0,{4},RobotMode();` ✓ | `PROGRESS.md` L278-284(`sendall(b"RobotMode();")`) |
| **我們的 code** | **帶 `;`** | `connection.py:223` `message.encode()+self._terminator` |

**判讀**：demo 是「實際能操控硬體」的證明,凡屬**線上協定格式**就以 demo 為準。demo 送線
**不帶 `;`**;我們送端卻 append `;`,與 demo 不一致 →
**依「對齊 demo 線上準則」原則,送端應改為不帶 `;`。**

> **分層界線（重要）**：對齊的是 demo 的**線上 byte 格式**,不是它的**程式風格**。
> demo 收端用 `recv(1024)` 不分幀(反模式 2),我們**不跟**;控制器*回應*仍以 `;` 終結
> (協定事實),收端 `extract_frames` 按 `;` 切框**保留**。
>
> 補充:finding 17 實機證明帶 `;` 也能控制硬體,故這**不是功能 bug**;改動目的是統一走
> demo 已長年驗證的不帶 `;` 寫法,降低與真相來源的分歧。

---

## 四、建議修法（待核准,本報告尚未實作）

送端對齊 demo(不帶 `;`),收端不動。線上唯一分歧點在 transport 後加的 `;`;builders 已與
demo 一致(裸字串、`{:f}`/`{:d}` 格式相同),無需動。改動為外科手術級:

| 檔案:行 | 現況 | 改為 |
|---|---|---|
| `robot_core/transport/connection.py:223` | `self.send(message.encode("utf-8") + self._terminator)` | `self.send(message.encode("utf-8"))` |
| `connection.py:205-215`（`request` docstring) | 「message: 不含 `;`」 | 補一句:送線**不帶** `;`(對齊 demo);`_terminator` 僅供收端分幀 |
| `tests/test_framing.py:119` | `assertEqual(sock.sent, b"EnableRobot();")` | `assertEqual(sock.sent, b"EnableRobot()")` |

**保留不動**：`self._terminator` 仍用於收端 `extract_frames`(line 238);雙 `;` 拒絕回應的
`_pending.clear()` / `_rx_buffer=b""`(PR #9, line 217-222)不受影響(那是收端邏輯)。

**選用後續（非本次)**：同步修正 CLAUDE.md「`;` 規則」段與 PROGRESS,把「送不加 `;`」標為
實作已對齊 demo,並補一條 finding 記錄「finding 17 帶 `;` 也可、但統一走 demo 不帶 `;`」。

---

## 五、驗證方式（修法核准後）

1. **離線回歸**(必跑)：
   ```bash
   .venv/bin/python -m unittest discover -s tests
   ```
   重點 `tests/test_framing.py::FramedConnectionTests` —— 改後 `sock.sent` 應為
   `b"EnableRobot()"`;收端雙 `;` 邊界、跨 chunk 重組案例**不應變動**(全綠)。
2. **實機確認**(上手臂時)：
   ```bash
   python -m robot_core.scripts.connect_test     # 三埠全連 → enable → 讀狀態 → disable
   ```
   送出 `RobotMode()`(無 `;`)應回 `0,{...},RobotMode();`,非 `-10000`。demo 長年如此送,
   風險低;仍按紀律低速、隨時可急停。
