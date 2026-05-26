# MG400 AI Robotics Platform — Claude Code 專案記憶

> 目標：把 MG400 機械手臂打造成「可擴展的 AI 機器人軟體平台」，而非一次性控制腳本。
> 終局形態：transport → protocol → state → safety → (AI agent / Web / 視覺) 的事件驅動系統。

---

## 第一規則：`reference/` 是協定字典，不是架構範本

`reference/TCP-IP-4Axis-Python/` 是 Dobot 原廠程式碼的 fork。

- **唯讀。** 禁止修改、禁止把專案程式碼寫進去、禁止 `import` 它的任何模組。
- 它的**唯一用途**是查詢 TCP/IP 協定事實：埠號、指令字串格式、feedback 二進位結構、錯誤碼對照。
- 它的**程式風格是已知反面教材，嚴禁複製**。具體反模式：
  1. 連線寫死在 `__init__`，無法重連 / 重試
  2. `recv(1024)` 假設「一次收一則」，未做協定分幀（TCP 不保證訊息邊界）
  3. 指令全靠手工拼字串、零參數驗證（且原檔已含多個複製貼上 bug：`SetHoldRegs` if/else 寫反、`DOGroup` 從未送出指令、`ToolDI` 送成 `DI`）
  4. 大量 module 級全域可變狀態 + 單一 Lock
  5. busy-wait 輪詢（`while True: sleep(0.001)`）
  6. 傳輸層 `import tkinter`，分層被 GUI 污染
  7. 靠 `__del__` 釋放 socket，非確定性
  8. 純同步阻塞，無 async

只有**資料**可搬進本專案（如 `files/alarm_*.json` 錯誤碼字典 → `config/`）；**程式碼一律重寫，不得沿用。**

---

## 架構藍圖：嚴格分層，依賴只能由上往下

```
controller / api   對 AI agent、Web、視覺暴露乾淨介面
      |
   safety          所有運動指令的強制閘門（E-stop、工作範圍、奇異點禁區）
      |
   state           RobotState：訂閱 feedback、事件驅動，取代全域變數
      |
   protocol        指令拼接 + 參數驗證（集中化、型別安全）
      |
   transport       socket 收送 + 重連 + 協定分幀；不認識「機器人」概念
```

- 每一層只依賴它正下方那層，禁止跨層或反向依賴。
- `transport` 對「誰在用它」一無所知，不得 import 上層或任何 UI；用 `logging`，不要 `print`。

---

## 編碼 do / don't

- **DO** 用 context manager 或顯式 `close()` 管理 socket。**DON'T** 靠 `__del__`。
- **DO** 在 transport 持續讀進緩衝、依結束符 `;` 切出完整訊息。**DON'T** 假設一次 `recv` = 一則回應。
- **DO** 把指令拼接集中在 protocol，下手臂前驗證參數與範圍。**DON'T** 在各處手寫指令字串。
- **DO** 用 `RobotState` 封裝狀態、用 `asyncio.Event` 或條件變數通知。**DON'T** 用全域變數 + busy-wait。
- **DO** 區分「指令入列」與「動作完成」——是兩個不同事件。**DON'T** 用 `sleep` 猜動作有沒有做完。
- 並行模型（重要，別一刀切）：
  - **feedback / state 串流、controller 頂層 → async**（持續推送、高併發，是 async 真正的用途）。
  - **dashboard / move 的 request–response → 可用同步**（本質循序，async 在此買不到並行好處）。
  - **DO** 把分幀與解析寫成 I/O-agnostic 純函式（如 `extract_frames(bytes)`、`parse_feedback(bytes)`），讓同步↔async 的切換是「薄 wrapper 替換」而非邏輯重寫。
  - **DON'T** 讓同步阻塞的 socket 呼叫被 async code 直接呼叫（會卡死 event loop）；需要時用 `run_in_executor` 或換成 async wrapper。

---

## 安全規則（優先於一切功能）

- `EmergencyStop()` 走**獨立高優先通道**，可插隊，不得與一般指令排同一佇列。
- **任何**運動指令送出前，一律先過 safety 層驗證：使能狀態、工作範圍、奇異點禁區。
- 可達區為環形（annulus）：避開中心柱奇異禁區、外緣伸展奇異、以及 J1 後方 40° 死角。
- 開發測試一律**低速**且隨時可急停。Phase 0（連線→上電→讀狀態→安全下電→急停）未過關前，**不下任何 `MovL`。**

---

## 硬體事實（協定真理來源）

- 三個 TCP 埠：`29999` dashboard（控制 / 設定指令）、`30003` move（運動佇列）、`30004` feed（持續推送 1440-byte 二進位狀態）。
- feedback 封包以 `test_value == 0x123456789abcdef` 做 magic-number 驗框。
- 規格：4 軸，最大伸距 440mm，重複精度 ±0.05mm，額定負載 500g（最大 750g），質心偏心 ≤ 40mm。
- 關節範圍：J1 ±160°、J2 −25°~85°、J3 −25°~105°、J4 −180°~180°。各軸最大速度 300°/s。
- 「等動作完成」原廠有兩條路：阻塞式 `Sync()`、或輪詢 `30004` 比對座標。本專案目標走第三條：訂閱 feedback + 事件通知。

---

## 建置與測試（請依實際環境補上真實指令）

```bash
# 單元測試（離線，stdlib unittest）：分幀 / 框架請求-回應 / feedback 驗框解析
python -m unittest discover -s tests       # 無 numpy 時 feedback 測試自動 skip

# Phase 0 實機連線測試（連上→上電→讀狀態→安全下電；無任何運動指令）
python -m robot_core.scripts.connect_test
MG400_IP=192.168.1.20 python -m robot_core.scripts.connect_test   # 覆寫 IP
```

- 依賴：`numpy`（見 `requirements.txt`）。**只有** `robot_core/transport/feedback.py`
  在「解析一幀」時才 import numpy；其餘層級（分幀、socket）無此依賴。
- 連線設定集中在 `config/robot.json`（IP 預設 `192.168.1.6`/LAN1、三埠號、transport
  逾時與重試）。可用 `MG400_IP` / `MG400_{DASHBOARD,MOVE,FEEDBACK}_PORT` 環境變數覆寫，
  不寫死在程式碼。

---

## 開發紀律

- 先寫乾淨介面與測試，再接實機。
- 任何「對硬體下指令」的程式，預設低速 + 可中止。
- 不確定協定細節時，查 `reference/`，不要臆測。
- 本檔請保持精簡（200 行以內）；更細的層級規則放各子目錄的 `CLAUDE.md`（例如 `safety/CLAUDE.md`）。
