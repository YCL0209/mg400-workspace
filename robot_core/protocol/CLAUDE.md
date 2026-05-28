# protocol 層規則（子層，補充根 CLAUDE.md）

把「參數 → 確切指令字串」、送出前靜態驗證、回應解析集中在這層（型別安全、單一真相）。

## 依賴與分層
- **只依賴 transport**（`FramedConnection`、`extract_frames`）。**禁止** import state / safety /
  kinematics / controller / 任何 UI；不得反向依賴。
- 純核心 + 薄 I/O wrapper：`builders`/`responses` 是純函式（零 I/O，離線可測）；`client` 才碰連線。
- 用 `logging`，不要 `print`。

## 第一鐵則：協定事實查 reference/，不臆造
- 指令字串格式、埠別、回應結構一律以 `reference/` 為準；查不到就標 `TODO` 問人，別自己編。
- **不得複製** reference 的已知 bug（`SetHoldRegs` if/else 反、`DOGroup` 沒送、`ToolDI` 送成 `DI`）。
- 送出指令**不加** `;`；`;` 只在回應。回應開頭整數 = ErrorID（0=OK），結構 `ErrorID,{value},Func();`。

## 職責邊界（別越界）
- 只做**靜態**驗證：型別、參數個數、基本範圍（SpeedFactor 1–100、關節理論範圍）。
- **不做**「當下能否安全執行」：使能狀態、工作範圍可達性、J2/J3 耦合限制、E-stop 搶占——那是 Phase 4 safety。
- 笛卡爾 `MovL/MovJ` 不驗範圍（可達性屬 safety/kinematics）。

## E-stop 通道分離（釘死）
- `emergency_stop()` 是 **dashboard(29999)** 指令，可插隊，**絕不**排進 move(30003) 佇列。
- 只在 `DashboardClient` 暴露；實際高優先搶占 plumbing 留 controller（Phase 6）。

## 通道
- Dashboard(29999)：控制/查詢 + EmergencyStop。Move(30003)：MovL/MovJ/JointMovJ（回應僅入列 ack，非動作完成）。
- 「動作完成」要訂閱 feedback（state 層），不要看入列 ack，也不要 sleep 猜。
