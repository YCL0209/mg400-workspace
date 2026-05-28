# safety 層規則（子層，補充根 CLAUDE.md）

所有運動指令送出前的強制閘門。藍圖：controller 之下、state 之上。

## 依賴與分層
- **依賴 kinematics（FK/IK）+ state 的 snapshot 型別**。**禁止** import protocol / transport。
- safety 只「判斷」，不「執行」：不開 socket、不送指令、不拼指令字串。
- snapshot 以 duck-type 取用（`is_enabled` / `has_error` / `joints`），型別僅供註解（TYPE_CHECKING），
  runtime 不 import state/transport stack。純函式、零 I/O（除讀 `config/safety.json`）。
- 用 `logging`，不要 `print`。

## 閘門（對一個運動目標 pose）
依序：未 enable → 駁回；有 active error → 駁回；IK 無解（不可達）→ 駁回；
出工作範圍（環形內外徑 / z 上下限 / J1 後方死角）→ 駁回；
選「最接近當前關節」的 IK 解（決定性），該解出關節範圍或違反 J2/J3 耦合 → 駁回；否則核准、回 chosen_joints。
- 駁回**不丟例外**，回 `SafetyDecision(approved, code, reason, chosen_joints)` 讓 controller 處置。

## E-stop 不受閘門管
- `EmergencyStop` / `ClearError` / `DisableRobot` / `ResetRobot` 永遠允許、可插隊，safety **不得擋**
  （`ALWAYS_ALLOWED_CONTROL`）。閘門只管「運動指令」。實際高優先搶占 plumbing 在 controller（Phase 6）。

## 參數來源
- `config/safety.json` 是**placeholder**（環形內外徑、z 上下限、J1 死角、J2/J3 耦合）；
  provenance 標明「待 Phase 2b 實測替換」。先寫邏輯、後校參數（同 kinematics 紀律）。
- J2/J3 耦合以線性半平面 `j2c*J2 + j3c*J3 <= max` 列表表示；目前預設空（只驗 per-axis 範圍），Phase 2b 補多邊形。
