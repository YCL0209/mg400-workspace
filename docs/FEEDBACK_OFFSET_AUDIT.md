# Feedback 1440-byte Offset 逐欄審計（B6）

**任務**：`docs/OFFICIAL_VS_PROJECT_DIFF.md` 的 B6 ——「feedback 欄位偏移逐欄對官方表 + 釘死 4 軸 r 的來源分量」。
**權威**：官方《TCP/IP 远程控制接口文档(4軸)_20240419》（控制器 1.7.0.0）offset 表（PDF feedback 段）。線上 byte 格式以 reference demo 為活證據（CLAUDE.md 權威序）。
**結論**：✅ 全 64 欄 offset 三方完全一致；4 軸 `r` = `ToolVectorActual[3]`。**無解析錯誤**，未改 dtype。

---

## 三方對照（我們 dtype ↔ reference `MyType` ↔ PDF 表）

`robot_core/transport/feedback.py` 的 numpy dtype 與 `reference/.../dobot_api.py` 的 `MyType` **逐欄一致**（僅 cosmetic：`m_actual` vs `m_actual[6]`）。兩者 offset 又與 PDF 官方表**逐欄相符**。核心欄位（已寫 unit test 釘死，見 `tests/test_feedback.py::FeedbackOffsetTests`）：

| 欄位 | PDF offset | 我們 dtype offset | 判定 |
|---|---|---|---|
| MessageSize / `len` | 0000 | 0 | ✅ |
| RobotMode / `robot_mode` | 0024 | 24 | ✅ |
| TestValue / `test_value` | 0048 | 48 | ✅ |
| QActual / `q_actual` | 0432 | 432 | ✅ |
| ToolVectorActual / `tool_vector_actual` | 0624 | 624 | ✅ |
| EnableStatus | 1026 | 1026 | ✅ |
| ErrorStatus | 1029 | 1029 | ✅ |
| Load / centerX/Y/Z | 1168/1176/1184/1192 | 同 | ✅ |
| TargetQuaternion / ActualQuaternion | 1352 / 1384 | 同 | ✅ |

全 64 欄（含保留位 padding）的 offset map 見 `EXPECTED_OFFSETS`（`tests/test_feedback.py`），測試逐欄 assert，未來任何 dtype 漂移即時被抓。

## 4 軸 `r` 的來源分量（釘死）

官方 `ToolVectorActual` 是 **6 doubles**，韌體布局為 `[X, Y, Z, Rx, Ry, Rz]`。4 軸 MG400 只有單一 yaw `R`：

- **demo 活證據**：`reference/.../ui.py::set_feed_joint` 把座標 label `["X","Y","Z","R"]` 綁到 `tool_vector_actual[0..3]` —— 即 **`r` = index 3**（`Rx` 槽），index 4/5 不用。
- **交叉印證**：dashboard `GetPose()` 回 `{X,Y,Z,R}`（4 分量），與「取前 4 個」一致。

→ 新增 `FeedbackFrame.pose` property 回 `(x, y, z, r) = tool_vector_actual[0..3]`，docstring 標明來源；測試 `test_pose_takes_r_from_tool_vector_index_3` 釘死。

**待辦（硬體）**：下次實機 session 用 live frame 把 `pose.r` 對 30004 對應分量交叉驗證（workbench 目前 Δ 只比對 x/y/z）。

## 型別解讀差異（offset 安全，不影響佈局）

兩欄 PDF 標 `double`、reference demo + 我們標 `int64`（皆 8 bytes，offset 完全不受影響，僅值解讀不同）：

| 欄位 | PDF 型別 | demo / 我們 | 處置 |
|---|---|---|---|
| `traceIndex` @1296 | double | int64 | 維持 int64（index 性質；demo 贏線上格式） |
| `SixForceValue[6]` @1304 | double | int64 | 維持 int64（PDF 本身標保留位） |

依 CLAUDE.md 權威序，線上 byte 格式以 demo 為準；已在 dtype 加註說明。

## 驗收

- `python -m unittest discover -s tests` → 185 passed（feedback 10 條全綠，含 3 條新測試）。
- 實機 live `r` 交叉驗證 → 列硬體待辦。
