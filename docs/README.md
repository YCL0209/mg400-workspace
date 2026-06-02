# `docs/` — SDK 規範參考 + 專案審計

這個目錄收的是「**比 PROGRESS / NEXT_TASKS 更靠官方**」的參考資料：從原廠 PDF 抽出來的規範、跟我們專案的差異盤點、歷史審計報告。當你（或未來的 agent）需要查「官方到底怎麼說」時，看這裡。

## 權威優先序（凡有衝突）

> 1. **官方 SDK PDF**（韌體 contract，最高）→ 抽取進 `OFFICIAL_*.md`
> 2. **demo（`reference/TCP-IP-4Axis-Python/`）線上 byte 格式**（PDF 不明確或衝突時的活證據）
> 3. **CLAUDE.md / PROGRESS.md / 本目錄外的我們文件**（整理 + 實機補出的擴充，最低）

這條規則也寫在 `CLAUDE.md` 第一規則段。

## 檔案

| 檔 | 性質 | 用途 |
|---|---|---|
| `OFFICIAL_COORDINATE_SYSTEM_SPEC.md` | **官方規範**（逐字抽自 PDF） | 4 個座標系（Base/Flange/User[0,9]/Tool[0,9]）模型 + 10 條座標系指令逐條詳規（User/Tool/SetUser/SetTool/CalcUser/CalcTool/GetPose/GetAngle/PositiveSolution/InverseSolution）。**未來開 UI 座標圖、做 tool offset 都從這查**。 |
| `OFFICIAL_VS_PROJECT_DIFF.md` | **審計**（官方 ⟷ 我們） | 6 大類差異盤點：A 已對齊 ✅、B1-B6 缺口 ⚠、C 文件過期。每條給優先序 + 建議補法。**這是 Phase 3.2 之後的 backlog 來源**。 |
| `REFERENCE_AUDIT_2026-06-01.md` | **歷史審計**（已部分過時） | 2026-06-01 第一次審計：8 大反模式對照 + 協定忠實度 + 觸發 finding 18（送 `;` 修法）。標頭有 stale 警告，看時注意對應「現況」要看上面兩份。 |

## 跟其他文件的關係

- 想看**現況**（code 跟 doc 怎麼對齊現在的 main）→ 看 `OFFICIAL_VS_PROJECT_DIFF.md` §A
- 想看**該補什麼**（沒做、做了一半的官方指令）→ 看 `OFFICIAL_VS_PROJECT_DIFF.md` §B
- 想看**座標系怎麼用**（指令原型、參數、範例）→ 看 `OFFICIAL_COORDINATE_SYSTEM_SPEC.md`
- 想看**我們實機補出的協定真理**（PDF 沒寫的）→ 看 `../PROGRESS.md` finding 區段
- 想看**短期 / 中期 / 長期路線圖**（什麼時候補什麼指令）→ 看 `../NEXT_TASKS.md`

## 來源 / Provenance

- `OFFICIAL_*.md` 由另一支 agent（spec extractor）從《TCP/IP 远程控制接口文档（4軸）_20240419》（68 頁 PDF，控制器韌體 1.7.0.0）逐字抽出。
- `REFERENCE_AUDIT_2026-06-01.md` 由「專案對比師」agent 在 2026-06-01 第一次硬體 session 前夕產出。
- 三檔原版本一度存在於 `.claude/worktrees/comparison-report` 或 repo 根目錄；本 PR 統一搬進 `docs/`。
