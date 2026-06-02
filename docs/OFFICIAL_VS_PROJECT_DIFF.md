# MG400 協定差異稽核:本專案 vs 官方 PDF(以官方為準)

**權威**:官方《TCP/IP 远程控制接口文档(4軸)_20240419》(控制器 1.7.0.0)。凡分歧以官方為準。
**對照**:本專案 `robot_core/` + 開發 md;demo(reference fork)為第三方佐證。
**抽取者**:專案對比師。

## 總評
核心協定(埠、框架、回應格式、feedback、狀態碼)**已忠實對齊官方**。差異集中在**指令覆蓋與可選參數缺口**(Phase 0 刻意精簡),外加一份 worktree 內的過期報告。**無「我們做錯、官方相反」的硬衝突。**

---

## A. 已對齊官方(核心協定,無需動作)✅

| 協定事實 | 官方(PDF 行) | 本專案 | 判定 |
|---|---|---|---|
| 三埠 29999/30003/30004 | 行 56-65 | `config/robot.json` | ✅ |
| 回應格式 `ErrorID,{val},Func();` | 行 84-88 | `framing.py` 解析 | ✅ |
| **送指令不帶 `;`** | 行 92/99(範例只有括號) | `connection.py:226`(commit 3d623c1 已拿掉) | ✅ 已對齊 |
| 收回應以 `;` 終結 | 行 84 | `extract_frames` 按 `;` 切 | ✅ |
| feedback 1440 bytes | 行 65/2837 | `feedback.py:36` | ✅ |
| magic `0x0123456789ABCDEF` | 行 ~2848(TestValue@offset 48) | `feedback.py:40` | ✅ |
| RobotMode 1~11 列舉 | 行 1011-1086 | PROGRESS finding 14 字典 | ✅ |
| 錯誤碼 -1/-10000/-20000/-30xxx/-40xxx | 行 3957-4036 | docs 記錄 | ✅ |
| SpeedFactor [1,100] | 行 437/458 | `builders.py:132` 驗證 | ✅ |
| 笛卡爾 `{x,y,z,r}` mm/度 | 坐標系段 | `OFFICIAL_COORDINATE_SYSTEM_SPEC.md` | ✅ |
| 指令大小寫不敏感 | 行 77-80 | (demo 用小寫 `continue()` 亦合法) | ✅ |

## B. 差異與缺口(以官方為準,建議補齊)⚠

### B1. `EnableRobot` 缺參數簽名
- **官方**(行 152/188-191):`EnableRobot(load,centerX,centerY,centerZ)`,支援 **0 / 1 / 4** 參數;load=double(kg)、centerX/Y/Z=double(-500~500 mm)。
- **本專案**(`builders.py:79`):只有 `EnableRobot()`,無參數簽名。
- **demo**:`EnableRobot(*dynParams)`,支援可變參數。
- **影響**:無法在使能時設負載/質心。對 500g 負載 + 偏心 ≤40mm 的實際抓取場景是缺口(否則動力學補償不準)。
- **建議**:`enable_robot(load=None, cx=None, cy=None, cz=None)` 支援 0/1/4 簽名,套官方值域驗證。

### B2. `MovL` / `MovJ` 缺可選參數
- **官方**(行 2082/2131):`MovL(X,Y,Z,R,User=,Tool=,SpeedL=,AccL=,CP=)`(MovJ 同,用 SpeedJ/AccJ)。
- **本專案**(`builders.py:161/166`):只有 `MovL(x,y,z,r)` 四位置參數。
- **影響**:無法逐指令設速度/加速度/坐標系/平滑過渡(CP)。動作只能吃全局速率。
- **建議**:加可選 kwargs,拼成 `MovL(x,y,z,r,SpeedL=..,AccL=..,User=..,Tool=..,CP=..)`。

### B3. `JointMovJ` 缺可選參數
- **官方**(行 2176):`JointMovJ(J1,J2,J3,J4,SpeedJ=,AccJ=,CP=)`。
- **本專案**(`builders.py:175`):只有 4 位置參數。
- **建議**:同 B2 加可選 SpeedJ/AccJ/CP。

### B4. 指令覆蓋缺口(官方有、我們無)
官方文檔記載但 `builders.py` 未實作:
- **IO**:`DO(index,status)`(行 1427)— demo 有,我們無。
- **運動**:`Arc`(行 2392)、`Circle`(行 2465)、`MovLIO`(行 2222)、`MovJIO`(行 2308)。
- **速度/加速度獨立設定**:`SpeedJ`/`AccJ`/`SpeedL`/`AccL`(行 579-655)。
- **坐標系**:`User`/`Tool`/`SetUser`/`SetTool`/`CalcUser`/`CalcTool`(見坐標系 spec)。
- **正逆解**:`PositiveSolution`/`InverseSolution`。
- **註**:Phase 0 範圍刻意只做連線/使能/讀狀態,以上多屬後續 Phase。**相對官方屬「未實作」非「做錯」**,列此供路線圖排序。

### B5. 額外 feedback 埠未利用
- **官方**(行 65-68):除 30004(8ms/1440B)外,另有 **30005**(200ms)、**30006**(可配置,預設 50ms)。
- **本專案**:只用 30004。
- **影響**:無(功能足夠)。僅記錄官方尚有低頻 feedback 埠可選用。

### B6. feedback 欄位偏移未逐欄對官方表
- **官方**(行 2836-3700):給**精確 byte offset 表**——如 RobotMode@0024、TestValue@0048、QActual@0432、ToolVectorActual@0624(6 doubles:X,Y,Z,Rx,Ry,Rz)、狀態旗標@~1025-1029。
- **本專案**(`feedback.py` numpy dtype):實機 magic 驗框通過(⇒ TestValue@48 正確)、FK 匹配(⇒ q_actual / tool_vector 我們讀的欄位偏移正確)。但**未對全部 1440 bytes 逐欄核對官方偏移**。
- **特別注意**:官方 `ToolVectorActual` 是 **6 分量**(含 Rx,Ry,Rz);4 軸機型的 `r` 對應哪一個分量要確認(我們 workbench 的 Δ 只比對 x,y,z 三軸,r 未從 feedback 交叉驗證)。
- **建議**:做一次完整 dtype↔官方偏移表核對,把保留欄位/padding 對齊,並釘死 4 軸 r 的來源分量。

## C. 文檔過期 / 不一致 ⚠

### C1. `COMPARISON_REPORT.md`(worktree 內)已過期
- 內容仍寫「現在 code 送**帶** `;`、建議拿掉」,但主 repo 已於 commit `3d623c1` 拿掉。
- 該報告對 `;` 的「現況」與「建議」**都 stale**。該檔僅存在於 worktree,主 repo 無。
- **建議**:更新或標註該報告 `;` 段為「已實作對齊」,或從交付物移除。

### C2. `CLAUDE.md` ✅ 已對齊(確認,非問題)
- `CLAUDE.md:83`「送指令不加 `;`」現與**官方**(行 92/99)+ **code**(connection.py:226)三方一致。無需改。

### C3. 錯誤碼 `-10000` 語義:我們比官方多記錄(標明來源)
- **官方**(行 ~4000):`-10000 = 命令不存在`(定義較窄)。
- **本專案**(PROGRESS findings 16/17):記錄韌體**過載使用** -10000——指令存在但「狀態拒絕 / 缺埠未掛載」也回 -10000。
- **這是我們靠實機補出的擴充,非與官方矛盾**;但 docs 應標明「官方定義 = 命令不存在;-10000 在本韌體被過載,需交叉比對」,避免誤判。(現 PROGRESS 已有此註,維持即可。)

## D. 逐項對照速查

| 項 | 官方 | 本專案 | demo | 結論 |
|---|---|---|---|---|
| 送 `;` | 不需 | 不帶(已修) | 不帶 | ✅ 對齊 |
| EnableRobot 參數 | 0/1/4 | 僅 0 | 可變 | ⚠ B1 缺 |
| MovL/MovJ 可選參數 | 有 | 無 | 有 | ⚠ B2 缺 |
| JointMovJ 可選參數 | 有 | 無 | 有 | ⚠ B3 缺 |
| DO / Arc / Circle / IO | 有 | 無 | 有 | ⚠ B4 缺 |
| 坐標系 / 正逆解指令 | 有 | 無 | 有 | ⚠ B4 缺 |
| feedback 30005/30006 | 有 | 未用 | 未用 | ⓘ B5 |
| feedback 偏移逐欄 | 精確表 | 部分驗證 | 無驗證 | ⚠ B6 待核 |
| RobotMode/錯誤碼/格式 | 規範 | 對齊 | 對齊 | ✅ |

---

## 給開發 agent 的建議優先序
1. **B6**(feedback 偏移逐欄核對 + 釘死 4 軸 r 分量)— 影響讀數正確性,優先。
2. **B1/B2/B3**(補 EnableRobot 負載參數、Mov* 可選速度/坐標系)— 進入運動 Phase 的前置。
3. **B4**(DO/Arc/Circle/正逆解/坐標系指令)— 依 Phase 路線圖逐步補。
4. **C1**(更新過期的 COMPARISON_REPORT)。

凡實作以**官方 PDF 原型 + 值域**為準;線上 byte 格式若與 demo 衝突,先對齊 demo 再求證。
