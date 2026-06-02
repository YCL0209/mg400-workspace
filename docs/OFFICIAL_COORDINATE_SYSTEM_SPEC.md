# MG400 官方坐標系 Spec(TCP/IP 4 軸)

**來源**:Dobot 官方《TCP/IP 远程控制接口文档(4轴)_20240419》(68 頁,pdftotext 抽取)。
**用途**:供開發 agent 對接坐標系相關功能。
**權威性**:標「官方」者逐字對應 PDF;標「對接註記」者為本專案補充,非官方。
**版本需求**:`SetUser` / `CalcUser` / `SetTool` / `CalcTool` 於**控制器 1.6.0.0**(2024/01/30)新增;`User()` / `Tool()` / `GetPose` / `GetAngle` / `PositiveSolution` / `InverseSolution` 屬既有指令。下指令前確認控制器韌體版本。

---

## 1. 坐標系模型(四系,層層相對)

| 坐標系 | 角色 | 索引 | 索引 0 初始值 |
|---|---|---|---|
| 基坐標系 Base | 底座 / 世界原點 | — | — |
| 法蘭坐標系 Flange | 末端法蘭安裝面 | — | — |
| 用戶坐標系 User | 自訂工件/工作平面參考 | `[0,9]` | = 基坐標系 |
| 工具坐標系 Tool | TCP 相對法蘭的偏移 | `[0,9]` | = 法蘭坐標系 |

- User 系決定「笛卡爾座標相對誰量」;Tool 系決定「TCP(工具中心點)在法蘭上的偏移」。
- User/Tool 各有 10 個槽(0~9);槽 0 是不可改的預設基準(基坐標系 / 法蘭坐標系)。

## 2. 位姿格式約定(官方)

所有坐標系值與笛卡爾點位一律 `{x, y, z, r}`:
- `x, y, z`:平移,單位 **mm**
- `r`:繞 z 軸旋轉,單位 **度**
- 4 軸 SCARA 只有單一旋轉自由度,故只有 `r`,無 RPY/四元數。
- 大括號可省略:`SetUser(1,{10,10,10,0})` ≡ `SetUser(1,10,10,10,0)`。

## 3. 指令總覽

| 指令 | 類型 | 作用 | 版本 |
|---|---|---|---|
| `User(index)` | 隊列 | 設**全局**用戶坐標系 | 既有 |
| `Tool(index)` | 隊列 | 設**全局**工具坐標系 | 既有 |
| `SetUser(index,table)` | 立即 | 修改指定用戶坐標系之值 | 1.6.0+ |
| `SetTool(index,table)` | 立即 | 修改指定工具坐標系之值 | 1.6.0+ |
| `CalcUser(index,matrix_direction,table)` | 立即 | 計算用戶坐標系變換後之值 | 1.6.0+ |
| `CalcTool(index,matrix_direction,table)` | 立即 | 計算工具坐標系變換後之值 | 1.6.0+ |
| `GetPose(User=0,Tool=0)` | 立即 | 讀當前位姿之笛卡爾座標(可指定系) | 既有 |
| `GetAngle()` | 立即 | 讀當前位姿之關節座標 | 既有 |
| `PositiveSolution(J1,J2,J3,J4,User,Tool)` | 立即 | 正解:關節 → 笛卡爾(控制器算) | 既有 |
| `InverseSolution(X,Y,Z,R,User,Tool,isJointNear,JointNear)` | 立即 | 逆解:笛卡爾 → 關節(控制器算) | 既有 |

## 4. 指令詳規(官方原文)

### 4.1 `User(index)` — 隊列指令
- **作用**:設定全局用戶坐標系。運動指令未指定 User 時用此全局值。
- **生效範圍**:僅本次 TCP/IP 控制 session;未設則沿用進入 TCP/IP 模式前控制軟體設定。
- **參數**:`index` int — 已標定的用戶坐標系索引。
- **回傳**:`ErrorID,{},User(index);`。`ErrorID=-1` 表示該索引不存在。
- **範例**:`User(1)` → 設用戶坐標系 1 為全局。

### 4.2 `Tool(index)` — 隊列指令
- **作用**:設定全局工具坐標系(語義同 `User`)。
- **參數**:`index` int — 已標定的工具坐標系索引。
- **回傳**:`ErrorID,{},Tool(index);`。`ErrorID=-1` 表示索引不存在。
- **範例**:`Tool(1)` → 設工具坐標系 1 為全局。

### 4.3 `SetUser(index,table)` — 立即指令(1.6.0+)
- **作用**:修改指定用戶坐標系的值。
- **參數**:`index` int `[0,9]`(0=基坐標系初始值);`table` `{x,y,z,r}`,建議用 `CalcUser` 取得。
- **回傳**:`ErrorID,{},SetUser(index,table);`
- **範例**:`SetUser(1,{10,10,10,0})` → 設用戶系 1 為 X=10,Y=10,Z=10,R=0。

### 4.4 `SetTool(index,table)` — 立即指令(1.6.0+)
- **作用**:修改指定工具坐標系的值。
- **參數**:`index` int `[0,9]`(0=法蘭坐標系初始值);`table` `{x,y,z,r}`,建議用 `CalcTool` 取得。
- **回傳**:`ErrorID,{},SetTool(index,table);`
- **範例**:`SetTool(1,{10,10,10,0})` → 設工具系 1 為 X=10,Y=10,Z=10,R=0。

### 4.5 `CalcUser(index,matrix_direction,table)` — 立即指令(1.6.0+)
- **作用**:計算用戶坐標系變換後的值(常與 `SetUser` 搭配:先 Calc 得結果再 Set)。
- **參數**:
  - `index` int `[0,9]`(0=基坐標系)
  - `matrix_direction` int:`1`=左乘(index 系沿**基坐標系**偏轉 table);`0`=右乘(index 系沿**自己**偏轉 table)
  - `table` `{x,y,z,r}` 偏移值
- **回傳**:`ErrorID,{x,y,z,r},CalcUser(index,matrix_direction,table);` —— `{x,y,z,r}` 為算出的新坐標系。
- **範例**:
  - `CalcUser(1,1,{10,10,10,10})`:用戶系 1 沿基坐標系平移 (10,10,10)、旋轉 r=10 → newUser。
  - `CalcUser(1,0,{10,10,10,10})`:用戶系 1 沿自己平移 (10,10,10)、旋轉 r=10 → newUser。

### 4.6 `CalcTool(index,matrix_direction,table)` — 立即指令(1.6.0+)
- **作用**:計算工具坐標系變換後的值。
- **參數**:`index` int `[0,9]`(0=法蘭坐標系);`matrix_direction`:`1`=左乘(沿**法蘭坐標系**偏轉);`0`=右乘(沿自己);`table` `{x,y,z,r}`。
- **回傳**:`ErrorID,{x,y,z,r},CalcTool(index,matrix_direction,table);`
- **範例**:`CalcTool(1,1,{10,10,10,10})`:工具系 1 沿法蘭坐標系平移 (10,10,10)、旋轉 r=10 → newTool。

### 4.7 `GetPose(User=0,Tool=0)` — 立即指令
- **作用**:取得機械臂當前位姿的笛卡爾座標。
- **參數**(均**可選**,不傳時用全局 User/Tool):`User` int — 已標定用戶系索引;`Tool` int — 已標定工具系索引。
- **回傳**:`ErrorID,{X,Y,Z,R},GetPose();`
- **範例**:
  - `GetPose()`:當前位姿在全局 User/Tool 下的笛卡爾座標。
  - `GetPose(User=1,Tool=0)`:當前位姿在用戶系 1、工具系 0 下的笛卡爾座標。
- **重點**:同一物理姿態,選不同 User/Tool 回傳的 `{X,Y,Z,R}` 不同。

### 4.8 `GetAngle()` — 立即指令
- **作用**:取得機械臂當前位姿的關節座標。
- **回傳**:`ErrorID,{J1,J2,J3,J4},GetAngle();`(4 軸機型只有 J1~J4)。

### 4.9 `PositiveSolution(J1,J2,J3,J4,User,Tool)` — 立即指令(正解)
- **作用**:給定各關節角,**由控制器**計算末端在指定 User 坐標系下的笛卡爾座標(TCP 由 Tool 系決定)。
- **參數**:`J1~J4` double(度);`User` int 已標定用戶系;`Tool` int 已標定工具系。
- **回傳**:`ErrorID,{x,y,z,r},PositiveSolution(J1,J2,J3,J4,User,Tool);`
- **範例**:`PositiveSolution(0,0,-90,0,1,1)`。

### 4.10 `InverseSolution(X,Y,Z,R,User,Tool,isJointNear,JointNear)` — 立即指令(逆解)
- **作用**:給定末端笛卡爾座標,**由控制器**計算各關節角。
- **多解問題**:一個位姿可對應多組關節解;靠 `isJointNear` / `JointNear` 指定一組參考關節座標,系統選最接近的解。
- **參數**:`X,Y,Z` double(mm);`R` double(度);`User`/`Tool` int;`isJointNear` 是否啟用就近解;`JointNear` 參考關節座標。
- **回傳**:`ErrorID,{J1,J2,J3,J4},InverseSolution(X,Y,Z,R,User,Tool,isJointNear,JointNear);`

## 5. 關鍵語義(官方)

1. **隊列 vs 立即**:`User()`/`Tool()` 是**隊列指令**(進運動佇列,依序生效);其餘 Set/Calc/Get/Solution 是**立即指令**(即時回應,不進佇列)。
2. **全局坐標系只在本次 session 生效**:`User()`/`Tool()` 設的全局值僅本次 TCP/IP 模式有效;沒設就沿用進入前控制軟體(DobotStudio)的設定。
3. **左乘 vs 右乘**(`matrix_direction`):`1`=左乘=沿**父系**(User 沿基、Tool 沿法蘭)偏轉;`0`=右乘=沿**自身**偏轉。決定 table 偏移的參考框。
4. **標定前提**:User/Tool 坐標系須先以控制軟體標定,才能用 index 選用;或用 `SetUser`/`SetTool`(搭配 `CalcUser`/`CalcTool`)程式化設定。
5. **正逆解皆由控制器計算**:FK/IK 數學不對外公開,只能透過 `PositiveSolution`/`InverseSolution` 指令取得;文檔不提供 DH 參數、連桿長度、變換矩陣。

## 6. 文檔未涵蓋(明確記錄,避免 agent 誤找)

- **FK/IK 運算式**:無。控制器黑盒,只開正逆解指令。
- **關節極限數值**(J1~J4 各自範圍):協定 PDF 無數值表。
- **平行四邊形 J3−J2 耦合約束/閾值**:協定 PDF 無;官方 `alarm_controller.json` 僅命名報警(平行四邊形限位 id 72/73、關節限位 id 66~69),**不給觸發角度**。極限數值只能實機探。

---

## 對接註記(本專案補充,非官方)

- 本專案 `robot_core/kinematics.forward_kinematics` 計算的是**法蘭中心在基坐標系**下的位姿,等同官方 `User=0, Tool=0`。`config/calibration_pairs.json` 的 pose 同此基準。
- 裝工具時,兩條路:(a) 用控制器 `Tool(index)` 讓控制器算 TCP;(b) 在本專案 kinematics/safety 自加 tool z_offset(`config/safety.json` 的 `_future_tool_offset` 已留 hook)。
- 坐標系(位置參考基準)與關節極限(可達範圍)正交:前者 PDF 有指令定義,後者 PDF 無數值。
