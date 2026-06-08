# MG400 官方 TCP/IP 指令完整參考(4 軸)

**來源**:Dobot 官方《TCP/IP 远程控制接口文档(4轴)_20240419》(68 頁,對應控制器版本 **1.7.0.0**;`pdftotext -layout` 逐字抽取)。
**用途**:供開發 agent / Claude Code 對接 MG400 TCP/IP 介面時,逐指令查原型、參數、回傳與範例。
**權威性**:本文標「官方」段落逐字對應 PDF;標「對接註記」者為本專案補充,非官方。衝突時以 PDF 原文為準(權威階序見 `CLAUDE.md`:PDF > `reference/TCP-IP-4Axis-Python/` > 本文/PROGRESS)。
**涵蓋**:**67 個指令**(Dashboard 53 + 運動 14)+ §3 1440-byte 實時反饋欄位表 + §4 通用錯誤碼。
**相關文件**:坐標系語意深入說明見 [`OFFICIAL_COORDINATE_SYSTEM_SPEC.md`](OFFICIAL_COORDINATE_SYSTEM_SPEC.md);專案實作落差見 [`OFFICIAL_VS_PROJECT_DIFF.md`](OFFICIAL_VS_PROJECT_DIFF.md);程式對應在 `robot_core/protocol/builders.py`。

> **版本提醒**:部分指令有最低韌體要求(逐指令標註)。`SetUser/CalcUser/SetTool/CalcTool/PalletCreate/GetPalletPose` 需 **1.6.0+**;`MoveJog/RelMovJUser/RelMovLUser/RelJointMovJ` 需 **1.5.6+**;`Wait` 需 **1.5.9+**;TCP/IP 協議本身需 **1.5.5.0+**。下指令前確認控制器韌體版本。

---

## 0. 通用約定(官方,PDF §1–§2 前言)

### 端口

| 端口 | 名稱 | 用途 | 反饋週期 |
|---|---|---|---|
| **29999** | Dashboard | 上位機下發設置類指令、主動查詢機器人狀態 | — |
| **30003** | 運動 | 下發運動相關指令控制機器人運動 | — |
| **30004** | 實時反饋 | 機器人實時狀態(1440 byte/包) | 每 **8ms** |
| **30005** | 實時反饋 | 同上格式 | 每 **200ms**(1.5.9+ 可配置) |
| **30006** | 實時反饋 | 同上格式,可配置(預設 50ms,改週期需聯繫技術支持) | 預設 **50ms** |

### 應答格式

所有指令下發後機器人回應一條訊息:

```
ErrorID,{value,...,valueN},消息名稱(Param1,Param2,...,ParamN);
```

- `ErrorID == 0`:命令接收成功;非 0 代表有錯誤,見 [§4 通用錯誤碼](#4-通用錯誤碼通用-pdf-§5)。
- `{value,...,valueN}`:回傳值,無回傳值則為 `{}`。
- `消息名稱(...)`:即下發的命令本身。

範例:下發 `MovL(-500,100,200,150)` → 回 `0,{},MovL(-500,100,200,150);`(0=成功,`{}`=無回傳)。
下發不存在的 `Mov(-500,100,200,150)` → 回 `-10000,{},Mov(-500,100,200,150);`(-10000=命令不存在)。

### 立即指令 vs 隊列指令

- **立即指令**:下發後立刻執行並返回執行結果。
- **隊列指令**:下發後立刻返回,但不立刻執行 —— 進入後台演算法隊列排隊等待執行。
- Dashboard 指令(29999)大部分為立即指令,部分與運動/IO 相關者為隊列指令(本文每指令標題已標註)。
- **運動指令(30003)全部為隊列指令。**

**陷阱**:若在隊列指令後緊接立即指令,立即指令可能在隊列指令完成前就執行。例:

```
MovJ(-100,100,200,150)   // 隊列指令
RobotMode()              // 立即指令 → 在運動完成前執行,返回 7(運動中)
```

若要確保立即指令執行時前序指令都已執行完,先呼叫 `Sync()`(阻塞至前序全部執行完):

```
MovJ(-500,100,200,150)   // 隊列指令
Sync()
RobotMode()              // 在運動完成後執行,返回 5(空閒)
```

### 位姿格式

笛卡爾點位與坐標系值一律 `{x, y, z, r}`:`x,y,z` 平移(mm)、`r` 繞 z 軸旋轉(度)。4 軸 SCARA 只有單一旋轉自由度故僅 `r`。大括號可省略:`SetUser(1,{10,10,10,0})` ≡ `SetUser(1,10,10,10,0)`。

---

## 指令總覽表

> 類型欄:**立即** = 立即指令,**隊列** = 隊列指令。版本欄空白表示既有指令(≥1.5.5.0)。

### Dashboard(29999)

| 指令 | 類型 | 作用 | 版本 |
|---|---|---|---|
| [`EnableRobot`](#enablerobot立即-29999) | 立即 | 使能機械臂(執行隊列指令前必須) | |
| [`DisableRobot`](#disablerobot立即-29999) | 立即 | 下使能機器人 | |
| [`ClearError`](#clearerror立即-29999) | 立即 | 清除機器人報警 | |
| [`ResetRobot`](#resetrobot立即-29999) | 立即 | 停止機器人並清空指令隊列 | |
| [`RunScript`](#runscript立即-29999) | 立即 | 運行指定工程 | |
| [`StopScript`](#stopscript立即-29999) | 立即 | 停止運行中的工程 | |
| [`PauseScript`](#pausescript立即-29999) | 立即 | 暫停運行中的工程 | |
| [`ContinueScript`](#continuescript立即-29999) | 立即 | 繼續已暫停的工程 | |
| [`Pause`](#pause立即-29999) | 立即 | 暫停 TCP 下發的運動指令(不清隊列) | |
| [`Continue`](#continue立即-29999) | 立即 | 繼續 Pause 暫停的運動指令 | |
| [`StartDrag`](#startdrag立即-29999) | 立即 | 進入拖拽模式 | |
| [`StopDrag`](#stopdrag立即-29999) | 立即 | 退出拖拽模式 | |
| [`EmergencyStop`](#emergencystop立即-29999) | 立即 | 緊急停止並下電 | |
| [`Wait`](#wait隊列-29999) | 隊列 | 指令隊列延時 | 1.5.9+ |
| [`SpeedFactor`](#speedfactor立即-29999) | 立即 | 設置全局速度比例 | |
| [`User`](#user隊列-29999) | 隊列 | 設全局用戶坐標系 | |
| [`Tool`](#tool隊列-29999) | 隊列 | 設全局工具坐標系 | |
| [`SetPayLoad`](#setpayload隊列-29999) | 隊列 | 設末端負載 | |
| [`AccJ`](#accj隊列-29999) | 隊列 | 設關節運動加速度比例 | |
| [`AccL`](#accl隊列-29999) | 隊列 | 設直線/弧線運動加速度比例 | |
| [`SpeedJ`](#speedj隊列-29999) | 隊列 | 設關節運動速度比例 | |
| [`SpeedL`](#speedl隊列-29999) | 隊列 | 設直線/弧線運動速度比例 | |
| [`Arch`](#arch隊列-29999) | 隊列 | 設 Jump 運動全局門型參數索引 | |
| [`CP`](#cp隊列-29999) | 隊列 | 設平滑過渡比例 | |
| [`SetArmOrientation`](#setarmorientation隊列-29999) | 隊列 | 設手系(M1 Pro 特有) | |
| [`SetCollisionLevel`](#setcollisionlevel隊列-29999) | 隊列 | 設碰撞檢測等級 | |
| [`SetUser`](#setuser立即-29999) | 立即 | 修改指定用戶坐標系 | 1.6.0+ |
| [`CalcUser`](#calcuser立即-29999) | 立即 | 計算用戶坐標系 | 1.6.0+ |
| [`SetTool`](#settool立即-29999) | 立即 | 修改指定工具坐標系 | 1.6.0+ |
| [`CalcTool`](#calctool立即-29999) | 立即 | 計算工具坐標系 | 1.6.0+ |
| [`RobotMode`](#robotmode立即-29999) | 立即 | 取機器人當前狀態(1~11) | |
| [`GetAngle`](#getangle立即-29999) | 立即 | 取當前關節坐標 | |
| [`GetPose`](#getpose立即-29999) | 立即 | 取當前笛卡爾坐標 | |
| [`GetErrorID`](#geterrorid立即-29999) | 立即 | 取當前報錯錯誤碼 | |
| [`PositiveSolution`](#positivesolution立即-29999) | 立即 | 正解(關節角 → 笛卡爾) | |
| [`InverseSolution`](#inversesolution立即-29999) | 立即 | 逆解(笛卡爾 → 關節角) | |
| [`PalletCreate`](#palletcreate立即-29999) | 立即 | 創建托盤 | 1.6.0+ |
| [`GetPalletPose`](#getpalletpose立即-29999) | 立即 | 取托盤指定點位 | 1.6.0+ |
| [`DO`](#do隊列-29999) | 隊列 | 設數字輸出端口(隊列) | |
| [`DOExecute`](#doexecute立即-29999) | 立即 | 設數字輸出端口(立即) | |
| [`DOGroup`](#dogroup立即-29999) | 立即 | 設多個數字輸出端口(立即) | |
| [`ToolDO`](#tooldo隊列-29999) | 隊列 | 設末端數字輸出(隊列) | |
| [`ToolDOExecute`](#tooldoexecute立即-29999) | 立即 | 設末端數字輸出(立即) | |
| [`DI`](#di立即-29999) | 立即 | 取 DI 端口狀態 | |
| [`ToolDI`](#tooldi立即-29999) | 立即 | 取末端 DI 端口狀態 | |
| [`ModbusCreate`](#modbuscreate立即-29999) | 立即 | 創建 Modbus 主站並連接從站 | |
| [`ModbusClose`](#modbusclose立即-29999) | 立即 | 斷開 Modbus 連接、釋放主站 | |
| [`GetInBits`](#getinbits立即-29999) | 立即 | 讀觸點寄存器(離散輸入) | |
| [`GetInRegs`](#getinregs立即-29999) | 立即 | 讀輸入寄存器 | |
| [`GetCoils`](#getcoils立即-29999) | 立即 | 讀線圈寄存器 | |
| [`SetCoils`](#setcoils立即-29999) | 立即 | 寫線圈寄存器 | |
| [`GetHoldRegs`](#getholdregs立即-29999) | 立即 | 讀保持寄存器 | |
| [`SetHoldRegs`](#setholdregs立即-29999) | 立即 | 寫保持寄存器 | |

### 運動(30003,全為隊列指令)

| 指令 | 作用 | 版本 |
|---|---|---|
| [`MovJ`](#movj隊列-30003) | 關節運動至笛卡爾目標點 | |
| [`MovL`](#movl隊列-30003) | 直線運動至笛卡爾目標點 | |
| [`JointMovJ`](#jointmovj隊列-30003) | 關節運動至關節坐標目標點 | |
| [`MovLIO`](#movlio隊列-30003) | 直線運動並行設 DO | |
| [`MovJIO`](#movjio隊列-30003) | 關節運動並行設 DO | |
| [`Arc`](#arc隊列-30003) | 圓弧插補運動 | |
| [`Circle`](#circle隊列-30003) | 整圓插補運動 | |
| [`MoveJog`](#movejog隊列-30003) | 點動機械臂 | 1.5.6+ |
| [`Sync`](#sync隊列-30003) | 阻塞至隊列最後指令執行完 | |
| [`RelMovJUser`](#relmovjuser隊列-30003) | 沿用戶坐標系相對關節運動 | 1.5.6+ |
| [`RelMovLUser`](#relmovluser隊列-30003) | 沿用戶坐標系相對直線運動 | 1.5.6+ |
| [`RelJointMovJ`](#reljointmovj隊列-30003) | 沿關節坐標系相對關節運動 | 1.5.6+ |
| [`MovJExt`](#movjext隊列-30003) | 控制滑軌(擴展軸)運動 | |
| [`SyncAll`](#syncall隊列-30003) | 阻塞至隊列所有指令執行完 | |

---

## 1. Dashboard 指令(29999 端口)

> 所有 Dashboard 指令透過 **29999** 端口下發。

### 2.1 控制相關指令

#### EnableRobot(立即, 29999)
**原型** `EnableRobot(load,centerX,centerY,centerZ)`
**描述** 使能機械臂。執行隊列指令(機械臂運動、隊列 IO 等)前必須先使能機械臂。
**參數**(均為可選,可帶 **0 / 1 / 4** 個)

| 參數名 | 類型 | 範圍/單位 | 說明 |
|---|---|---|---|
| load | double | kg | 負載重量,不可超過各型號機器人負載範圍 |
| centerX | double | -500 ~ 500 mm | X 方向偏心距離 |
| centerY | double | -500 ~ 500 mm | Y 方向偏心距離 |
| centerZ | double | -500 ~ 500 mm | Z 方向偏心距離 |

- 帶 **0** 個:使能時不設置負載重量與偏心參數。
- 帶 **1** 個:該參數表示負載重量。
- 帶 **4** 個:分別為負載重量與偏心參數。

**回傳** `ErrorID,{},EnableRobot(load,centerX,centerY,centerZ);`
**範例** `EnableRobot()`(不設負載/偏心);`EnableRobot(0.5)`(負載 0.5kg);`EnableRobot(0.5,0,0,5.5)`(負載 + Z 偏心 5.5mm)。

#### DisableRobot(立即, 29999)
**原型** `DisableRobot()`
**描述** 下使能機器人。
**回傳** `ErrorID,{},DisableRobot();`

#### ClearError(立即, 29999)
**原型** `ClearError()`
**描述** 清除機器人報警。清除後可用 `RobotMode` 判斷是否仍處報警狀態。部分報警需解決原因或重啟控制櫃才能清除。
**回傳** `ErrorID,{},ClearError();`
> **說明**:清除報警後,需透過 `Continue` 指令重新開啟運動隊列。

#### ResetRobot(立即, 29999)
**原型** `ResetRobot()`
**描述** 停止機器人,清空已規劃的指令隊列。
**回傳** `ErrorID,{},ResetRobot();`

#### RunScript(立即, 29999)
**原型** `RunScript(projectName)`
**描述** 運行指定工程。
**參數** `projectName`(string):工程文件的名稱。
**回傳** `ErrorID,{},RunScript(projectName);`
**範例** `RunScript("demo")` 運行名為 demo 的工程。

#### StopScript(立即, 29999)
**原型** `StopScript()`
**描述** 停止正在運行的工程。
**回傳** `ErrorID,{},StopScript();`

#### PauseScript(立即, 29999)
**原型** `PauseScript()`
**描述** 暫停正在運行的工程。
**回傳** `ErrorID,{},PauseScript();`

#### ContinueScript(立即, 29999)
**原型** `ContinueScript()`
**描述** 繼續已暫停的工程。
**回傳** `ErrorID,{},ContinueScript();`

#### Pause(立即, 29999)
**原型** `Pause()`
**描述** 暫停非工程下發的運動指令(一般即 TCP 下發的運動指令),不清空運動隊列。
**回傳** `ErrorID,{},Pause();`

#### Continue(立即, 29999)
**原型** `Continue()`
**描述** 與 `Pause` 對應,繼續運行 Pause 暫停的運動指令;或用於碰撞、報警導致機器人停止後,重新恢復接收運動指令並運行。
**回傳** `ErrorID,{},Continue();`

#### StartDrag(立即, 29999)
**原型** `StartDrag()`
**描述** 機械臂進入拖拽模式。處於報錯狀態時無法進入。
**回傳** `ErrorID,{},StartDrag();`

#### StopDrag(立即, 29999)
**原型** `StopDrag()`
**描述** 機械臂退出拖拽模式。處於報錯狀態時無法退出。
**回傳** `ErrorID,{},StopDrag();`

#### EmergencyStop(立即, 29999)
**原型** `EmergencyStop()`
**描述** 緊急停止機械臂。急停後機械臂會下電並報警,需清除報警後才能重新上電與使能。
**回傳** `ErrorID,{},EmergencyStop();`

#### Wait(隊列, 29999)
**原型** `wait(time)` ⚠️ 原文小寫
**描述** 指令隊列延時一段時間。控制器 **1.5.9+** 支持。
**參數** `time`(int):延時時間,單位 ms,範圍 (0, 3600*1000)。
**回傳** `ErrorID,{},wait(time);`
**範例** `wait(1000)` 延時 1000ms。

### 2.2 設置相關指令

#### SpeedFactor(立即, 29999)
**原型** `SpeedFactor(ratio)`
**描述** 設置全局速度比例。實際運動速度 = 運動指令可選參數比例 × 控制軟件設定值 × 全局速度比例。僅在本次 TCP/IP 控制模式生效,未設置時沿用進入模式前控制軟件設定值。
**參數** `ratio`(int):全局運動速度比例,取值範圍 1~100。
**回傳** `ErrorID,{},SpeedFactor(ratio);`
**範例** `SpeedFactor(80)` 設全局比例 80%。

#### User(隊列, 29999)
**原型** `User(index)`
**描述** 設置全局用戶坐標系。下發運動指令時若未指定坐標系,使用此全局值。僅本次 TCP/IP 模式生效。
**參數** `index`(int):已標定的用戶坐標系索引(需先透過控制軟件標定)。
**回傳** `ErrorID,{},User(index);`若 `ErrorID == -1` 表示設置的用戶坐標索引不存在。
**範例** `User(1)`。
> 坐標系語意詳見 [`OFFICIAL_COORDINATE_SYSTEM_SPEC.md`](OFFICIAL_COORDINATE_SYSTEM_SPEC.md)。

#### Tool(隊列, 29999)
**原型** `Tool(index)`
**描述** 設置全局工具坐標系。下發運動指令時若未指定坐標系,使用此全局值。僅本次 TCP/IP 模式生效。
**參數** `index`(int):已標定的工具坐標系索引。
**回傳** `ErrorID,{},Tool(index);`若 `ErrorID == -1` 表示工具坐標索引不存在。
**範例** `Tool(1)`。
> 坐標系語意詳見 [`OFFICIAL_COORDINATE_SYSTEM_SPEC.md`](OFFICIAL_COORDINATE_SYSTEM_SPEC.md)。

#### SetPayLoad(隊列, 29999)
**原型** `SetPayLoad(weight,inertia)`
**描述** 設置機械臂末端負載。
**參數**

| 參數名 | 類型 | 範圍/單位 | 說明 |
|---|---|---|---|
| weight | float | kg | 負載重量,不可超過各型號負載範圍 |
| inertia | float | kgm² | 可選參數,負載慣量 |

**回傳** `ErrorID,{},SetPayLoad(weight,inertia);`
**範例** `SetPayLoad(0.3)` 設末端負載 0.3kg。

#### AccJ(隊列, 29999)
**原型** `AccJ(R)`
**描述** 設置關節運動方式的加速度比例。僅本次 TCP/IP 模式生效,未設置時預設 100。
**參數** `R`(int):加速度比例,取值範圍 [1,100]。
**回傳** `ErrorID,{},AccJ(R);`
**範例** `AccJ(50)`。

#### AccL(隊列, 29999)
**原型** `AccL(R)`
**描述** 設置直線和弧線運動方式的加速度比例。僅本次 TCP/IP 模式生效,未設置時預設 100。
**參數** `R`(int):加速度比例,取值範圍 [1,100]。
**回傳** `ErrorID,{},AccL(R);`
**範例** `AccL(50)`。

#### SpeedJ(隊列, 29999)
**原型** `SpeedJ(R)`
**描述** 設置關節運動方式的速度比例。僅本次 TCP/IP 模式生效,未設置時預設 100。
**參數** `R`(int):速度比例,取值範圍 [1,100]。
**回傳** `ErrorID,{},SpeedJ(R);`
**範例** `SpeedJ(50)`。

#### SpeedL(隊列, 29999)
**原型** `SpeedL(R)`
**描述** 設置直線和弧線運動方式的速度比例。僅本次 TCP/IP 模式生效,未設置時預設 100。
**參數** `R`(int):速度比例,取值範圍 [1,100]。
**回傳** `ErrorID,{},SpeedL(R);`
**範例** `SpeedL(50)`。

#### Arch(隊列, 29999)
**原型** `Arch(Index)`
**描述** 設置 Jump 運動的全局門型參數索引。呼叫 Jump 運動指令時若未指定,使用此全局值。僅本次 TCP/IP 模式生效,未設置時預設 0。
**參數** `Index`(int):門型參數索引(需先透過控制軟件設置)。
**回傳** `ErrorID,{},Arch(Index);`
**範例** `Arch(1)`。

#### CP(隊列, 29999)
**原型** `CP(R)`
**描述** 設置平滑過渡比例:機械臂連續運動經過多個點時,中間點以直角還是曲線過渡。**對 Jump 運動無效。** 僅本次 TCP/IP 模式生效,未設置時預設 0(不平滑過渡)。
**參數** `R`(unsigned int):平滑過渡比例,取值範圍 [0,100]。
**回傳** `ErrorID,{},CP(R);`
**範例** `CP(50)`。

#### SetArmOrientation(隊列, 29999)
**原型** `SetArmOrientation(LorR)`
**描述** 設置運動目標點的手系。目標點為笛卡爾坐標時,手系可確定機械臂唯一姿態。僅本次 TCP/IP 模式生效,未設置時不指定手系。**此指令為 M1 Pro 特有。**
**參數** `LorR`(int):0 = 左手系,1 = 右手系。
**回傳** `ErrorID,{},SetArmOrientation(LorR);`
**範例** `SetArmOrientation(1)`。

#### SetCollisionLevel(隊列, 29999)
**原型** `SetCollisionLevel(level)`
**描述** 設置碰撞檢測等級。僅本次 TCP/IP 模式生效,未設置時沿用進入模式前控制軟件設定值。
**參數** `level`(int):0 = 關閉碰撞檢測;1~5 數字越大靈敏度越高。
**回傳** `ErrorID,{},SetCollisionLevel(level);`
**範例** `SetCollisionLevel(1)`。

#### SetUser(立即, 29999)
**原型** `SetUser(index,table)`
**描述** 修改指定的用戶坐標系。控制器 **1.6.0+** 支持。
**必選參數**

| 參數名 | 類型 | 範圍 | 說明 |
|---|---|---|---|
| index | int | [0,9] | 用戶坐標系索引,坐標系 0 初始值為基坐標系 |
| table | — | — | 修改後的用戶坐標系 `{x,y,z,r}`,大括號可不帶,建議用 `CalcUser` 取得 |

**回傳** `ErrorID,{},SetUser(index,table);`
**範例** `SetUser(1,{10,10,10,0})` ≡ `SetUser(1,10,10,10,0)` 修改用戶坐標系 1 為 X=10,Y=10,Z=10,R=0。

#### CalcUser(立即, 29999)
**原型** `CalcUser(index,matrix_direction,table)`
**描述** 計算用戶坐標系。控制器 **1.6.0+** 支持。
**必選參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| index | int | 用戶坐標系索引 [0,9],坐標系 0 初始值為基坐標系 |
| matrix_direction | int | 計算方向。**1 = 左乘**(index 坐標系沿基坐標系偏轉 table 值);**0 = 右乘**(沿自己偏轉 table 值) |
| table | — | 用戶坐標系偏移值 `{x,y,z,r}`,大括號可不帶 |

**回傳** `ErrorID,{x,y,z,r},CalcUser(index,matrix_direction,table);`其中 `{x,y,z,r}` 為計算得出的用戶坐標系。
**範例** `CalcUser(1,1,{10,10,10,10})`(左乘);`CalcUser(1,0,{10,10,10,10})`(右乘)。

#### SetTool(立即, 29999)
**原型** `SetTool(index,table)`
**描述** 修改指定的工具坐標系。控制器 **1.6.0+** 支持。
**必選參數**

| 參數名 | 類型 | 範圍 | 說明 |
|---|---|---|---|
| index | int | [0,9] | 工具坐標系索引,坐標系 0 初始值為法蘭坐標系 |
| table | — | — | 修改後的工具坐標系 `{x,y,z,r}`,大括號可不帶,建議用 `CalcTool` 取得 |

**回傳** `ErrorID,{},SetTool(index,table);`
**範例** `SetTool(1,{10,10,10,0})` ≡ `SetTool(1,10,10,10,0)`。

#### CalcTool(立即, 29999)
**原型** `CalcTool(index,matrix_direction,table)`
**描述** 計算工具坐標系。控制器 **1.6.0+** 支持。
**必選參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| index | int | 工具坐標系索引 [0,9],坐標系 0 初始值為法蘭坐標系 |
| matrix_direction | int | 計算方向。**1 = 左乘**(沿法蘭坐標系偏轉);**0 = 右乘**(沿自己偏轉) |
| table | — | 工具坐標系偏移值 `{x,y,z,r}`,大括號可不帶 |

**回傳** `ErrorID,{x,y,z,r},CalcTool(index,matrix_direction,table);`其中 `{x,y,z,r}` 為計算得出的工具坐標系。
**範例** `CalcTool(1,1,{10,10,10,10})`(左乘);`CalcTool(1,0,{10,10,10,10})`(右乘)。

### 2.3 計算和獲取相關指令

#### RobotMode(立即, 29999)
**原型** `RobotMode()`
**描述** 獲取機器人當前狀態。
**回傳** `ErrorID,{Value},RobotMode();` —— Value 取值:

| 取值 | 定義 | 說明 |
|---|---|---|
| 1 | ROBOT_MODE_INIT | 初始化 |
| 2 | ROBOT_MODE_BRAKE_OPEN | 有任意關節的抱閘鬆開 |
| 3 | ROBOT_MODE_POWER_STATUS | 本體未上電 |
| 4 | ROBOT_MODE_DISABLED | 未使能(無抱閘鬆開) |
| 5 | ROBOT_MODE_ENABLE | 使能且空閒(未運行工程且無報警) |
| 6 | ROBOT_MODE_BACKDRIVE | 拖拽模式 |
| 7 | ROBOT_MODE_RUNNING | 運行狀態(軌跡複現/擬合中、執行運動命令中、工程運行中) |
| 8 | ROBOT_MODE_RECORDING | 軌跡錄製模式 |
| 9 | ROBOT_MODE_ERROR | 有未清除的報警。**此狀態優先級最高**,有報警時無論何狀態都返回 9 |
| 10 | ROBOT_MODE_PAUSE | 暫停狀態 |
| 11 | ROBOT_MODE_JOG | 點動中 |

#### GetAngle(立即, 29999)
**原型** `GetAngle()`
**描述** 獲取機械臂當前位姿的關節坐標。
**回傳** `ErrorID,{J1,J2,J3,J4},GetAngle();`

#### GetPose(立即, 29999)
**原型** `GetPose(User=0,Tool=0)`
**描述** 獲取機械臂當前位姿的笛卡爾坐標。
**參數**(均可選,不傳時用全局用戶/工具坐標系)`User`(int)已標定用戶坐標系索引;`Tool`(int)已標定工具坐標系索引。
**回傳** `ErrorID,{X,Y,Z,R},GetPose();`
**範例** `GetPose()`(全局);`GetPose(User=1,Tool=0)`。

#### GetErrorID(立即, 29999)
**原型** `GetErrorID()`
**描述** 獲取機器人當前報錯的錯誤碼。
**回傳** `ErrorID,{[[id,...,id], [id], [id], [id], [id], [id], [id]]},GetErrorID();`
- 第一組 `[id,...,id]`:控制器與演算法報警信息,無報警返回 `[]`,多個以逗號相隔。**碰撞檢測值為 -2**,其餘參考 `alarm_controller.json`。
- 後四組 `[id]`:機械臂四個伺服的報警信息,無報警返回 `[]`,參考 `alarm_servo.json`。

#### PositiveSolution(立即, 29999)
**原型** `PositiveSolution(J1,J2,J3,J4,User,Tool)`
**描述** 正解運算:給定各關節角度,計算機械臂末端在給定笛卡爾坐標系中的坐標值。
**參數**

| 參數名 | 類型 | 單位 | 說明 |
|---|---|---|---|
| J1~J4 | double | 度 | 各軸位置 |
| User | int | — | 已標定用戶坐標系索引 |
| Tool | int | — | 已標定工具坐標系索引 |

**回傳** `ErrorID,{x,y,z,r},PositiveSolution(J1,J2,J3,J4,User,Tool);`
**範例** `PositiveSolution(0,0,-90,0,1,1)`。

#### InverseSolution(立即, 29999)
**原型** `InverseSolution(X,Y,Z,R,User,Tool,isJointNear,JointNear)`
**描述** 逆解運算:給定末端笛卡爾坐標值,計算各關節角度。一個位姿可對應多個關節變量,故用指定關節坐標選最接近解。
**參數**

| 參數名 | 類型 | 單位 | 說明 |
|---|---|---|---|
| X,Y,Z | double | mm | 軸位置 |
| R | double | 度 | R 軸位置 |
| User | int | — | 已標定用戶坐標系索引 |
| Tool | int | — | 已標定工具坐標系索引 |
| isJointNear | int | — | 可選。0 或不帶 = JointNear 無效,按當前關節角就近選解;1 = 按 JointNear 就近選解 |
| JointNear | string | — | 可選。用於就近選解的關節坐標 |

**回傳** `ErrorID,{J1,J2,J3,J4},InverseSolution(X,Y,Z,R,User,Tool,isJointNear,JointNear);`
**範例** `InverseSolution(473,-141,469,-180,0,0)`;`InverseSolution(473,-141,469,-180,0,0,1,{0,0,-90,0})`。

#### PalletCreate(立即, 29999)
**原型** `PalletCreate(P1,P2,P3,P4,row=0,col=0,Palletname)`
**描述** 創建托盤。給定四角笛卡爾坐標點(P1~P4)與行列數,系統自動生成全部托盤點位。**最多 20 個托盤**,退出 TCP 模式時刪除所有托盤。控制器 **1.6.0+** 支持。
**參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| P1~P4 | array[double] | 四角笛卡爾坐標 `{X,Y,Z,R}` |
| row | int | 托盤行數 |
| col | int | 托盤列數 |
| Palletname | string | 托盤名稱,不可重複 |

**回傳** `ErrorID,{number},PalletCreate(P1,P2,P3,P4,row,col,Palletname);`其中 number 為已創建托盤數量。
**範例** `PalletCreate({56,-568,337,175.5755},{156,-568,337,175.5755},{156,-468,337,175.5755},{56,-468,337,175.5755},row=10,col=10,pallet1)`。

#### GetPalletPose(立即, 29999)
**原型** `GetPalletPose(Palletname,index)`
**描述** 獲取已創建托盤的指定點位。索引與點位對應參考 `PalletCreate`。控制器 **1.6.0+** 支持。
**參數** `Palletname`(string)托盤名稱;`index`(int)點位索引,**從 1 開始**。
**回傳** `ErrorID,{X,Y,Z,R},GetPalletPose(Palletname,index);`
**範例** `GetPalletPose(pallet1,5)`。

### 2.4 IO 相關指令

#### DO(隊列, 29999)
**原型** `DO(index,status)`
**描述** 設置數字輸出端口狀態(**隊列指令**)。
**參數** `index`(int)DO 端子編號;`status`(int)1 = 有信號,0 = 無信號。
**回傳** `ErrorID,{},DO(index,status);`
**範例** `DO(1,1)`。

#### DOExecute(立即, 29999)
**原型** `DOExecute(index,status)`
**描述** 設置數字輸出端口狀態(**立即指令**,無視指令隊列)。
**參數** `index`(int)DO 端子編號;`status`(int)1 = 有信號,0 = 無信號。
**回傳** `ErrorID,{},DOExecute(index,status);`
**範例** `DOExecute(1,1)`。

#### DOGroup(立即, 29999)
**原型** `DOGroup(index1,value1,index2,value2,...,indexN,valueN)`
**描述** 設置多個數字輸出端口狀態(**立即指令**)。
**參數** 成對的 `indexN`(int)DO 端子編號與 `valueN`(int)狀態(1 有信號 / 0 無信號)。
**回傳** `ErrorID,{},DOGroup(index1,value1,index2,value2,...,indexn,valuen);`
**範例** `DOGroup(4,1,6,0,2,1,7,0)` 設 DO4 有、DO6 無、DO2 有、DO7 無。

#### ToolDO(隊列, 29999)
**原型** `ToolDO(index,status)`
**描述** 設置末端數字輸出端口狀態(**隊列指令**)。
**參數** `index`(int)末端 DO 端子編號;`status`(int)1 有信號 / 0 無信號。
**回傳** `ErrorID,{},ToolDO(index,status);`
**範例** `ToolDO(1,1)`。

#### ToolDOExecute(立即, 29999)
**原型** `ToolDOExecute(index,status)`
**描述** 設置末端數字輸出端口狀態(**立即指令**,無視指令隊列)。
**參數** `index`(int)末端 DO 端子編號;`status`(int)1 有信號 / 0 無信號。
**回傳** `ErrorID,{}, ToolDOExecute(index,status);`
**範例** `ToolDOExecute(1,1)`。

#### DI(立即, 29999)
**原型** `DI(index)`
**描述** 獲取 DI 端口的狀態。
**參數** `index`(int)DI 端子編號。
**回傳** `ErrorID,{value},DI(index);` —— value:0 無信號,1 有信號。
**範例** `DI(1)`。

#### ToolDI(立即, 29999)
**原型** `ToolDI(index)`
**描述** 獲取末端 DI 端口的狀態。
**參數** `index`(int)末端 DI 端子編號。
**回傳** `ErrorID,{value},ToolDI(index);` —— value:0 無信號,1 有信號。
**範例** `ToolDI(1)`。

### 2.5 Modbus 相關指令

#### ModbusCreate(立即, 29999)
**原型** `ModbusCreate(ip,port,slave_id,isRTU)`
**描述** 創建 Modbus 主站並與從站建立連接。**最多同時連接 5 個設備。**
**參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| ip | string | 從站 IP 地址 |
| port | int | 從站端口 |
| slave_id | int | 從站 ID |
| isRTU | int | 可選。不帶或 0 = ModbusTCP;1 = ModbusRTU |

**回傳** `ErrorID,{index},ModbusCreate(ip,port,slave_id,isRTU);`
- ErrorID:0 創建成功,-1 創建失敗,其餘見通用錯誤碼。
- index:返回的主站索引(0~4),後續呼叫其他 Modbus 指令時使用。

**範例** `ModbusCreate(127.0.0.1,60000,1,1)`。

#### ModbusClose(立即, 29999)
**原型** `ModbusClose(index)`
**描述** 與 Modbus 從站斷開連接,釋放主站。
**參數** `index`(int)創建主站時返回的主站索引。
**回傳** `ErrorID,{},ModbusClose(index);`
**範例** `ModbusClose(0)`。

#### GetInBits(立即, 29999)
**原型** `GetInBits(index,addr,count)`
**描述** 讀取 Modbus 從站觸點寄存器(離散輸入)地址的值。
**參數** `index`(int)主站索引;`addr`(int)觸點寄存器起始地址;`count`(int)連續讀取數量,取值範圍 **1~16**。
**回傳** `ErrorID,{value1,value2,...,valuen},GetInBits(index,addr,count);`數量與 count 相同。
**範例** `GetInBits(0,3000,5)`。

#### GetInRegs(立即, 29999)
**原型** `GetInRegs(index,addr,count,valType)`
**描述** 按指定數據類型讀取 Modbus 從站輸入寄存器地址的值。
**參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| index | int | 主站索引 |
| addr | int | 輸入寄存器起始地址 |
| count | int | 連續讀取數量,取值範圍 **[1,4]** |
| valType | string | 可選。`U16`(空預設,16 位無號,占 1 寄存器)/ `U32`(32 位無號,占 2)/ `F32`(32 位單精度浮點,占 2)/ `F64`(64 位雙精度浮點,占 4) |

**回傳** `ErrorID,{value1,value2,...,valuen},GetInRegs(index,addr,count);`數量與 count 相同。
**範例** `GetInRegs(0,4000,3)`(值類型 U16)。

#### GetCoils(立即, 29999)
**原型** `GetCoils(index,addr,count)`
**描述** 讀取 Modbus 從站線圈寄存器地址的值。
**參數** `index`(int)主站索引;`addr`(int)線圈寄存器起始地址;`count`(int)連續讀取數量,取值範圍 **[1,16]**。
**回傳** `ErrorID,{value1,value2,...,valuen},GetCoils(index,addr,count);`數量與 count 相同。
**範例** `GetCoils(0,1000,3)`。

#### SetCoils(立即, 29999)
**原型** `SetCoils(index,addr,count,valTab)`
**描述** 將指定的值寫入線圈寄存器指定地址。
**參數** `index`(int)主站索引;`addr`(int)線圈寄存器起始地址;`count`(int)連續寫入數量,取值範圍 **[1,16]**;`valTab`(string)要寫入的值,數量與 count 相同。
**回傳** `ErrorID,{},SetCoils(index,addr,count,valTab);`
**範例** `SetCoils(0,1000,3,{1,0,1})`。

#### GetHoldRegs(立即, 29999)
**原型** `GetHoldRegs(index,addr, count,valType)`
**描述** 按指定數據類型讀取 Modbus 從站保持寄存器地址的值。
**參數** `index`(int)主站索引;`addr`(int)保持寄存器起始地址;`count`(int)連續讀取數量,取值範圍 **[1,4]**;`valType`(string)可選,`U16`/`U32`/`F32`/`F64`(同 `GetInRegs`)。
**回傳** `ErrorID,{value1,value2,...,valuen},GetHoldRegs(index,addr, count,valType);`數量與 count 相同。
**範例** `GetHoldRegs(0,3095,1)`(值類型 U16)。

#### SetHoldRegs(立即, 29999)
**原型** `SetHoldRegs(index,addr, count,valTab,valType)`
**描述** 按指定數據類型將指定的值寫入 Modbus 從站保持寄存器指定地址。
**參數** `index`(int)主站索引;`addr`(int)保持寄存器起始地址;`count`(int)連續寫入數量,取值範圍 **[1,4]**;`valTab`(string)要寫入的值,數量與 count 相同;`valType`(string)可選,`U16`/`U32`/`F32`/`F64`。
**回傳** `ErrorID,{},SetHoldRegs(index,addr, count,valTab,valType);`
**範例** `SetHoldRegs(0,3095,2,{6000,300}, U16)`。

---

## 2. 運動指令(30003 端口)

> 運動相關指令透過 **30003** 端口下發,**全部為隊列指令**。

### 通用說明(官方,PDF §3.1)

- **坐標系參數**:笛卡爾運動指令的可選參數 `User`/`Tool` 指定目標點的用戶/工具坐標系索引;不帶則用全局坐標系(見 [`User`](#user隊列-29999)/[`Tool`](#tool隊列-29999))。
- **速度參數**:可選參數 `SpeedJ`/`SpeedL`/`AccJ`/`AccL` 指定本指令的加速度/速度比例。實際比例 = 運動指令比例 × 控制軟件再現設定值 × 全局速率。未指定則用全局設置。
- **平滑過渡參數**:可選參數 `CP` 指定當前指令到下一條指令之間的平滑過渡比例。
- **使用限制**:TCP 運動指令**不支持**在可選參數中帶 `SYNC` 實現同步,請改用 [`Sync()`](#sync隊列-30003) 或 [`SyncAll()`](#syncall隊列-30003)。

#### MovJ(隊列, 30003)
**原型** `MovJ(X,Y,Z,R,User=index,Tool=index,SpeedJ=R,AccJ=R,CP=R)`
**描述** 從當前位置以**關節運動**方式運動至笛卡爾坐標目標點。軌跡非直線,所有關節同時完成運動。
**參數** `X`/`Y`/`Z`(double, mm)、`R`(double, 度)目標點位置;其餘為通用可選參數(見上)。
**回傳** `ErrorID,{},MovJ(X,Y,Z,R);`
**範例** `MovJ(-100,100,200,150,AccJ=50)` 以 50% 加速度關節運動至 {-100,100,200,150}。

#### MovL(隊列, 30003)
**原型** `MovL(X,Y,Z,R,User=index,Tool=index,SpeedL=R,AccL=R,CP=R)`
**描述** 從當前位置以**直線運動**方式運動至笛卡爾坐標目標點。
**參數** `X`/`Y`/`Z`(double, mm)、`R`(double, 度)目標點位置;其餘為通用可選參數。
**回傳** `ErrorID,{},MovL(X,Y,Z,R);`
**範例** `MovL(-100,100,200,150,SpeedL=60)` 以 60% 速度直線運動至 {-100,100,200,150}。

#### JointMovJ(隊列, 30003)
**原型** `JointMovJ(J1,J2,J3,J4,SpeedJ=R,AccJ=R,CP=R)`
**描述** 從當前位置以**關節運動**方式運動至**關節坐標**目標點。
**參數** `J1`~`J4`(double, 度)目標點各軸位置;其餘為通用可選參數。
**回傳** `ErrorID,{},JointMovJ(J1,J2,J3,J4);`
**範例** `JointMovJ(0,0,-90,0,SpeedJ=60,AccJ=50)`。

#### MovLIO(隊列, 30003)
**原型** `MovLIO(X,Y,Z,R,{Mode,Distance,Index,Status},...,{Mode,Distance,Index,Status},User=index,Tool=index,SpeedL=R,AccL=R,CP=R)`
**描述** 從當前位置以**直線運動**方式運動至笛卡爾目標點,運動時並行設置數字輸出端口狀態。
**參數** `X`/`Y`/`Z`(double, mm)、`R`(double, 度)目標點;`{Mode,Distance,Index,Status}` 為並行 DO 參數(可設多組):

| 子參數 | 類型 | 說明 |
|---|---|---|
| Mode | int | 觸發模式。0 = 距離百分比,1 = 距離數值 |
| Distance | int | 指定距離。正數 = 離起點距離,負數 = 離目標點距離;Mode=0 時為總距離百分比 (0,100];Mode=1 時為距離值(mm) |
| Index | int | DO 端子編號 |
| Status | int | 要設置的 DO 狀態,0 無信號,1 有信號 |

**回傳** `ErrorID,{},MovLIO(X,Y,Z,R,{Mode,Distance,Index,Status},...,SpeedL=R,AccL=R);`
**範例** `MovLIO(-100,100,200,150,{0,50,1,0})` 運動到 50% 距離時將 DO1 設為無信號。

#### MovJIO(隊列, 30003)
**原型** `MovJIO(X,Y,Z,R,{Mode,Distance,Index,Status},...,{Mode,Distance,Index,Status},User=index,Tool=index,SpeedJ=R,AccJ=R,CP=R)`
**描述** 從當前位置以**關節運動**方式運動至笛卡爾目標點,運動時並行設置數字輸出端口狀態。
**參數** 同 [`MovLIO`](#movlio隊列-30003)(並行 DO 參數 `{Mode,Distance,Index,Status}` 含義相同)。
**回傳** `ErrorID,{},MovJIO(X,Y,Z,R,{Mode,Distance,Index,Status},...);`
**範例** `MovJIO(-100,100,200,150,{0,50,1,0})`。

#### Arc(隊列, 30003)
**原型** `Arc(X1,Y1,Z1,R1,X2,Y2,Z2,R2,User=index,Tool=index,SpeedL=R,AccL=R,CP=R)`
**描述** 從當前位置以**圓弧插補**方式運動至目標點。需經由當前位置、圓弧中間點、目標點三點確定圓弧,因此**當前位置不能在 P1、P2 確定的直線上**。
**參數**

| 參數名 | 類型 | 單位 | 說明 |
|---|---|---|---|
| X1,Y1,Z1 | double | mm | 圓弧中間點位置 |
| R1 | double | 度 | 圓弧中間點 R |
| X2,Y2,Z2 | double | mm | 目標點位置 |
| R2 | double | 度 | 目標點 R |

**回傳** `ErrorID,{},Arc(X1,Y1,Z1,R1,X2,Y2,Z2,R2);`
**範例** `Arc(-350,-200,200,150,-300,-250,200,150)`。

#### Circle(隊列, 30003)
**原型** `Circle(count,{X1,Y1,Z1,R1},{X2,Y2,Z2,R2})`
**描述** 從當前位置進行**整圓插補**運動,運動指定圈數後回到當前位置。需經由當前位置、P1、P2 三點確定整圓,**當前位置不能在 P1、P2 確定的直線上**,且整圓不能超出運動範圍。
**參數**

| 參數名 | 類型 | 單位 | 說明 |
|---|---|---|---|
| count | int | — | 整圓運動圈數,≥1 的整數 |
| X1,Y1,Z1 | double | mm | P1 點位置 |
| R1 | double | 度 | P1 點 R |
| X2,Y2,Z2 | double | mm | P2 點位置 |
| R2 | double | 度 | P2 點 R |

**回傳** `ErrorID,{},Circle(count,{X1,Y1,Z1,R1},{X2,Y2,Z2,R2});`
**範例** `Circle(1,{-350,-200,200,150},{-300,-250,200,150})`。

#### MoveJog(隊列, 30003)
**原型** `MoveJog(axisID,CoordType=typeValue,User=index,Tool=index)`
**描述** 點動機械臂。下發後機械臂沿指定軸**持續點動**,需再下發 `MoveJog()`(空參數)停止;下發任意非指定 string 的 `MoveJog(string)` 也會停止。控制器 **1.5.6+** 支持。
**參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| axisID | string | 點動運動軸:`J1+`/`J1-`…`J4+`/`J4-`(關節正/負方向);`X+`/`X-`、`Y+`/`Y-`、`Z+`/`Z-`、`R+`/`R-`(笛卡爾軸正/負方向) |
| CoordType | int | 可選。僅當 axisID 指定笛卡爾軸時生效。0 = 用戶坐標系,1 = 工具坐標系 |

**回傳** `ErrorID,{},MoveJog(axisID,CoordType=typeValue,User=index,Tool=index);`
**範例** `MoveJog(j2-)` 沿 J2 負方向點動;`MoveJog()` 停止點動。

#### Sync(隊列, 30003)
**原型** `Sync()`
**描述** 阻塞程序執行隊列指令,待隊列**最後**的指令執行完後才返回。
**回傳** `ErrorID,{},Sync();`
**範例**
```
MovJ(x,y,z,r)
Sync()
RobotMode()   // 待運動到位後才取狀態
```

#### RelMovJUser(隊列, 30003)
**原型** `RelMovJUser(OffsetX,OffsetY,OffsetZ,OffsetR,User,SpeedJ=R,AccJ=R,Tool=Index,CP=R)`
**描述** 沿用戶坐標系進行**相對運動**,末端運動方式為**關節運動**。控制器 **1.5.6+** 支持。
**參數** `offsetX`/`offsetY`/`offsetZ`(double, mm)各軸偏移量;`offsetR`(double, 度)R 軸偏移量;`User`(int)選擇已標定的用戶坐標系索引;其餘為可選參數。
**回傳** `ErrorID,{},RelMovJUser(OffsetX,OffsetY,OffsetZ,OffsetR,User,SpeedJ=R,AccJ=R,Tool=Index);`
**範例** `RelMovJUser(10,10,10,0,0)` 沿用戶坐標系 0 在 X/Y/Z 各偏移 10mm。

#### RelMovLUser(隊列, 30003)
**原型** `RelMovLUser(OffsetX,OffsetY,OffsetZ,OffsetR,User,SpeedL=R,AccL=R,Tool=Index,CP=R)`
**描述** 沿用戶坐標系進行**相對運動**,末端運動方式為**直線運動**。控制器 **1.5.6+** 支持。
**參數** `offsetX`/`offsetY`/`offsetZ`(double, mm)各軸偏移量;`offsetR`(double, 度)R 軸偏移量;`User`(int)選擇已標定的用戶坐標系索引;其餘為可選參數。
**回傳** `ErrorID,{},RelMovLUser(OffsetX,OffsetY,OffsetZ,OffsetR,User);`
**範例** `RelMovLUser(10,10,10,0,0)` 沿用戶坐標系 0 在 X/Y/Z 各偏移 10mm。

#### RelJointMovJ(隊列, 30003)
**原型** `RelJointMovJ(Offset1,Offset2,Offset3,Offset4,SpeedJ=R,AccJ=R,CP=R)`
**描述** 沿關節坐標系進行**相對運動**,末端運動方式為**關節運動**。控制器 **1.5.6+** 支持。
**參數** `offset1`~`offset4`(double, 度)各關節軸偏移量;其餘為可選參數。
**回傳** `ErrorID,{},RelJointMovJ(Offset1,Offset2,Offset3,Offset4);`
**範例** `RelJointMovJ(10,10,10,0)` J1/J2/J3 各偏移 10 度。

#### MovJExt(隊列, 30003)
**原型** `MovJExt(Angle|Distance,SpeedE=50,AccE=50,Sync=1)`
**描述** 控制滑軌(擴展軸)運動到目標角度或位置。
**參數**

| 參數名 | 類型 | 說明 |
|---|---|---|
| Angle / Distance | float | 運動的目標角度或距離。含義取決於擴展軸工藝高級設置中的運動類型:關節時為度,直線時為毫米 |
| SpeedE | int | 可選。運動速度比例 1~100,預設 100 |
| AccE | int | 可選。運動加速度比例 1~100,預設 100 |
| Sync | int | 可選。同步標識 0 或 1,預設 0。0 = 異步(立即返回);1 = 同步(待執行完才返回) |

**回傳** `ErrorID,{},MovJExt(Angle|Distance,SpeedE=50,AccE=50,Sync=1);`
**範例** `MovJExt(300)`(若運動類型為毫米,擴展軸運動至 300mm)。

#### SyncAll(隊列, 30003)
**原型** `SyncAll()`
**描述** 阻塞程序執行隊列指令,待隊列中**所有**指令執行完後才返回。主要用於有擴展軸的場景:擴展軸與機械臂各自獨立運動,`Sync` 在隊列最後一條指令執行完就返回,前面的擴展軸指令可能未執行完;要確保全部執行完用 `SyncAll`。
**回傳** `ErrorID,{},SyncAll();`
**範例**
```
MovJ(x1,y1,z1,r1)
MovJExt(distance)
MovJ(x2,y2,z2,r2)
SyncAll()
RobotMode()   // 待機械臂與擴展軸都完成運動後才取狀態
```

---

## 3. 實時反饋信息(官方,PDF §4)

控制器透過 **30004 / 30005 / 30006** 端口實時反饋機器人狀態信息。每包 **1440 字節**,以標準格式排列。

> **位元組序**:資料以**小端**(低位優先)儲存。例:值 1234 = `0000 0100 1101 0010`,以兩字節傳遞時第一字節 `1101 0010`(低 8 位)、第二字節 `0000 0100`(高 8 位)。

### 1440-byte 封包欄位表

| 含義 | 數據類型 | 數量 | 字節大小 | 字節位置 | 描述 |
|---|---|---|---|---|---|
| MessageSize | unsigned short | 1 | 2 | 0000~0001 | 消息字節總長度 |
| N/A | unsigned short | 3 | 6 | 0002~0007 | 保留位 |
| DigitalInputs | uint64 | 1 | 8 | 0008~0015 | 當前數字輸入端子狀態(見 DI/DO 說明) |
| DigitalOutputs | uint64 | 1 | 8 | 0016~0023 | 當前數字輸出端子狀態(見 DI/DO 說明) |
| RobotMode | uint64 | 1 | 8 | 0024~0031 | 機器人模式(見 [`RobotMode`](#robotmode立即-29999) 指令) |
| TimeStamp | uint64 | 1 | 8 | 0032~0039 | Unix 時間戳(單位 ms) |
| N/A | uint64 | 1 | 8 | 0040~0047 | 保留位 |
| **TestValue** | uint64 | 1 | 8 | 0048~0055 | 內存結構測試標準值 **0x0123 4567 89AB CDEF** |
| N/A | double | 1 | 8 | 0056~0063 | 保留位 |
| SpeedScaling | double | 1 | 8 | 0064~0071 | 速度比例 |
| N/A | double | 1 | 8 | 0072~0079 | 保留位 |
| VMain | double | 1 | 8 | 0080~0087 | 控制板電壓 |
| VRobot | double | 1 | 8 | 0088~0095 | 機器人電壓 |
| IRobot | double | 1 | 8 | 0096~0103 | 機器人電流 |
| N/A | double | 1 | 8 | 0104~0111 | 保留位 |
| N/A | double | 1 | 8 | 0112~0119 | 保留位 |
| N/A | double | 3 | 24 | 0120~0143 | 保留位 |
| N/A | double | 3 | 24 | 0144~0167 | 保留位 |
| N/A | double | 3 | 24 | 0168~0191 | 保留位 |
| QTarget | double | 6 | 48 | 0192~0239 | 目標關節位置 |
| QDTarget | double | 6 | 48 | 0240~0287 | 目標關節速度 |
| QDDTarget | double | 6 | 48 | 0288~0335 | 目標關節加速度 |
| ITarget | double | 6 | 48 | 0336~0383 | 目標關節電流 |
| MTarget | double | 6 | 48 | 0384~0431 | 目標關節扭矩 |
| QActual | double | 6 | 48 | 0432~0479 | 實際關節位置 |
| QDActual | double | 6 | 48 | 0480~0527 | 實際關節速度 |
| IActual | double | 6 | 48 | 0528~0575 | 實際關節電流 |
| ActualTCPForce | double | 6 | 48 | 0576~0623 | 保留位 |
| ToolVectorActual | double | 6 | 48 | 0624~0671 | TCP 笛卡爾實際坐標值 |
| TCPSpeedActual | double | 6 | 48 | 0672~0719 | TCP 笛卡爾實際速度值 |
| TCPForce | double | 6 | 48 | 0720~0767 | TCP 力值(通過關節電流計算) |
| ToolVectorTarget | double | 6 | 48 | 0768~0815 | TCP 笛卡爾目標坐標值 |
| TCPSpeedTarget | double | 6 | 48 | 0816~0863 | TCP 笛卡爾目標速度值 |
| MotorTemperatures | double | 6 | 48 | 0864~0911 | 關節溫度 |
| JointModes | double | 6 | 48 | 0912~0959 | 關節控制模式,8 = 位置模式,10 = 力矩模式 |
| VActual | double | 6 | 48 | 0960~1007 | 關節電壓 |
| HandType | char | 4 | 4 | 1008~1011 | 手系(見 [`SetArmOrientation`](#setarmorientation隊列-29999)) |
| User | char | 1 | 1 | 1012 | 用戶坐標系 |
| Tool | char | 1 | 1 | 1013 | 工具坐標系 |
| RunQueuedCmd | char | 1 | 1 | 1014 | 演算法隊列運行標誌 |
| PauseCmdFlag | char | 1 | 1 | 1015 | 演算法隊列暫停標誌 |
| VelocityRatio | char | 1 | 1 | 1016 | 關節速度比例(0~100) |
| AccelerationRatio | char | 1 | 1 | 1017 | 關節加速度比例(0~100) |
| JerkRatio | char | 1 | 1 | 1018 | 關節加加速度比例(0~100) |
| XYZVelocityRatio | char | 1 | 1 | 1019 | 笛卡爾位置速度比例(0~100) |
| RVelocityRatio | char | 1 | 1 | 1020 | 笛卡爾姿態速度比例(0~100) |
| XYZAccelerationRatio | char | 1 | 1 | 1021 | 笛卡爾位置加速度比例(0~100) |
| RAccelerationRatio | char | 1 | 1 | 1022 | 笛卡爾姿態加速度比例(0~100) |
| XYZJerkRatio | char | 1 | 1 | 1023 | 笛卡爾位置加加速度比例(0~100) |
| RJerkRatio | char | 1 | 1 | 1024 | 笛卡爾姿態加加速度比例(0~100) |
| BrakeStatus | char | 1 | 1 | 1025 | 機器人抱閘狀態(見 BrakeStatus 說明) |
| EnableStatus | char | 1 | 1 | 1026 | 機器人使能狀態 |
| DragStatus | char | 1 | 1 | 1027 | 機器人拖拽狀態 |
| RunningStatus | char | 1 | 1 | 1028 | 機器人運行狀態 |
| ErrorStatus | char | 1 | 1 | 1029 | 機器人報警狀態 |
| JogStatusCR | char | 1 | 1 | 1030 | 機器人點動狀態 |
| RobotType | char | 1 | 1 | 1031 | 機器人型號(見 RobotType 說明) |
| DragButtonSignal | char | 1 | 1 | 1032 | 保留位 |
| EnableButtonSignal | char | 1 | 1 | 1033 | 保留位 |
| RecordButtonSignal | char | 1 | 1 | 1034 | 保留位 |
| ReappearButtonSignal | char | 1 | 1 | 1035 | 保留位 |
| JawButtonSignal | char | 1 | 1 | 1036 | 保留位 |
| SixForceOnline | char | 1 | 1 | 1037 | 保留位 |
| N/A | char | 1 | 82 | 1038~1119 | 保留位 |
| MActual[6] | double | 6 | 48 | 1120~1167 | 四個關節的實際扭矩 |
| Load | double | 1 | 8 | 1168~1175 | 末端負載重量(kg) |
| CenterX | double | 1 | 8 | 1176~1183 | 末端負載 X 方向偏心距離(mm) |
| CenterY | double | 1 | 8 | 1184~1191 | 末端負載 Y 方向偏心距離(mm) |
| CenterZ | double | 1 | 8 | 1192~1199 | 末端負載 Z 方向偏心距離(mm) |
| User[6] | double | 6 | 48 | 1200~1247 | 用戶坐標系坐標值 |
| Tool[6] | double | 6 | 48 | 1248~1295 | 工具坐標系坐標值 |
| TraceIndex | double | 1 | 8 | 1296~1303 | 軌跡複現運行索引 |
| SixForceValue[6] | double | 6 | 48 | 1304~1351 | 保留位 |
| TargetQuaternion[4] | double | 4 | 32 | 1352~1383 | [qw,qx,qy,qz] 目標四元數 |
| ActualQuaternion[4] | double | 4 | 32 | 1384~1415 | [qw,qx,qy,qz] 實際四元數 |
| N/A | char | 1 | 24 | 1416~1440 | 保留位 |
| **TOTAL** | | | **1440** | | 1440 byte package |

> **對接註記**:`TestValue`(偏移 0048~0055)固定為 magic `0x0123456789ABCDEF`,可用於對齊/校驗封包邊界。專案以此驗證 30004 串流(見 `PROGRESS.md` 反饋相關發現)。

### DI/DO 說明

DI/DO 各占 8 字節,每字節 8 位,最大可表示各 64 個端口狀態。每字節從低到高每一位表示一個端子:1 = 有信號,0 = 無信號或無對應端子。
例:第一字節 `0x01`(00000001)、其餘全 0 → DI1 為 1,其餘為 0。

### BrakeStatus 說明

該字節按位表達各關節抱閘狀態,對應位為 1 表示該關節抱閘已鬆開:

| 位 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|---|
| 含義 | 保留 | 保留 | 關節1 | 關節2 | 關節3 | 關節4 | 保留 | 保留 |

例:`0x04`(00000100)→ 關節 4 抱閘鬆開。
> 原文示例文字寫「0x03(00000100):關節4抱閘鬆開」,其二進位 `00000100` 對應 bit2 = 關節 4,以二進位為準。

### RobotType 說明

| 取值 | 代表機型 |
|---|---|
| 1 | MG400 |
| 2 | M1 Pro |
| **4** | M1 Pro(支持 RS485 功能)←(1.7.0.0 新增) |

---

## 4. 通用錯誤碼(官方,PDF §5)

| 錯誤碼 | 描述 | 備註 |
|---|---|---|
| 0 | 無錯誤 | 下發成功 |
| -1 | 沒有獲取成功 | 命令接收失敗 / 執行失敗 |
| … | … | … |
| -10000 | 命令錯誤 | 下發的命令不存在 |
| -20000 | 參數數量錯誤 | 下發命令中的參數數量錯誤 |
| -30001 | 第 1 個參數的參數類型錯誤 | -30000 表示參數類型錯誤,最後一位 1 表示第 1 個參數 |
| -30002 | 第 2 個參數的參數類型錯誤 | -30000 表示參數類型錯誤,最後一位 2 表示第 2 個參數 |
| … | … | (依此類推,`-3000N` = 第 N 個參數類型錯誤) |
| -40001 | 第 1 個參數的參數範圍錯誤 | -40000 表示參數範圍錯誤,最後一位 1 表示第 1 個參數 |
| -40002 | 第 2 個參數的參數範圍錯誤 | -40000 表示參數範圍錯誤,最後一位 2 表示第 2 個參數 |
| … | … | (依此類推,`-4000N` = 第 N 個參數範圍錯誤) |

> **對接註記**:錯誤碼採區段編碼 —— `-3000N`(類型錯誤)、`-4000N`(範圍錯誤)的最後一位即出錯參數的序號(從 1 起)。控制器/演算法與伺服的詳細報警另見 `alarm_controller.json`、`alarm_servo.json`(見 [`GetErrorID`](#geterrorid立即-29999))。
