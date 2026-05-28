# kinematics 層規則（子層，補充根 CLAUDE.md）

純數學層：關節角 ↔ 笛卡爾。與 transport/state 平行、互不依賴。

## 依賴與分層
- **無硬體依賴**：不開 socket、不連 dashboard、不下運動指令、不 import transport/state。
- 純 stdlib：FK 與校驗只用 `math`／`dataclasses`／`json`，**不依賴 numpy**（離線、可在任何 Python 跑）。
- 機構參數一律從 `config/kinematics.json` 讀，**不在程式裡寫死幾何值**。

## 機構模型（最關鍵，別搞錯）
- MG400 是**平行四連桿**4 軸：J2/J3 各自設定該連桿的**絕對角**、連桿讓法蘭恆鉛直。
  **不是** serial 6-DOF DH 鏈，別套 DH。
- θ2=J2（從鉛直起算）、θ3=J3（從水平起算）：
  `rho = base_r + L1·sin(θ2) + L2·cos(θ3)`；`z = base_z + L1·cos(θ2) − L2·sin(θ3)`；
  `x = rho·cos(J1)`；`y = rho·sin(J1)`；`r = J1 + J4`（**不 wrap**，可超過 ±180）。
- 原點在 J1 轉軸上、+X 正前方、+Z 向上；pose 量法蘭中心（無 TCP 偏移）。

## 參數來源與校驗
- 連桿參數是用 10 筆**實測** (joint, pose) 配對最小平方反推（殘差 < 0.01mm），
  存於 config，附 provenance。非 datasheet——機構/零點/量測基準變了就要**重新校驗**。
- 校驗介面 `evaluate(samples)` 輸出逐軸誤差 + max/mean，是判斷參數對不對的依據。

## 不做（範圍外）
- **不驗證/不夾關節範圍**（理論範圍與實機 J2/J3 耦合限制屬 Phase 2b safety）。FK 只算數。
- 逆運動學（IK）、軌跡、TCP 偏移：之後的里程碑，本層先不臆測。
