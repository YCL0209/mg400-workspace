# state 層規則（子層，補充根 CLAUDE.md）

state 把 transport 的 feedback 串流變成「可觀察、可訂閱」的機器人狀態。

## 依賴與分層
- **只依賴 transport**（`AsyncFeedbackStream`、`parse_feedback`、`FeedbackFrame`）。
  不得 import protocol / safety / controller / UI，不得反向依賴。
- socket I/O 與驗框／解析屬 transport；state 不自己開 socket、不重寫解析（重用純函式）。

## 並行與通知
- 單一 asyncio event loop。`RobotState.update()` 由 consumer task 呼叫，subscriber
  同步在同一 loop 內執行 → **subscriber 必須快、不可阻塞**（要做重活請自己 `create_task`）。
- **邊緣觸發**：只在被監看欄位（`WATCHED_FIELDS`）真的變化時通知，不要每幀都發
  （30004 高頻，每幀通知會洗版 / 拖垮上層）。
- 通知機制單一：**callback 訂閱**（`subscribe(cb) -> unsubscribe`）。`wait_for_change`
  等便利方法一律疊在 callback 上，不要另立第二套機制。禁止全域變數 + busy-wait。

## 串流處理
- **排空到最新幀**：producer 持續讀、`_LatestFrameSlot` 只留最新；consumer 落後就跳到
  最新狀態，不啃陳舊積壓。被略過的幀要計數（`stale_frame_count`）。
- **無效幀**：驗框失敗計數並略過（不為單一壞幀斷線）；連線中斷才重連並計數。
- **生命週期**：顯式 `start()/stop()` 或 async context manager。`stop()` 必須 cancel 背景
  task、接住 `CancelledError`、關閉 socket——不得殘留 task 或洩漏 socket。用 logging，不 print。

## 未來（尚未做，先記著）
- 「指令入列」vs「動作完成」是兩個不同事件（Phase 2 motion 才需要；本層先不臆測）。
- pose 邊緣偵測用 per-component 容差（`DEFAULT_POSE_DEADBAND`，可由 `RobotState(pose_deadband=...)`
  覆寫）；整數欄位精確比較。目前單一容差含混 mm 與 deg，若需要再拆 per-axis。
