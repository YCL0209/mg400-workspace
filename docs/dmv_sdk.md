# DMV-SDK 整合說明

> 本文件說明本專案如何封裝**台達(Delta)DMV-SDK** 工業相機。
> 對應程式碼:[`robot_core/camera/__init__.py`](../robot_core/camera/__init__.py)(`DeltaCamera` class)。
> Forked 自 phase5-panel 並改走 logging、加 Mac-safe import guard;原始碼結構不變,公開 API 完全相容。

---

## 1. 這是什麼 / 不是什麼

| | 說明 |
|---|---|
| **DMV-SDK 本體** | 台達**閉源**機器視覺相機 SDK,Python 模組名 `DmvSDK`。**隨台達 DMV 軟體安裝**,只存在於接實機相機的 **Windows** 機器上。**不在 repo 裡、也無法下載產生。** |
| **本專案的東西** | `camera.py` 的 `DeltaCamera` —— 把 DmvSDK 那串底層流程包成簡單 API 的封裝層。 |
| **輸出格式** | 一律回傳 **RGB numpy array `(H, W, 3)`, `dtype=uint8`**,可直接餵下游模型(灰階也會展成三通道)。 |

⚠️ **import 大小寫敏感**:模組是 `DmvSDK`,不是 `dmvsdk`(README 工程紀律第 74 行)。

⚠️ **平台限制**:Mac 開發機**沒有** DmvSDK,`import DmvSDK`(`camera.py:12`)在 Mac 會直接失敗。相機相關流程只能在 Win 實機跑(對照 README「跨平台分工」)。

---

## 2. `DeltaCamera` API 速查

```python
from camera import DeltaCamera
```

| 方法 | 用途 | 回傳 | 失敗行為 |
|---|---|---|---|
| `open()` | 連接相機(7 步流程前 5 步) | — | 找不到相機 → `RuntimeError` |
| `close()` | 關閉相機、釋放 SDK 系統 | — | — |
| `grab_one_rgb(timeout_ms=3000)` | **單張**拍照 | RGB `np.ndarray (H,W,3)` | 取像失敗 / 影像不完整 → `RuntimeError` |
| `start_continuous()` | 切連續模式 + 啟動串流 | — | 未 open → `RuntimeError` |
| `grab_continuous_rgb(timeout_ms=1000)` | **連續**抓一幀 | RGB `np.ndarray` 或 `None` | timeout / 影像不完整 → 回 `None`(交上層處理) |
| `stop_continuous()` | 停止連續串流 | — | — |
| `__enter__` / `__exit__` | context manager | — | 進出自動 `open()` / `close()` |

> 單張模式遇錯 **raise**;連續模式遇錯 **回 `None`** —— 這是兩種模式刻意的差異(連續迴圈不該被單幀失敗打斷)。

---

## 3. 七步取像流程

對照 `camera.py:25-106`。

### `open()` — 步驟 1~5

| 步驟 | 動作 | 關鍵 SDK 呼叫 |
|---|---|---|
| 1 | 建立 SDK 系統 | `DcSystemCreate()` |
| 2 | 取得第一台裝置 | `DcSystemGetDevice(system, None)` → `None` 就拋錯 |
| 3 | 開啟裝置(**獨佔模式**) | `DcDeviceOpen(device, DC_DEVICE_ACCESS_TYPE_CONTROL)` |
| 4 | 設單張模式 + 關 trigger | `DcNodeListSetValue(..., "AcquisitionMode", "SingleFrame")`、`DcNodeListSetSelectedValue(..., "TriggerMode", "Off")` |
| 5 | 準備 data stream + buffer | `DcDeviceGetDataStream` → `DcDataStreamAllocAndAnnounceBuffer` → `DcDataStreamQueueBuffer` |

### `grab_one_rgb()` — 步驟 6~7

| 步驟 | 動作 | 關鍵 SDK 呼叫 |
|---|---|---|
| 6 | 開始 → 等填滿 → 停止 | `DcDataStreamStartAcquisition` → `DcDataStreamGetFilledBuffer(timeout_ms)` → `DcDataStreamStopAcquisition` |
| 6.5 | 完整性檢查 | `DcBufferIsComplete(buffer)` → `False` 表示傳輸丟封包,拋錯 |
| 7 | 取影像 → 轉 numpy | `DcBufferGetImage` → `DcImageGetWidth/Height/PixelFormat` → 轉 RGB(見下節) |

---

## 4. DMV-SDK API ↔ 封裝對照

| DMV-SDK 呼叫 | 出現在 | 大致行號 |
|---|---|---|
| `DcSystemCreate` | `open()` 步驟 1 | `camera.py:28` |
| `DcSystemGetDevice` | `open()` 步驟 2 | `camera.py:31` |
| `DcDeviceOpen` | `open()` 步驟 3 | `camera.py:37` |
| `DcDeviceGetInfo` | `open()`(印相機名) | `camera.py:39` |
| `DcDeviceGetRemoteNodeList` / `DcNodeListSetValue` / `DcNodeListSetSelectedValue` | `open()` 步驟 4、`start_continuous()` | `camera.py:45-49`、`116-120` |
| `DcDeviceGetDataStream` / `DcDataStreamAllocAndAnnounceBuffer` / `DcDataStreamQueueBuffer` | `open()` 步驟 5、`start_continuous()` | `camera.py:52-54`、`123-125` |
| `DcDataStreamStartAcquisition` / `DcDataStreamStopAcquisition` | 步驟 6、`start/stop_continuous` | `camera.py:64,74`、`128,170` |
| `DcDataStreamGetFilledBuffer` | 步驟 6、連續抓幀 | `camera.py:67`、`134` |
| `DcBufferIsComplete` | 完整性檢查 | `camera.py:76`、`140` |
| `DcBufferGetImage` | 步驟 7 | `camera.py:80`、`145` |
| `DcImageGetWidth/Height/PixelFormat` / `DcPixelFormatToString` | 步驟 7 | `camera.py:82-87` |
| `DcImageCreate` / `DcImageConvertFormat` / `DcImageGetData` / `DcImageDestroy` | 彩色轉換 | `camera.py:98-104` |
| `Mono8` / `BGR8`(常數) | 像素格式判斷 | `camera.py:90,99` |
| `DcSystemDestroy` | `_cleanup()` | `camera.py:178` |

---

## 5. 像素格式處理

對照 `camera.py:89-104`。無論相機原生格式為何,**封裝層一律回傳 RGB 三通道**,讓下游模型統一處理。

- **灰階(`Mono8`)**
  ```
  DcImageGetData → np.array(uint8).reshape(H, W)
  → np.stack([arr, arr, arr], axis=-1)   # 單通道複製成三通道
  ```
- **彩色(其他格式)**
  ```
  DcImageCreate() → DcImageConvertFormat(image, image2, BGR8)
  → DcImageGetData → reshape(H, W, 3)     # 得 BGR
  → arr_bgr[:, :, ::-1].copy()            # BGR → RGB(numpy 反序)
  → DcImageDestroy(image2)                # 釋放暫存 image
  ```

> 彩色路徑會 `DcImageCreate` 一個暫存 `image2`,用完務必 `DcImageDestroy`,否則記憶體洩漏。

---

## 6. 單張 vs 連續模式

| | 單張(`grab_one_rgb`) | 連續(`grab_continuous_rgb`) |
|---|---|---|
| AcquisitionMode | `SingleFrame` | `Continuous` |
| buffer 數量 | 1 個 | 4 個(連續建議 4~8) |
| 啟停時機 | **每張** start → grab → stop | `start_continuous()` **一次**啟動,結束才 `stop_continuous()` |
| buffer 歸還 | 不需(每張重來) | **每幀必須 `DcDataStreamQueueBuffer` 歸還** |
| 遇錯 | `RuntimeError` | 回 `None` |

⭐ **連續模式關鍵**(`camera.py:164`):每抓完一幀,一定要 `DcDataStreamQueueBuffer(buffer)` 把 buffer 歸還給 SDK 重複用。**不歸還 → buffer 耗盡 → 串流卡死。**

---

## 7. 工程地雷 / 紀律

- **import 大小寫**:`DmvSDK` 不是 `dmvsdk`(README 第 74 行)。
- **獨佔模式**:`DC_DEVICE_ACCESS_TYPE_CONTROL` → 同一時間**只能一個程序**開相機;別的程式(或上次沒關乾淨)佔著就連不上。
- **色彩空間**:封裝回傳 **RGB**;用 OpenCV 顯示要記得轉 **BGR**(README 第 77 行)。
- **一定要關**:用 `with DeltaCamera()` 或手動 `close()`,觸發 `DcSystemDestroy` 釋放系統,否則相機被佔住、下次開不起來。
- **連續 buffer 要歸還**(見上節)。
- **完整性檢查**:`DcBufferIsComplete` 為 `False` 代表傳輸丟封包 —— 單張會拋錯,連續會丟棄該幀並歸還 buffer。

---

## 8. 使用範例

### 單張拍照存 PNG(對應 `camera.py:196-218` 的 `__main__`)

```python
from pathlib import Path
from PIL import Image
from camera import DeltaCamera

with DeltaCamera() as cam:          # 自動 open() / close()
    rgb = cam.grab_one_rgb()        # RGB np.ndarray (H, W, 3)

Image.fromarray(rgb).save(Path("camera_output/first_shot.png"))
```

直接跑封裝自帶的測試:

```bash
python camera.py        # 拍一張存到 camera_output/first_shot.png
```

### 連續取像(即時迴圈)

```python
from camera import DeltaCamera

cam = DeltaCamera()
cam.open()
cam.start_continuous()
try:
    while True:
        rgb = cam.grab_continuous_rgb(timeout_ms=1000)
        if rgb is None:             # timeout / 丟幀 → 跳過
            continue
        ...                         # 餵下游模型 / 顯示(顯示記得 RGB→BGR)
finally:
    cam.stop_continuous()
    cam.close()
```
