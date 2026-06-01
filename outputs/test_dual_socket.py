"""T7B 診斷:還原 workbench 「同時開 29999 + 30004」場景。

背景(2026-06-01):workbench 透過 dashboard 29999 送 `RobotMode();` 回 -10000,
但 raw socket 一次性連線送同樣 bytes 回 `0,{5},RobotMode();` 成功。
唯一差別是 workbench 還同時開了 feedback 30004 連線。

這個 script 重現 workbench 的 connect 順序:dashboard → feedback → 等 → 送 dashboard 指令。
看 dashboard 第一個指令會不會就被拒。

預期結果:
- 回 `b'0,{5},RobotMode();'` → 多 socket 並存不是元兇,嫌疑轉向 workbench 自己 code 某處
- 回 `b'-10000,{},'`        → 並存就是元兇,要改連線模型(延遲開 feedback 或第一個指令後再開)
- 別的                       → 貼回來看

用法:
    python outputs/test_dual_socket.py

注意:跑前先把 workbench / DobotStudio Pro / reference demo 全關掉。
"""
import socket
import time

ARM_IP = "192.168.1.6"

print("Step 1: open dashboard 29999")
dash = socket.socket()
dash.settimeout(5.0)
dash.connect((ARM_IP, 29999))

print("Step 2: open feedback 30004 (像 workbench 那樣)")
feed = socket.socket()
feed.settimeout(5.0)
feed.connect((ARM_IP, 30004))

# 讀一筆 feedback frame(workbench 也會這麼做)
try:
    fb = feed.recv(1440)
    print(f"  feedback got {len(fb)} bytes (one frame)")
except Exception as e:
    print(f"  feedback recv error: {e}")

print("Step 3: 等 1 秒(模擬 user 思考的延遲)")
time.sleep(1)

print("Step 4: 透過 dashboard 送 RobotMode();")
dash.sendall(b"RobotMode();")
reply = dash.recv(4096)
print(f"  Reply: {reply!r}")

dash.close()
feed.close()
print("Done.")
