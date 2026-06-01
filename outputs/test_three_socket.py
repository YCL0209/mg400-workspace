"""T7B 診斷:還原 reference fork 的「全套 connect」順序。

背景觀察:reference ui.py / PythonExample.py / main.py 全部 connect 時開三個 port:
    29999 (dashboard) + 30003 (move) + 30004 (feedback)
我們 workbench 只開兩個:29999 + 30004,沒開 30003。

這版韌體可能在「client 連完三個埠」才把 dashboard interface 完整掛載 —— 少開
30003 → controller 不認 client 完整 → dashboard 指令全 -10000。

這個 script 還原 reference 的三 port 開法,然後送 RobotMode 看能不能通。

預期結果:
- `b'0,{5},RobotMode();'` → **30003 是關鍵** → workbench 要改:啟動時也連 30003
- `b'-10000,{},'`        → 三 port 也不行,問題不在 port count → 看別的地方

跟 test_dual_socket.py 對照跑,兩個結果一起貼回來。

用法:
    python outputs/test_three_socket.py
"""
import socket
import time

ARM_IP = "192.168.1.6"

print("Step 1: open dashboard 29999 (reference 順序第 1)")
dash = socket.socket()
dash.settimeout(5.0)
dash.connect((ARM_IP, 29999))

print("Step 2: open move 30003 (reference 順序第 2 — workbench 漏的這個)")
move = socket.socket()
move.settimeout(5.0)
move.connect((ARM_IP, 30003))

print("Step 3: open feedback 30004 (reference 順序第 3)")
feed = socket.socket()
feed.settimeout(5.0)
feed.connect((ARM_IP, 30004))

# 讀一筆 feedback frame
try:
    fb = feed.recv(1440)
    print(f"  feedback got {len(fb)} bytes (one frame)")
except Exception as e:
    print(f"  feedback recv error: {e}")

print("Step 4: 等 1 秒(模擬 user 思考)")
time.sleep(1)

print("Step 5: 透過 dashboard 送 RobotMode();")
dash.sendall(b"RobotMode();")
reply = dash.recv(4096)
print(f"  Reply: {reply!r}")

dash.close()
move.close()
feed.close()
print("Done.")
