"""T7B 診斷:單獨開 dashboard 29999、等 1 秒、送 RobotMode。

對照組 — 排除「idle timeout」假設。workbench 連線後等用戶輸入會有秒級延遲,
這個 script 還原同樣的「開 + idle + 送」流程,但只開 dashboard 一個 port。

預期結果:
- 回 `b'0,{5},RobotMode();'` → idle 不是問題,單獨 dashboard 1 秒延遲 OK
  → 跟 test_dual_socket 對照,若那個失敗、這個成功,確認「並存」是 trigger
- 回 `b'-10000,{},'`        → idle 也會壞,問題更廣

用法:
    python outputs/test_dashboard_only_idle.py

跟 test_dual_socket.py 配對跑,兩個結果一起貼回來才能定位。
"""
import socket
import time

ARM_IP = "192.168.1.6"

s = socket.socket()
s.settimeout(5.0)
s.connect((ARM_IP, 29999))
print("Dashboard 29999 connected, sleep 1s to mimic user delay...")
time.sleep(1)

s.sendall(b"RobotMode();")
reply = s.recv(4096)
print(f"Reply: {reply!r}")

s.close()
print("Done.")
