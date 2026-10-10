#!/usr/bin/env python3
# Runs Zephyr shell commands on the kit's console and prints the replies.
#   shell.py "net iface" "net stats"
import re, subprocess, sys, time
import serial

tty = subprocess.run(["./console_tty.sh"], capture_output=True, text=True,
                     cwd=sys.path[0]).stdout.strip()
s = serial.Serial(tty, 115200, timeout=0.3)
s.reset_input_buffer()
for cmd in sys.argv[1:]:
    s.write((cmd + "\r\n").encode())
    time.sleep(1.5)
    out = s.read(20000).decode(errors="replace")
    print(re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", out).replace("\r", ""))
