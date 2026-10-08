#!/usr/bin/env python3
"""Send single-letter commands to the PS5->PPM ESP32 and print its output.

Usage: esp_cmd.py [commands] [seconds]   e.g.  esp_cmd.py k 3
"""
import sys
import time

import serial

PORT = "/dev/cu.usbserial-110"

cmds = sys.argv[1] if len(sys.argv) > 1 else ""
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3

s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.2
s.dtr = s.rts = False  # don't reset the board on open
s.open()
time.sleep(0.3)
s.reset_input_buffer()
for c in cmds:
    s.write(c.encode())
    time.sleep(0.2)
end, out = time.time() + secs, b""
while time.time() < end:
    out += s.read(4096)
print(out.decode(errors="replace"))
