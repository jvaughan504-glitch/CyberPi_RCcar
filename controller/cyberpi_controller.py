# MicroPython script for Makeblock CyberPi
# Broadcasts throttle/steering commands to an ESP32 receiver over UDP.
# Upload with mBlock (upload mode) and run.

import cyberpi
import time
import wifi
import socket
import json

SSID = "cyberpi-rc"
PASSWORD = "drivefast"
UDP_PORT = 4210
TARGET_IP = "192.168.4.2"  # ESP32 static IP in receiver sketch

# Networking
wifi.start_ap(SSID, PASSWORD)
wifi.set_hostname("cyberpi-remote")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0)

# Input filtering
SMOOTHING = 0.25
steer_filtered = 0.0

cyberpi.console.clear()
cyberpi.console.println("CyberPi RC Controller")
cyberpi.console.println("AP SSID: %s" % SSID)
cyberpi.console.println("Hold B to drive forward, A for reverse")

while True:
    # Tilt steering: x-axis roll controls steering
    roll = cyberpi.get_roll()  # degrees; left negative, right positive
    steering = max(-1.0, min(1.0, roll / 30.0))  # clamp around +-30°
    steer_filtered = (SMOOTHING * steering) + ((1 - SMOOTHING) * steer_filtered)

    # Throttle from buttons: B forward, A reverse
    throttle = 0.0
    if cyberpi.button_b.is_pressed():
        throttle = 1.0
    elif cyberpi.button_a.is_pressed():
        throttle = -1.0

    payload = json.dumps({
        "throttle": throttle,
        "steering": steer_filtered
    })

    try:
        sock.sendto(payload.encode(), (TARGET_IP, UDP_PORT))
    except OSError:
        pass  # ignore transient Wi‑Fi errors

    # Small status indicator on the LED ring
    brightness = int(abs(throttle) * 100)
    if throttle > 0:
        cyberpi.led_ring.show_color((0, brightness, 0))
    elif throttle < 0:
        cyberpi.led_ring.show_color((brightness, 0, 0))
    else:
        cyberpi.led_ring.show_color((0, 0, int(abs(steer_filtered) * 100)))

    time.sleep(0.05)  # 20 Hz update
