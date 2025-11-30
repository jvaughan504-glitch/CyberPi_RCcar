# MicroPython script for Makeblock CyberPi
# Broadcasts throttle/steering commands to an ESP32 receiver over UDP.
# Upload with mBlock (upload mode) and run.

import cyberpi
import json
import socket
import time
import wifi

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


def clamp(value, minimum=-1.0, maximum=1.0):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def wait_for_dual_press(duration_s=1.5):
    """Return True if A and B are held simultaneously for duration_s seconds."""
    start = time.time()
    while True:
        if cyberpi.button_a.is_pressed() and cyberpi.button_b.is_pressed():
            if time.time() - start >= duration_s:
                return True
        else:
            return False
        time.sleep(0.05)


def edit_settings(settings):
    """Simple on-device menu to tweak endpoints/mix."""
    items = [
        ("Throttle end", "throttle_end", 0.2, 1.0, 0.05),
        ("Steering end", "steering_end", 0.2, 1.0, 0.05),
        ("Mix", "mix", 0.0, 1.5, 0.05),
    ]
    index = 0
    cyberpi.console.clear()
    cyberpi.console.println("Settings mode")
    cyberpi.console.println("A=-, B=+, hold A+B next")

    while True:
        label, key, min_v, max_v, step = items[index]
        value = settings[key]
        cyberpi.console.println("%s: %3d%%" % (label, int(value * 100)))

        # Adjust with single presses
        if cyberpi.button_b.is_pressed():
            settings[key] = clamp(value + step, min_v, max_v)
            time.sleep(0.2)
        elif cyberpi.button_a.is_pressed():
            settings[key] = clamp(value - step, min_v, max_v)
            time.sleep(0.2)

        # Move to next item by holding both buttons
        if wait_for_dual_press(0.8):
            cyberpi.console.clear()
            index = (index + 1) % len(items)
            if index == 0:
                cyberpi.console.println("Exiting settings")
                time.sleep(0.8)
                cyberpi.console.clear()
                return
            cyberpi.console.println("Settings mode")
            cyberpi.console.println("A=-, B=+, hold A+B next")

        time.sleep(0.05)


settings = {
    "throttle_end": 1.0,
    "steering_end": 1.0,
    "mix": 1.0,
}

cyberpi.console.clear()
cyberpi.console.println("CyberPi RC Controller")
cyberpi.console.println("AP SSID: %s" % SSID)
cyberpi.console.println("Hold B=forward, A=reverse")
cyberpi.console.println("Hold A+B to edit settings")

while True:
    if wait_for_dual_press():
        edit_settings(settings)

    # Tilt steering: x-axis roll controls steering
    roll = cyberpi.get_roll()  # degrees; left negative, right positive
    steering = clamp(roll / 30.0)
    steer_filtered = (SMOOTHING * steering) + ((1 - SMOOTHING) * steer_filtered)
    steer_filtered = clamp(steer_filtered * settings["steering_end"])

    # Throttle from buttons: B forward, A reverse
    throttle = 0.0
    if cyberpi.button_b.is_pressed():
        throttle = settings["throttle_end"]
    elif cyberpi.button_a.is_pressed():
        throttle = -settings["throttle_end"]

    payload = json.dumps({
        "throttle": clamp(throttle),
        "steering": steer_filtered,
        "mix": settings["mix"],
        "aux_a": 1 if cyberpi.button_a.is_pressed() else 0,
        "aux_b": 1 if cyberpi.button_b.is_pressed() else 0,
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
