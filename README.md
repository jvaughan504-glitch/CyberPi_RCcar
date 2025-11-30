# CyberPi RC Car

This project shows how to build an RC car that uses a Makeblock CyberPi as the handheld controller and an ESP32 as the receiver/motor driver interface. The repository contains:

- `controller/` – CyberPi MicroPython example that reads the built-in accelerometer and buttons, then broadcasts steering/throttle data over Wi‑Fi using UDP.
- `receiver/` – ESP32 Arduino sketch that connects to the controller's Wi‑Fi access point, decodes the UDP commands, and drives either a steering servo + bidirectional ESC (default) or two DC motors via an H-bridge (L298N/L9110S/DRV8833).

## Hardware overview

1. **Controller (CyberPi)**
   - Powered by its internal battery.
   - Uses built-in Wi‑Fi to host an access point (default SSID `cyberpi-rc`) that the ESP32 joins.
   - Uses the accelerometer to steer (tilt left/right) and the B button to throttle forward. The A button enables reverse.

2. **Receiver (ESP32)**
   - Connects to the CyberPi access point.
   - Listens for UDP packets on port `4210` containing throttle/steering values.
   - Drives either:
     - A **standard steering servo + RC car ESC** (default): servo on `GPIO12`, ESC signal on `GPIO13`, both at 50 Hz with 1000–2000 µs pulses (adjust in `receiver/esp32_receiver.ino`).
     - **Dual DC motors via an H-bridge**: left motor on pins `GPIO25`/`GPIO26`, right motor on pins `GPIO27`/`GPIO14` (set `USE_RC_OUTPUTS` to `false`).

## Communication protocol

The controller sends a small JSON blob 20 times per second:

```json
{"throttle": 0.6, "steering": -0.25}
```

- `throttle`: `-1.0` (full reverse) to `1.0` (full forward)
- `steering`: `-1.0` (full left) to `1.0` (full right)

Packets are sent via UDP to `192.168.4.2:4210` by default (the ESP32 static IP once connected to the CyberPi AP).

## Quick start

1. Flash the ESP32 with the sketch in `receiver/esp32_receiver.ino` using the Arduino IDE or PlatformIO.
2. Copy `controller/cyberpi_controller.py` to the CyberPi (Makeblock mBlock -> Upload). The script starts a Wi‑Fi AP and begins sending packets.
3. Power the RC car and CyberPi. When the ESP32 joins the AP, tilting the CyberPi steers; holding **B** drives forward, and holding **A** drives backward.

### Selecting the output mode (servo/ESC vs. H-bridge)

Open `receiver/esp32_receiver.ino` and set `USE_RC_OUTPUTS`:

- `true` (default): outputs hobby PWM for a steering servo (`SERVO_PIN`) and ESC (`ESC_PIN`). Adjust the min/max pulse widths if your hardware expects a different calibration range.
- `false`: drives two DC motors using PWM on the H-bridge pins.

## Safety

- Test the car with wheels off the ground first to confirm motor direction.
- Add fuses and use a motor driver rated for your motor stall current.
- If the car runs away, cut power or reset the ESP32; the receiver includes a 500 ms failsafe timeout.
