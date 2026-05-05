# motorgo-gimbus-driver

ESP32-S3 BLDC motor driver for the Gimbus servo board. Receives serial commands from a Jetson Nano and drives a single BLDC motor via SimpleFOC + DRV8316.

## Hardware

| Component | Part |
|-----------|------|
| MCU | ESP32-S3 |
| Motor driver | DRV8316 (6-PWM mode) |
| Encoder | MT6701 (SPI, HSPI bus) |
| Motor | 11 pole-pair, 6.9Ω, 24 KV BLDC |
| Indicator | SK6812 RGB LED |

## Building & Flashing

Use the `driver` PlatformIO environment:

```bash
pio run -e driver          # build
pio run -e driver -t upload # flash
pio device monitor -b 115200 # monitor
```

First boot will run sensor calibration (motor rotates slowly for ~10 seconds) and save the result to SPIFFS. Subsequent boots load the saved calibration instantly.

To force recalibration at runtime, send `CMD:R` over serial (see below). To wipe calibration manually, erase SPIFFS via `pio run -e driver -t erasefs` and power cycle.

---

## Serial Protocol

Communication is over USB CDC at **115200 baud**. All messages are newline-terminated (`\n`).

### Commands — Jetson → Driver

| Format | Description |
|--------|-------------|
| `CMD:V:<float>` | Velocity mode — target in **rad/s** |
| `CMD:P:<float>` | Position mode — target in **radians** |
| `CMD:T:<float>` | Torque mode — target as **voltage** (clamped to ±1.2 V) |
| `CMD:O` | Disable motor immediately |
| `CMD:R` | Force sensor recalibration (motor will rotate ~10 s) |

**Examples:**
```
CMD:V:10.0       # spin at 10 rad/s
CMD:V:-5.5       # spin in reverse at 5.5 rad/s
CMD:P:3.14159    # move to π radians
CMD:P:0.0        # return to zero position
CMD:T:0.5        # apply 0.5 V torque
CMD:O            # stop and disable
CMD:R            # wipe saved calibration and recalibrate now
```

**Input clamping:**
- Velocity: clamped to `±100.0 rad/s`
- Torque: clamped to `±1.2` (voltage units)
- Position: unclamped — respect mechanical limits in your application

### Telemetry — Driver → Jetson

Sent automatically at **10 Hz**. Format:

```
TELEM:<mode>:<target>:<angle>:<velocity>:<timestamp_ms>
```

| Field | Type | Description |
|-------|------|-------------|
| `mode` | char | `V`=velocity, `P`=position, `T`=torque, `D`=disabled, `C`=calibrating, `E`=error |
| `target` | float | Current commanded target value |
| `angle` | float | Shaft angle in radians |
| `velocity` | float | Shaft velocity in rad/s |
| `timestamp_ms` | uint32 | `millis()` on the ESP32 |

**Example output:**
```
TELEM:V:10.0000:3.1416:9.8500:12345
TELEM:D:0.0000:3.1416:0.0012:12445
TELEM:C:0.0000:1.2300:0.0000:13000
TELEM:E:0.0000:0.0000:0.0000:0
```

### Debug Messages — Driver → Jetson

Prefixed with `DBG:`. Informational only — safe to ignore in production parsing.

```
DBG:=== Gimbus Motor Driver Starting ===
DBG:Calibration loaded from SPIFFS
DBG:Motor ready
DBG:Heartbeat timeout - motor disabled
DBG:Unknown command type 'X'
```

### Parsing on the Jetson

Filter lines by prefix:

```python
for line in serial_port:
    line = line.strip()
    if line.startswith("TELEM:"):
        _, mode, target, angle, velocity, ts = line.split(":")
    elif line.startswith("DBG:"):
        print("[ESP32]", line[4:])  # log or ignore
```

---

## Safety Behaviors

### Heartbeat Watchdog
If no valid `CMD:*` is received for **1000 ms**, the motor is automatically disabled and the LED returns to cyan. This protects against serial cable disconnects or Jetson crashes.

Resume by sending any valid command.

### Mode Switch Safety
When switching between control modes (V/P/T), the target is zeroed before the new mode is activated to prevent sudden jerks.

### Recalibration (`CMD:R`)
Sending `CMD:R` triggers a full sensor recalibration at runtime:

1. Motor is disabled immediately and `MODE_CALIBRATING` is set
2. Telemetry continues at 10 Hz with mode char `C` — the Jetson can poll this to know when calibration is done
3. The heartbeat watchdog is **suspended** for the entire calibration run (motor is intentionally rotating, no commands expected)
4. The old SPIFFS calibration file is deleted, a fresh calibration is run (~10 s), and the result is saved
5. FOC is re-initialized with the new calibration
6. Driver returns to `MODE_DISABLED` (cyan LED) and the heartbeat timer is reset — the Jetson can resume sending commands normally

> **Note:** The motor shaft must be free to rotate during calibration. Do not send any other commands while `TELEM:C:...` is being received.

### Calibration Failure
If SPIFFS fails to mount or calibration cannot complete, the driver enters `MODE_ERROR`:
- LED flashes **red**
- Sends `TELEM:E:...`
- Motor enable is refused
- Requires power cycle to retry

---

## LED State Reference

| Color | State |
|-------|-------|
| Blue | Booting / initializing |
| Cyan | Disabled / idle |
| Green | Velocity mode active |
| Yellow | Position mode active |
| Magenta | Torque mode active |
| Orange | Calibrating (motor rotating) |
| Red (flashing) | Error — calibration failure |

---

## PID Parameters (defaults)

| Loop | P | I | D | Notes |
|------|---|---|---|-------|
| Velocity | 0.75 | 0.075 | 0.001 | output_ramp=1000, LPF Tf=0.05 |
| Angle | 20.0 | 0 | 0 | |
| Current Q | 1.0 | 0 | 0 | output_ramp=100, LPF Tf=0.01 |
| Current D | 0.25 | 0 | 0 | LPF Tf=0.01 |

Torque controller uses `voltage` mode (no current sense required).
