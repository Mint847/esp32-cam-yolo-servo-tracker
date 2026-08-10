# ESP32-CAM YOLO Servo Tracker — Complete Setup & Debug Guide

This skill documents every issue encountered and resolved when building a real-time
YOLO object tracking system with an ESP32-CAM, ESP32-S3, PCA9685 servo driver, and Python.

---

## System Architecture

```
[Python main.py on PC]
    |-- OpenCV reads MJPEG stream from ESP32-CAM over WiFi (HTTP port 81)
    |-- YOLO (OpenVINO) runs inference on each frame
    |-- Calculates pan/tilt servo deltas
    |-- Sends "D,pan,tilt\n" or "C\n" over USB serial (COM port)
         |
         v
[ESP32-S3 (servo_controller.ino)]
    |-- Reads serial commands
    |-- Controls PCA9685 via I2C
         |
         v
[PCA9685 Servo Driver]
    |-- Channel 0: Pan servo (left/right)
    |-- Channel 1: Tilt servo (up/down)

[ESP32-CAM (esp32_cam_stream.ino)]
    |-- Connects to WiFi
    |-- Streams MJPEG at http://<IP>:81/stream
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Camera | ESP32-CAM (AI-Thinker) |
| Servo controller | ESP32-S3 DevKit (native USB, no UART bridge) |
| Servo driver | PCA9685 16-channel PWM driver |
| Pan servo | PCA9685 channel 0 |
| Tilt servo | PCA9685 channel 1 |
| Power | External 5V supply to PCA9685 V+ terminal and esp32 camera |

---

## Wiring — PCA9685 to ESP32-S3

| PCA9685 Pin | ESP32-S3 Pin | Notes |
|-------------|--------------|-------|
| VCC | 3.3V | Logic power for PCA9685 chip |
| GND | GND | Shared ground |
| SDA | GPIO8 | I2C data |
| SCL | GPIO9 | I2C clock |
| V+ | External 5V | Servo motor power — MUST be separate supply |

> **Critical:** V+ (servo power) must come from an external 5V supply, NOT from the ESP32's 3.3V
> pin. Servos draw too much current. If V+ is missing, servos go completely limp.

---

## Arduino IDE Settings for ESP32-S3 (Native USB Board)

> These settings are critical and often cause invisible failures if wrong.

| Setting | Value | Why |
|---------|-------|-----|
| Board | ESP32S3 Dev Module | |
| USB CDC On Boot | **Enabled** | See section below |
| Upload Speed | 921600 | |
| Flash Size | 8MB | match your board |

### USB CDC On Boot MUST be Enabled

The ESP32-S3 DevKit (no UART bridge chip) uses **native USB** for both uploading and serial
communication. With "USB CDC On Boot: **Disabled**":
- `Serial.begin()` is called late in `setup()`
- By the time it initializes, the USB has disconnected and reconnected
- Serial Monitor shows "not connected / connecting to board" on RST
- All `Serial.println()` output is lost — nothing ever appears in Serial Monitor
- Python's `ser.write()` commands are **never received** by the sketch

With "USB CDC On Boot: **Enabled**":
- USB CDC starts from the bootloader
- Serial is live before `setup()` even runs
- RST still disconnects briefly but output is visible
- Python serial communication works correctly

**Symptom of wrong setting:** Sketch uploads fine, ROM bootloader prints
`ESP-ROM:esp32s3-20210327`, but zero sketch output ever appears.

---

## Arduino Sketch — servo_controller.ino

### Use Adafruit_PWMServoDriver Library (NOT raw Wire calls)

**Wrong approach (raw I2C Wire):**
```cpp
#include <Wire.h>
Wire.begin(8, 9);      // Can hang due to I2C bus lockup
Wire.beginTransmission(0x40);
// ... raw register writes
```

**Correct approach (Adafruit library):**
```cpp
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // 27 MHz — must match your hardware
  pwm.setPWMFreq(50);                    // 50 Hz for servos
}
```

The raw Wire approach caused **I2C bus lockup** — if the PCA9685 held SDA low from a
previous failed transaction, `Wire.begin()` would block forever. The Adafruit library
handles bus recovery internally.

### Oscillator Frequency

The PCA9685's internal oscillator varies. Use `setOscillatorFrequency(27000000)` (27 MHz).
Verify by checking that your servos reach their correct endpoints.
Default is 25 MHz which causes incorrect PWM timing.

### Serial Command Protocol

```
C\n           -> Center both servos (smooth eased animation)
D,pan,tilt\n  -> Move servos by delta (e.g. D,5,-3\n)
D,0,0\n       -> Stop / freeze at current position
```

### Command Timeout

```cpp
unsigned long commandTimeoutMs = 2000;  // ms
```
If no command is received for this long, the Arduino freezes the target at current position.
**Must be > 500ms** because YOLO can lose detection for several seconds (camera shake,
occlusion). With 500ms, the servo freezes and resets every time detection is briefly lost.

### Servo Limits and Center

```cpp
int panCenter  = 300;  int panMin  = 180;  int panMax  = 430;
int tiltCenter = 410;  int tiltMin = 230;  int tiltMax = 470;
```
These are PWM count values (not microseconds). Adjust to match your physical servo range.

### Direction Pins

```cpp
int PAN_DIR  = 1;   // set to -1 to reverse pan direction
int TILT_DIR = 1;   // set to -1 to reverse tilt direction
```

---

## Python — main.py

### Non-Blocking Serial Writes (Critical for Performance)

`ser.write()` in the main loop **blocks the YOLO inference loop**. This causes:
- YOLO to slow down dramatically
- MJPEG camera buffer to overflow -> `[mjpeg] error dc / error y=29 x=18`
- System to crash

**Fix:** Use a background thread with a queue:

```python
import queue

serial_queue = queue.Queue(maxsize=10)

def serial_writer_thread():
    while True:
        data = serial_queue.get()
        if data is None:
            break
        ser.write(data)

def send_serial(data: bytes):
    """Non-blocking: drops command if queue is full rather than blocking."""
    try:
        serial_queue.put_nowait(data)
    except queue.Full:
        pass

# Start it
threading.Thread(target=serial_writer_thread, daemon=True).start()

# Use send_serial() everywhere instead of ser.write()
send_serial(b"C\n")
send_serial(f"D,{pan},{tilt}\n".encode())
```

### Serial Reader Thread (for Debugging Arduino)

Add this to see Arduino responses in Python's terminal:

```python
def serial_reader_thread():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[ARDUINO] {line}")
        except Exception:
            break
        time.sleep(0.01)

threading.Thread(target=serial_reader_thread, daemon=True).start()
```

Expected output:
```
[ARDUINO] Starting...
[ARDUINO] READY
[ARDUINO] CENTER
[ARDUINO] GOT D,5,0
[ARDUINO] TGT pan=305 tilt=410
```

### Servo Direction Signs

```python
PAN_SIGN  = 1   # flip to -1 if pan tracks backwards (away from target)
TILT_SIGN = 1   # flip to -1 if tilt tracks backwards
```

These multiply the error before sending to the Arduino. If the servo chases away from the
target ("same-poles magnet repulsion" behaviour), flip the sign.

### Camera Stream URL

```python
CAMERA_URL = "http://192.168.1.179:81/stream"
```

- Port 81 is the MJPEG stream endpoint from the ESP32-CAM firmware
- Port 80 is the camera web server UI
- The IP changes on router reboot — check router DHCP table or Arduino Serial Monitor on boot

### ESP32-CAM: Only 1 Concurrent Client

The ESP32-CAM MJPEG stream supports **one client at a time**. If the browser is open viewing
the feed, Python (`cv2.VideoCapture`) will fail to get a frame. Always close the browser tab
before running `main.py`.

### Startup Frame Wait

```python
for _ in range(200):       # 200 x 0.1s = 20 second timeout
    first_frame = reader.read()
    if first_frame is not None:
        break
    time.sleep(0.1)
```
20 seconds is needed because OpenCV + MJPEG stream negotiation can be slow.
Original 5 seconds was too short.

### Port Conflict on Upload

`main.py` holds the COM port open. If you try to upload an Arduino sketch while Python is
running, you get:
```
A fatal error occurred: Could not open COM13, the port doesn't exist
```
Always stop Python (`q` key or Ctrl+C) before uploading a new sketch.

---

## Common Error Reference

| Error | Cause | Fix |
|-------|-------|-----|
| `Error number -138 occurred` (tcp connect) | ESP32-CAM offline, wrong IP, or browser has the stream open | Close browser, check IP, power-cycle camera |
| `Could not get camera frame` | Camera not reachable within 20s | Check WiFi, IP, close other clients |
| `PermissionError: could not open port 'COM13'` | Python is running and holds the port | Stop Python first |
| Servos limp (no resistance when moved by hand) | No power to PCA9685 V+ terminal | Connect external 5V to V+ |
| Servos have power but don't move | I2C bus lockup or wrong I2C pins | Use Adafruit library, check GPIO8/GPIO9 wiring |
| Serial Monitor shows nothing (ESP32-S3 native USB) | USB CDC On Boot is Disabled | Set to Enabled in Arduino IDE board settings |
| YOLO slow + `[mjpeg] error dc` | `ser.write()` blocking main loop | Use async serial queue |
| Servo tracks away from target | PAN_SIGN or TILT_SIGN is wrong | Flip sign(s) from 1 to -1 or vice versa |
| Arduino not responding to Python commands | Wrong USB CDC setting | Enable USB CDC On Boot |
| `[tcp] connection failed` after first run | Camera DHCP IP changed | Check router, assign static IP to ESP32-CAM MAC |

---

## Debugging Checklist

When "servos don't move", check in this order:

1. **Does Serial Monitor show `Starting...` and `READY`?**
   - No -> USB CDC On Boot is Disabled. Enable it and re-upload.

2. **Do you see `[ARDUINO]` lines in Python terminal?**
   - No -> Arduino is not receiving Python's commands. Check serial reader thread.

3. **Do you see D,X,Y lines printed in Python terminal?**
   - No -> YOLO is not detecting the target. Check lighting, confidence, camera stream.

4. **Do servos move freely by hand?**
   - Yes (no resistance) -> V+ not connected. Power issue.
   - No (resist but don't follow) -> Direction or limits problem.

5. **Does typing `C` in Serial Monitor move the servo to center?**
   - No -> PCA9685 I2C broken. Check GPIO8/GPIO9 wiring and Adafruit library.
   - Yes -> I2C works. Issue is Python->Arduino serial communication.

---

## File Locations

```
Model/
+-- main.py                                          # Python YOLO tracking + serial
+-- best_openvino_model/                             # YOLO model (OpenVINO format)
+-- ESP32-CAM-AI-Object-Tracking/
    +-- SKILL.md                                     # This file
    +-- arduino/
        +-- servo_controller/
        |   +-- servo_controller.ino                 # ESP32-S3 servo controller
        +-- esp32_cam_stream/
        |   +-- esp32_cam_stream.ino                 # ESP32-CAM stream firmware
        +-- servo_test/
            +-- servo_test.ino                       # Diagnostic/test sketch
```

---

## Final Working Configuration Summary

### Python (main.py)
```python
SERIAL_PORT   = "COM13"
CAMERA_URL    = "http://192.168.1.179:81/stream"
TARGET_CLASS  = 0
PAN_SIGN      = 1      # may need -1 depending on servo mounting
TILT_SIGN     = 1      # may need -1 depending on servo mounting
COMMAND_DELAY = 0.015
```

### Arduino (servo_controller.ino)
```cpp
// Adafruit library - NOT raw Wire
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm.begin();
pwm.setOscillatorFrequency(27000000);
pwm.setPWMFreq(50);

// I2C pins (ESP32-S3) — set by default Wire on ESP32-S3
// SDA = GPIO8, SCL = GPIO9

// Channels
#define PAN_CH  0
#define TILT_CH 1

// Timeout
unsigned long commandTimeoutMs = 2000;

// Directions (adjust if tracking inverted)
int PAN_DIR  = 1;
int TILT_DIR = 1;
```

### Arduino IDE
```
Board:             ESP32S3 Dev Module
USB CDC On Boot:   Enabled          <- critical for native USB boards
USB Mode:          Hardware CDC and JTAG
```
