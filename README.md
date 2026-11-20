# AeroTouch: Touchless Mid-Air Surgical Display Controller

[![Platform](https://img.shields.io/badge/MCU-TI%20MSP430F5529%20%2B%20ESP32--S3-red.svg)]()
[![Language](https://img.shields.io/badge/Firmware-Bare--Metal%20C%20%2F%20C%2B%2B-blue.svg)]()
[![Protocol](https://img.shields.io/badge/Interface-Bit--Bang%20I2C%20%7C%20UART%20%7C%20USB%20HID-green.svg)]()
[![Accuracy](https://img.shields.io/badge/Gesture%20Accuracy-91.67%25-brightgreen.svg)]()
[![Latency](https://img.shields.io/badge/Avg%20Response-2.17%20ms-orange.svg)]()

**AeroTouch** is an embedded, contactless human-machine interface engineered to preserve surgical sterility in operating rooms and clinical suites. By replacing physical mice, keyboards, and touchscreens with mid-air optical and acoustic sensing, clinicians can navigate diagnostic imaging series (swipe, zoom) without breaking aseptic technique or generating single-use sterile drape waste.

---

## System Architecture

AeroTouch decouples real-time deterministic sensor fusion from host USB enumeration across a dual-microcontroller architecture:

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                               SENSING LAYER                                 │
│  [HC-SR04 Ultrasonic]      [VL53L0X #1 (Left)]      [VL53L0X #2 (Right)]    │
│   Presence & Wake           940nm ToF Ranging        940nm ToF Ranging      │
│   (TRIG/ECHO GPIO)          (Bit-Bang I2C P2)        (Bit-Bang I2C P3)      │
│                                                                             │
│  [10kΩ Potentiometer] ──► Dynamic Entry/Exit Threshold Tuning (40–340 mm)   │
└──────────────┬──────────────────────┬────────────────────────┬──────────────┘
               │ Timer_A0 Capture     │ Port 2 I2C             │ Port 3 I2C
               ▼                      ▼                        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PROCESSING LAYER (TI MSP430F5529)                      │
│  • Hybrid Power FSM: LPM3 Sleep (200 ms ACLK poll) ◄──► 50 Hz Active Loop   │
│  • Discrete 2x2 MIMO EMA IIR Low-Pass Filter (α = 0.9, O(1) Integer Math)   │
│  • Deterministic Gesture State Machine (Swipe Left/Right, Pinch In/Out)     │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ USCI_A0 UART (115,200 baud) + WAKE_OUT
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                   COMMUNICATION & HID LAYER (ESP32-S3)                      │
│  • 256-Byte Non-Blocking Ring Buffer & Frame Integrity Validator            │
│  • Cadence-Scaled Native USB OTG HID Keyboard Translator (Driverless)       │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ Native USB HID
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     DISPLAY LAYER (Host Workstation)                        │
│              Driverless Medical Image Series Viewer Control                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Engineering Highlights

* **Zero-Collision Dual VL53L0X Acquisition:** Resolves the fixed `0x29` 7-bit $\text{I}^2\text{C}$ slave address conflict without external `XSHUT` wiring by implementing isolated software Bit-Bang $\text{I}^2\text{C}$ drivers on **Port 2** (`P2.0`/`P2.2`) and **Port 3** (`P3.0`/`P3.1`).
* **Optical Crosstalk Suppression:** Narrows the stock $25^\circ$ VL53L0X Field of View to $\approx 10^\circ$ using custom matte-black aperture tubes paired with a central physical divider baffle to eliminate adjacent channel triggers.
* **Event-Driven Power Management (`LPM3` $\leftrightarrow$ `LPM0`):** Driven by `Timer_B0` ($32.768\text{ kHz}$ `REFO`), the MSP430 idles in `LPM3` while polling the HC-SR04 every $200\text{ ms}$. A 2-ping approach confirmation ($\le 400\text{ mm}$) asserts `WAKE_OUT` (`P1.4`) and enters a $50\text{ Hz}$ active tracking loop, returning to sleep after a $1\text{ s}$ asymmetric hysteresis hold-off ($> 450\text{ mm}$).
* **Discrete-Time MIMO State-Space Filtering:** Smooths optical photon jitter in $O(1)$ space and $O(1)$ integer arithmetic using a decoupled first-order IIR Exponential Moving Average filter ($\alpha = 0.9$, eigenvalues $|\lambda_{1,2}| = 0.1 < 1$ inside the unit circle).
* **Adaptive Threshold Calibration:** A 12-bit `ADC12` potentiometer input (`P7.0` / `A12`) dynamically scales the gesture entry threshold from $40\text{ mm}$ to $340\text{ mm}$ with a $+40\text{ mm}$ exit hysteresis band to prevent boundary chatter.
* **Driverless USB OTG HID Translation:** The ESP32-S3 validates incoming ASCII frames via a 256-byte ring buffer and translates gestures into native USB HID keystrokes (`Right Arrow`, `Left Arrow`, `Ctrl + =`, `Ctrl + -`) with cadence-based double-step acceleration.

---

## Hardware Pinout & Wiring

### TI MSP430F5529 LaunchPad
| Peripheral / Signal | MSP430 Pin | Interface / Function | Notes |
| :--- | :---: | :--- | :--- |
| **HC-SR04 `TRIG`** | `P1.3` | GPIO Output | $11.4\ \mu\text{s}$ trigger pulse |
| **HC-SR04 `ECHO`** | `P1.2` | `Timer_A0` Capture (`TA0.1`) | Rising/falling edge hardware capture |
| **VL53L0X #1 (Left)** | `P2.0` (SDA), `P2.2` (SCL) | Software Bit-Bang $\text{I}^2\text{C}$ | Isolated Bus 1 (`0x29`) |
| **VL53L0X #2 (Right)** | `P3.0` (SDA), `P3.1` (SCL) | Software Bit-Bang $\text{I}^2\text{C}$ | Isolated Bus 2 (`0x29`) |
| **Potentiometer** | `P7.0` (`A12`) | 12-bit `ADC12` | Maps `0–4095` $\rightarrow$ `40–340 mm` |
| **ESP32-S3 UART** | `P3.3` (TX), `P3.4` (RX) | `USCI_A0` UART | `115,200 baud`, `8-N-1` |
| **ESP32-S3 Wake Line** | `P1.4` | GPIO Output (`WAKE_OUT`) | `HIGH` in active mode, `LOW` in `LPM3` |
| **PC Debug Terminal** | `P4.4` (TX), `P4.5` (RX) | `USCI_A1` UART | `115,200 baud` telemetry logs |
| **Status Indicator** | `P1.0` | GPIO Output (`LED1`) | Active/Sleep state indicator |

---

## Gesture Command Mapping

| Hand Gesture | Physical Motion | UART Frame | ESP32-S3 USB HID Output | Viewer Action |
| :--- | :--- | :---: | :--- | :--- |
| **Swipe Right** | Left sensor $\rightarrow$ Right sensor ($30\text{--}400\text{ ms}$) | `SWIPE_RIGHT\n` | `Right Arrow` (x2 if $<200\text{ ms}$) | Next Image / Slice |
| **Swipe Left** | Right sensor $\rightarrow$ Left sensor ($30\text{--}400\text{ ms}$) | `SWIPE_LEFT\n` | `Left Arrow` (x2 if $<200\text{ ms}$) | Previous Image / Slice |
| **Pinch Out** | Both hands/fingers move apart | `ZOOM_IN\n` | `Ctrl` + `=` (x2 if $<50\text{ ms}$) | Zoom In |
| **Pinch In** | Both hands/fingers move closer | `ZOOM_OUT\n` | `Ctrl` + `-` (x2 if $<50\text{ ms}$) | Zoom Out |

---

## Experimental Performance

Evaluated across 60 structured gesture trials, approach/departure state transitions, and UART loopback stress tests:

| Evaluation Metric | Measured Result | Notes |
| :--- | :---: | :--- |
| **Overall Gesture Accuracy** | **91.67%** ($55/60$) | $100\%$ accuracy on Swipe Left & Swipe Right ($50\text{ mm}$) |
| **Average Processing Latency** | **2.17 ms** | Range: `1.473 ms` – `2.851 ms` (sensor read to UART TX) |
| **UART Frame Integrity** | **100%** ($10/10$) | Zero dropped valid commands; $100\%$ malformed frame rejection |
| **False-Positive Rejection** | **100%** ($3/3$) | Zero unintended triggers during stationary/no-gesture trials |
| **Power State Transitions** | **4/4 Stages Passed** | Reliable `LPM3` wake ($400\text{ mm}$) and sleep hold-off ($450\text{ mm}$) |

---

## Repository Structure

```text
├── firmware/
│   ├── msp430/
│   │   ├── main.c              # Full integrated firmware (LPM3/LPM0 FSM, EMA, Gesture Engine)
│   │   └── main_raw_demo.c     # Standalone unthrottled raw sensor telemetry diagnostic
│   └── esp32s3/
│       └── hid_bridge.ino      # UART ring-buffer parser & native USB OTG HID mapper
├── docs/
│   └── AeroTouch_Report.pdf    # Technical report (CEG2001 Sensors & Control / CEG2002 Embedded Systems)
└── README.md
```

---

## Contributors (SIT CEG2001 / CEG2002 — AY2026 Trimester 1)

* **Lex Lee Gao Hao** (`2501194`)
* **Lee Yu Huan** (`2502685`)
* **Manavalan Srihanush** (`2501002`)
* **Boey Meng Long** (`2500296`)