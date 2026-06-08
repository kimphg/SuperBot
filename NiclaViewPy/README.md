# NiclaViewer — PyQt5

Real-time IMU and video viewer for the **Arduino Nicla Vision** board over USB.  
Python port of the C++ `NiclaView` application.

---

## Requirements

| Requirement | Version |
|---|---|
| Python | 3.8 + |
| PyQt5 | 5.15 + |
| PyQt5-Qt5 | 5.15 + |

PyQt5 5.15 includes `QtChart` and `QtSerialPort` out of the box.

```bash
pip install PyQt5
```

If `QtChart` is missing (older PyQt5):

```bash
pip install PyQtChart
```

---

## Run

```bash
python niclaviewer.py
```

---

## USB driver (Windows)

The Nicla Vision presents as a **STM32 Virtual COM Port**.  
Install **[OpenMV IDE](https://openmv.io/pages/download)** — it bundles the correct driver.

Once installed, connect the board via USB; it will appear as `COMx` in Device Manager.  
The app auto-detects and connects. No manual port selection needed.

---

## Firmware

Flash `medic_imu/main.py` to the board using OpenMV IDE.  
The firmware streams binary frames over USB CDC:

```
┌──────────┬────────┬─────────────────────┬─────────────────┐
│ MAGIC    │ TYPE   │ LENGTH (4 B, BE)     │ PAYLOAD         │
│ "NICL"   │ 1 byte │ uint32 big-endian   │ N bytes         │
└──────────┴────────┴─────────────────────┴─────────────────┘

TYPE 0x01  IMU text:   "IMU,<seq>,<ts_ms>,<roll>,<pitch>,<yaw>\n"
TYPE 0x02  Video JPEG: raw JPEG bytes
```

---

## Layout

```
┌───────────────────────────────────┬──────────────────────────┐
│                                   │  Roll / Pitch chart      │
│   Video cross (5 camera views)    │  (auto-scaling time axis)│
│        [top]                      ├──────────────────────────┤
│   [left] [live] [right]           │  Start / Stop / Reset    │
│        [bottom]                   ├──────────────────────────┤
│                                   │  Instructions            │
├───────────────────────────────────┼──────────────────────────┤
│  Patient info                     │  Results + Save buttons  │
│  Tên BN / Mã BN / Năm sinh        │  Angles + PDF / CSV      │
└───────────────────────────────────┴──────────────────────────┘
```

---

## Features

- **Auto-connect** — detects Nicla Vision by USB VID (`0x0483` STM32 / `0x2341` Arduino) or port description keyword; retries every 2 s
- **Live video** — center label shows the live camera feed; corner labels capture snapshots at extreme angles
- **Real-time chart** — roll (red) and pitch (blue) vs. elapsed time in seconds; x-axis extends automatically in 10 s steps
- **Measurement mode** — Start resets all extremes and begins recording; Stop freezes the chart
- **Button states** — Start turns green when idle, Stop turns red while measuring, others gray out
- **Save result** — captures chart and video frame screenshots, writes PDF report and CSV
- **PDF report** — cumulative HTML report with patient info and measurement data (Vietnamese)
- **CSV export** — `STT, Time(s), Roll(deg), Pitch(deg)`

---

## File structure

```
NiclaViewPy/
├── niclaviewer.py   # entire application (single file)
├── requirements.txt
└── README.md
```

---

## Angles reference

| Label | Meaning |
|---|---|
| Roll | Left/right rotation of the head. Positive = right. |
| Pitch | Tilt (0 = level, positive = looking up, negative = looking down). |
| Góc xoay | Roll |
| Góc ngẩng | Pitch |
| Biên độ cúi / ngẩng | Pitch min/max range |
| Biên độ xoay ngang | Roll min/max range |
