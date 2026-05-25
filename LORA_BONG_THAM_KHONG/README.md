# Arduino LoRa Telemetry System

A complete telemetry stack for a high-altitude balloon (or similar long-range platform):
- **Arduino transmitter** reads sensors and broadcasts a `|`-delimited payload over LoRa at 433 MHz.
- **Arduino receiver** prints parsed telemetry to its serial port and also computes flight stats.
- **PyQt5 desktop GUI** ingests the receiver's serial output, plots position on a Google-satellite map, draws live charts, auto-logs to CSV, and replays old logs.

The transmitter also drives a heater resistor that activates after time or distance limits are exceeded — intended as a safety / self-destruct trigger for sensitive payloads.

---

## Layout

| File / folder | Role |
|---|---|
| `arduino_lora_phat/arduino_lora_phat.ino` | Transmitter sketch |
| `arduino_lora_thu/arduino_lora_thu.ino` | Receiver sketch |
| `lora_receiver.py` | PyQt5 desktop GUI |
| `requirements.txt` | Python dependencies |
| `logs/` | Auto-created CSV log directory |

---

## Hardware

### Transmitter board
| Component | Interface | Pin(s) |
|---|---|---|
| LoRa RA-02 (SX1278) | SPI | CS=D10, RST=D9, IRQ=D8 |
| GY-68 / BMP180 (pressure & temperature) | I²C | SDA=A4, SCL=A5 |
| u-blox M8N GPS | SoftwareSerial | RX=D4, TX=D3 |
| Heater resistor | Digital out | D6 |

### Receiver board
| Component | Interface | Pin(s) |
|---|---|---|
| LoRa RA-02 (SX1278) | SPI | CS=D10, RST=D9, IRQ=D8 |
| USB-to-serial back to PC | — | — |

---

## LoRa configuration

Identical on both ends:

| Parameter | Value |
|---|---|
| Frequency | 433 MHz |
| Spreading factor | SF10 |
| Bandwidth | 62.5 kHz |
| Coding rate | 4/8 |
| TX power | 20 dBm (100 mW) |
| Sync word | default (`0x12`, public) |

Derived: symbol time ≈ 16.4 ms, effective bit rate ≈ 305 bps, receiver sensitivity ≈ −137 dBm, link budget ≈ 157 dB. A 40-byte payload is on-air ≈ 1.4 s. **At ~5 s per packet this is ~28% duty cycle — exceeds the ETSI 433 MHz band limit in EU/UK. Increase the loop delay or move to SF7/SF8 + 125 kHz if you need to be compliant.**

---

## Libraries

### Arduino (install via Library Manager)
- [`LoRa`](https://github.com/sandeepmistry/arduino-LoRa) by Sandeep Mistry
- [`Adafruit BMP085/BMP180`](https://github.com/adafruit/Adafruit-BMP085-Library)
- [`TinyGPSPlus`](https://github.com/mikalhart/TinyGPSPlus)
- `SPI`, `Wire`, `SoftwareSerial` (built-in)

### Python (`pip install -r requirements.txt`)
- `PyQt5`
- `PyQtWebEngine` (Leaflet map)
- `pyserial`
- `pyqtgraph` (charts)

---

## Payload format

Nine `|`-delimited fields:

```
<Temperature>|<Pressure>|<BMP_Alt>|<Lat>|<Lon>|<GPS_Alt>|<Speed_kmh>|<Course>|<HeatStatus>
```

Example:
```
25.3|101325|120.5|21.027764|105.834160|125.0|0.0|270.0|OFF
```

Invalid GPS fields are sent as `N/A`.

---

## Transmitter logic

Each loop (~5 s):
1. Read GPS for ~5 s into TinyGPS++.
2. Read BMP180 (temperature, pressure, barometric altitude).
3. Extract GPS lat/lon/altitude/speed/course.
4. **Heater trigger check** — drives D6 HIGH (permanently for the session) if either:
   - Uptime exceeds **5 hours**, or
   - **Straight-line** distance from launch point exceeds **20 km**
5. Build the payload string and transmit via LoRa.

The launch point is captured on the first valid GPS fix.

⚠ **Known caveat**: state (`hasStartLocation`, `startLat`, `startLon`, `isHeating`) is RAM-only — if the Nano resets (brown-out, cold weather, ESD), the start location is re-captured at the *current* position and the heater state resets to OFF. For mission-critical use, persist these to EEPROM and enable the AVR brown-out detector.

---

## Receiver sketch (Arduino-side)

Listens for LoRa packets and prints a formatted block to the Nano's serial port. Tracks two cumulative flight statistics locally:
- **Travel time** — `millis()` deltas between consecutive packets while altitude > 300 m (large gaps are dropped to handle missed packets).
- **Distance traveled** — cumulative haversine distance between consecutive valid GPS fixes while above 300 m.

Sample output:
```
=== GOI TIN MOI ===
Raw: [25.3|101325|120.5|21.027764|105.834160|125.0|0.0|270.0|OFF]
  Nhiet do      : 25.3 C
  Ap suat       : 101325 Pa
  Do cao BMP    : 120.5 m
  Lat (GPS)     : 21.027764
  Lon (GPS)     : 105.834160
  Alt (GPS)     : 125.0 m
  Van toc       : 0.0 km/h
  Huong di      : Tay (W) (270.0 do)
  SUOI (D6)     : OFF
  RSSI          : -72 dBm
  Thoi gian bay : 00:14:32 (chi dem khi >300m)
  Quang duong   : 12.45 km
--------------------------------------------------
```

---

## Desktop GUI (`lora_receiver.py`)

### Run
```powershell
pip install -r requirements.txt
python lora_receiver.py
```

### Layout
- **Top bar** — serial port picker, refresh, baud (default 9600), Connect/Disconnect, Auto-center map toggle, Clear Trail, **Open Log...**
- **Telemetry panel** (left) — time, all sensor values, **travel time (>300 m)**, **distance traveled**, **straight-line from start**, **average speed (m/s)**, **average vertical speed (m/s)**, **max altitude**, plus heater state and RSSI. Speed values display in **m/s** (converted from the km/h the transmitter sends).
- **Map** (right) — Leaflet view with Google **Satellite** as the default basemap; layer switcher to Hybrid or OpenStreetMap. Shows the live position marker, green start dot, red trail polyline, and a cyan speed-projection arrow (60 s ahead at current speed/course).
- **Bottom tabs**
  - **Charts** — three pyqtgraph plots, all updating live and on log load:
    - Altitude vs Time
    - Horizontal speed vs Time (X-axis linked to altitude plot)
    - Horizontal speed vs Altitude (scatter — wind profile by altitude)
    - **Double-click any chart** to pop it out into a 1000×700 detail window that stays live-updated.
  - **Serial Log** — raw line-by-line dump of the receiver's serial output.

### Click-to-inspect track points
Each recorded position has a small red dot on the map with a generous invisible 28-px hit target around it. Click any dot (or anywhere on the polyline) to open a popup with that point's timestamp, temperature, pressure, both altitudes, speed, **vertical speed**, course, heater state, and RSSI.

### GPS outlier filter (>500 km from start)
- **Live mode** — positions more than 500 km from the captured start point are silently dropped (no marker, no distance increment); a status-bar message reports the deviation.
- **Replay mode** — outliers are detected on load and **linearly interpolated** between the nearest valid neighbours. Interpolated points are drawn in **orange** instead of red, and their popups are prefixed `[Interpolated]`.

### Auto-CSV log
On startup, a new `logs/lora_log_YYYYMMDD_HHMMSS.csv` file opens and every received packet is flushed to it. Columns:

```
timestamp, temp_C, pressure_Pa, bmp_alt_m, lat, lon, gps_alt_m,
speed_kmh, course_deg, heater, rssi_dBm,
travel_time_s, distance_m, straight_line_m, raw
```

### Open Log...
File dialog (defaults to `logs/`) loads any past CSV:
- Auto-disconnects from serial first
- Plots the whole track (clickable per point)
- Fills the telemetry panel from the last valid row
- Computes all stats (recomputes from rows if loading a legacy log without the stat columns)
- Populates the charts
- Status bar reports `Loaded N rows (M fixes, K interpolated) from filename.csv`

---

## Getting started

1. Wire both boards per the pin tables above.
2. Install the Arduino libraries.
3. Flash `arduino_lora_phat.ino` to the transmitter and `arduino_lora_thu.ino` to the receiver.
4. Plug the **receiver** into your PC via USB. Note its COM port.
5. `pip install -r requirements.txt`, then `python lora_receiver.py`.
6. Pick the receiver's port, click Connect.
7. Power on the transmitter outdoors — wait for GPS lock before moving it.

---

## Heater / self-destruct logic (recap)

| Trigger | Threshold | Source |
|---|---|---|
| Uptime since boot | 5 h | `millis()` |
| **Straight-line** distance from launch | 20 km | `TinyGPSPlus::distanceBetween()` |

Either condition latches D6 HIGH for the rest of the session — until a reset. See the caveat about EEPROM persistence under [Transmitter logic](#transmitter-logic).

The GUI's **Straight-line from start** field mirrors exactly what the transmitter measures, so you can verify the trigger logic against the live data.
