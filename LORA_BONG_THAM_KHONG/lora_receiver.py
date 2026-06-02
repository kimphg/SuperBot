"""
LoRa Receiver GUI
Reads telemetry from the Arduino receiver sketch over serial, displays it,
plots live position on an OpenStreetMap (Leaflet) view, and auto-saves to CSV.
"""

import csv
import json
import math
import os
import re
import sys
import time
from datetime import datetime

ALT_THRESHOLD_M = 300.0
MAX_DELTA_S = 30.0  # ignore deltas larger than this (missed packets)
OUTLIER_THRESHOLD_M = 500_000.0  # >500 km from start = clearly junk GPS

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:
    import pyqtgraph as pg
    HAVE_PG = True
except ImportError:
    HAVE_PG = False
    pg = None  # type: ignore

# --------------------------------------------------------------------------
# Map HTML (Leaflet + Google Satellite). JS hooks: updatePosition(...), clearTrail()
# --------------------------------------------------------------------------
MAP_HTML = r"""
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>LoRa Map</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<style>
  html, body, #map { height: 100%; margin: 0; padding: 0; }
  .info {
    background: rgba(255,255,255,0.9);
    padding: 6px 10px;
    border-radius: 4px;
    font: 12px sans-serif;
    box-shadow: 0 1px 4px rgba(0,0,0,0.3);
  }
</style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://unpkg.com/leaflet-polylinedecorator@1.6.0/dist/leaflet.polylineDecorator.js"></script>
<script>
  var map = L.map('map').setView([21.0278, 105.8342], 13); // Hanoi default

  var googleSat = L.tileLayer(
    'https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
    {
      attribution: 'Imagery &copy; Google',
      subdomains: ['0', '1', '2', '3'],
      maxZoom: 20
    }
  ).addTo(map);

  var googleHybrid = L.tileLayer(
    'https://mt{s}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
    {
      attribution: 'Imagery &copy; Google',
      subdomains: ['0', '1', '2', '3'],
      maxZoom: 20
    }
  );

  var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap contributors',
    maxZoom: 19
  });

  L.control.layers(
    { 'Google Satellite': googleSat, 'Google Hybrid': googleHybrid, 'OpenStreetMap': osm },
    null,
    { position: 'topright', collapsed: true }
  ).addTo(map);

  var marker = null;
  var startMarker = null;
  var trail = L.polyline([], { color: 'red', weight: 3 }).addTo(map);
  var speedLine = null;
  var speedArrow = null;
  var autoCenter = true;
  var projectSeconds = 60;  // arrow shows projected position N seconds ahead
  var trackPoints = [];     // [{lat, lon, info}, ...] for click lookup
  var pointMarkers = [];    // small clickable dots per recorded point

  function addPointMarker(lat, lon, info, interpolated) {
    var fill = interpolated ? '#ffaa00' : '#ff2222';
    // Invisible larger hit target so the click area is generous
    var hit = L.circleMarker([lat, lon], {
      radius: 14,
      stroke: false,
      fill: true,
      fillColor: fill,
      fillOpacity: 0,
      interactive: true
    }).addTo(map);
    hit.bindPopup(info, { autoPan: true });
    pointMarkers.push(hit);

    // Visible dot, non-interactive (clicks fall through to the hit target)
    var dot = L.circleMarker([lat, lon], {
      radius: 4,
      color: '#ffffff',
      weight: 1,
      fillColor: fill,
      fillOpacity: 1,
      interactive: false
    }).addTo(map);
    pointMarkers.push(dot);
  }

  // Make the trail clickable - find the nearest recorded point.
  function findNearestIdx(lat, lon) {
    var best = -1, bestD = Infinity;
    for (var i = 0; i < trackPoints.length; i++) {
      var dy = trackPoints[i].lat - lat;
      var dx = trackPoints[i].lon - lon;
      var d = dy * dy + dx * dx;
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }
  trail.on('click', function(e) {
    if (trackPoints.length === 0) return;
    var i = findNearestIdx(e.latlng.lat, e.latlng.lng);
    if (i < 0) return;
    L.popup()
      .setLatLng([trackPoints[i].lat, trackPoints[i].lon])
      .setContent(trackPoints[i].info)
      .openOn(map);
  });

  function setAutoCenter(v) { autoCenter = v; }
  function setProjectSeconds(s) { projectSeconds = s; }

  // Forward geodesic: destination from start, bearing (deg), distance (m)
  function destinationPoint(lat, lon, bearingDeg, distanceM) {
    var R = 6378137.0;
    var d = distanceM / R;
    var brng = bearingDeg * Math.PI / 180;
    var lat1 = lat * Math.PI / 180;
    var lon1 = lon * Math.PI / 180;
    var lat2 = Math.asin(
      Math.sin(lat1) * Math.cos(d) +
      Math.cos(lat1) * Math.sin(d) * Math.cos(brng)
    );
    var lon2 = lon1 + Math.atan2(
      Math.sin(brng) * Math.sin(d) * Math.cos(lat1),
      Math.cos(d) - Math.sin(lat1) * Math.sin(lat2)
    );
    return [lat2 * 180 / Math.PI, lon2 * 180 / Math.PI];
  }

  function clearSpeedVector() {
    if (speedArrow) { map.removeLayer(speedArrow); speedArrow = null; }
    if (speedLine)  { map.removeLayer(speedLine);  speedLine  = null; }
  }

  function drawSpeedVector(lat, lon, speedKmh, courseDeg) {
    clearSpeedVector();
    if (speedKmh === null || courseDeg === null) return;
    if (!isFinite(speedKmh) || !isFinite(courseDeg)) return;
    if (speedKmh <= 0.1) return;  // stationary -> no arrow

    var distM = (speedKmh * 1000 / 3600) * projectSeconds;
    var end = destinationPoint(lat, lon, courseDeg, distM);

    speedLine = L.polyline([[lat, lon], end], {
      color: '#00e5ff', weight: 3, opacity: 0.9
    }).addTo(map);

    speedArrow = L.polylineDecorator(speedLine, {
      patterns: [{
        offset: '100%',
        repeat: 0,
        symbol: L.Symbol.arrowHead({
          pixelSize: 14,
          polygon: true,
          pathOptions: { color: '#00e5ff', fillOpacity: 1, weight: 0 }
        })
      }]
    }).addTo(map);
  }

  function updatePosition(lat, lon, popupHtml, isFirst, speedKmh, courseDeg) {
    var latlng = [lat, lon];
    if (!marker) {
      marker = L.marker(latlng).addTo(map);
    } else {
      marker.setLatLng(latlng);
    }
    marker.bindPopup(popupHtml);
    trail.addLatLng(latlng);
    trackPoints.push({ lat: lat, lon: lon, info: popupHtml });
    addPointMarker(lat, lon, popupHtml);
    if (isFirst && !startMarker) {
      startMarker = L.circleMarker(latlng, {
        radius: 6, color: 'green', fillColor: 'green', fillOpacity: 1
      }).addTo(map).bindPopup('Start');
    }
    drawSpeedVector(lat, lon, speedKmh, courseDeg);
    if (autoCenter) {
      map.setView(latlng, map.getZoom() < 15 ? 15 : map.getZoom());
    }
  }

  function clearTrail() {
    trail.setLatLngs([]);
    trackPoints = [];
    pointMarkers.forEach(function(m) { map.removeLayer(m); });
    pointMarkers = [];
    clearSpeedVector();
    if (marker) { map.removeLayer(marker); marker = null; }
    if (startMarker) { map.removeLayer(startMarker); startMarker = null; }
  }

  // Replay a recorded track all at once.
  // pointsWithInfo: [{lat, lon, info}, ...]
  function loadTrack(pointsWithInfo, popupHtml, speedKmh, courseDeg) {
    clearTrail();
    if (!pointsWithInfo || pointsWithInfo.length === 0) return;
    trackPoints = pointsWithInfo.slice();
    var latlngs = pointsWithInfo.map(function(p) { return [p.lat, p.lon]; });
    trail.setLatLngs(latlngs);
    pointsWithInfo.forEach(function(p) {
      addPointMarker(p.lat, p.lon, p.info, !!p.interpolated);
    });
    var first = latlngs[0];
    var last = latlngs[latlngs.length - 1];
    startMarker = L.circleMarker(first, {
      radius: 6, color: 'green', fillColor: 'green', fillOpacity: 1
    }).addTo(map).bindPopup('Start');
    marker = L.marker(last).addTo(map).bindPopup(popupHtml);
    drawSpeedVector(last[0], last[1], speedKmh, courseDeg);
    try {
      map.fitBounds(trail.getBounds(), { padding: [40, 40] });
    } catch (e) {
      map.setView(last, 15);
    }
  }
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------
# Serial worker thread
# --------------------------------------------------------------------------
class SerialReader(QThread):
    packet_received = pyqtSignal(dict)
    line_received = pyqtSignal(str)
    error = pyqtSignal(str)
    connected = pyqtSignal(bool)

    RAW_RE = re.compile(r"Raw:\s*\[(.*)\]")
    RSSI_RE = re.compile(r"RSSI\s*[:=]\s*(-?\d+)")

    def __init__(self, port, baud=9600, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._stop = False
        self._pending = None  # parsed payload waiting for RSSI

    def run(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=1)
        except serial.SerialException as e:
            self.error.emit(f"Cannot open {self._port}: {e}")
            self.connected.emit(False)
            return

        self.connected.emit(True)
        try:
            while not self._stop:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    self.error.emit(f"Serial read error: {e}")
                    break
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                except Exception:
                    continue
                if not line:
                    continue
                self.line_received.emit(line)
                self._process_line(line)
        finally:
            ser.close()
            self.connected.emit(False)

    def _process_line(self, line):
        m = self.RAW_RE.search(line)
        if m:
            self._pending = self._parse_payload(m.group(1))
            return
        if self._pending is not None:
            m2 = self.RSSI_RE.search(line)
            if m2:
                self._pending["rssi"] = int(m2.group(1))
                self._pending["timestamp"] = datetime.now().isoformat(timespec="seconds")
                self.packet_received.emit(self._pending)
                self._pending = None

    @staticmethod
    def _parse_payload(payload):
        # temp|pressure|bmpAlt|lat|lon|gpsAlt|speed|course|heatStatus
        parts = payload.split("|")
        keys = ["temp", "pressure", "bmp_alt", "lat", "lon",
                "gps_alt", "speed", "course", "heat"]
        d = {"raw": payload, "rssi": None}
        for i, k in enumerate(keys):
            d[k] = parts[i].strip() if i < len(parts) else "N/A"
        return d

    def stop(self):
        self._stop = True


# --------------------------------------------------------------------------
# Chart helpers — pop-out detail dialogs
# --------------------------------------------------------------------------
if HAVE_PG:
    class PopOutPlotWidget(pg.PlotWidget):
        """PlotWidget that emits doubleClicked on left double-click."""
        doubleClicked = pyqtSignal()

        def mouseDoubleClickEvent(self, ev):
            if ev.button() == Qt.LeftButton:
                self.doubleClicked.emit()
                ev.accept()
                return
            super().mouseDoubleClickEvent(ev)


class ChartDialog(QDialog):
    """Standalone window showing a single chart, live-updated from a callable
    that returns the (x, y) series tuple."""

    def __init__(self, title, x_label, y_label, get_data, curve_factory,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(1000, 700)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", x_label)
        self.plot.setLabel("left", y_label)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = curve_factory(self.plot)
        layout.addWidget(self.plot)

        self._get_data = get_data
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def _refresh(self):
        x, y = self._get_data()
        self.curve.setData(x, y)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class MainWindow(QMainWindow):
    BAUDS = ["9600", "19200", "38400", "57600", "115200"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LoRa Telemetry Receiver")
        self.resize(1280, 800)

        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.packet_count = 0
        self.first_fix_logged = False

        # Flight stats
        self.travel_time_s = 0.0
        self.total_distance_m = 0.0
        self.straight_line_m = 0.0
        self.avg_speed_ms = 0.0
        self.avg_vspeed_ms = 0.0
        self.max_alt_m = None
        self._vspeed_sum = 0.0
        self._vspeed_count = 0
        self._last_alt = None
        self._last_alt_t = None
        self._last_vspeed_ms = None  # most recent instantaneous v-speed (live)
        self._start_position = None  # first valid (lat, lon) seen
        self._last_packet_time = None
        self._was_above_threshold = False
        self._last_position = None  # (lat, lon)

        # Chart series state
        self._t_alt = []
        self._v_alt = []
        self._t_spd = []
        self._v_spd = []
        self._sa_alt = []  # altitude for the speed-vs-altitude scatter
        self._sa_spd = []
        self._series_t0 = None

        self._build_ui()
        self._refresh_ports()
        self._open_log_file()

    # ---- UI construction ----
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        # Connection bar
        bar = QHBoxLayout()
        bar.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        bar.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        bar.addWidget(self.refresh_btn)

        bar.addSpacing(12)
        bar.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(self.BAUDS)
        self.baud_combo.setCurrentText("9600")
        bar.addWidget(self.baud_combo)

        bar.addSpacing(12)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        bar.addWidget(self.connect_btn)

        bar.addStretch(1)

        self.autocenter_cb = QCheckBox("Auto-center map")
        self.autocenter_cb.setChecked(True)
        self.autocenter_cb.toggled.connect(self._set_autocenter)
        bar.addWidget(self.autocenter_cb)

        self.clear_trail_btn = QPushButton("Clear Trail")
        self.clear_trail_btn.clicked.connect(self._clear_trail)
        bar.addWidget(self.clear_trail_btn)

        self.open_log_btn = QPushButton("Open Log...")
        self.open_log_btn.clicked.connect(self._load_log)
        bar.addWidget(self.open_log_btn)

        root.addLayout(bar)

        # Splitter: left telemetry, right map
        hsplit = QSplitter(Qt.Horizontal)
        hsplit.addWidget(self._build_telemetry_panel())
        hsplit.addWidget(self._build_map_panel())
        hsplit.setStretchFactor(0, 0)
        hsplit.setStretchFactor(1, 1)
        hsplit.setSizes([360, 920])

        # Bottom: tabbed Charts + Serial Log
        bottom_tabs = QTabWidget()
        bottom_tabs.addTab(self._build_charts(), "Charts")

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        mono = QFont("Consolas")
        mono.setStyleHint(QFont.Monospace)
        self.log_view.setFont(mono)
        bottom_tabs.addTab(self.log_view, "Serial Log")

        vsplit = QSplitter(Qt.Vertical)
        vsplit.addWidget(hsplit)
        vsplit.addWidget(bottom_tabs)
        vsplit.setStretchFactor(0, 3)
        vsplit.setStretchFactor(1, 2)
        vsplit.setSizes([520, 280])
        root.addWidget(vsplit, 1)

        # Status bar
        self.setStatusBar(QStatusBar())
        self.status_csv = QLabel("")
        self.status_count = QLabel("Packets: 0")
        self.statusBar().addWidget(self.status_csv, 1)
        self.statusBar().addPermanentWidget(self.status_count)

    def _build_telemetry_panel(self):
        group = QGroupBox("Telemetry")
        layout = QFormLayout(group)
        layout.setLabelAlignment(Qt.AlignLeft)

        self.lbl_time = QLabel("-")
        self.lbl_travel_time = QLabel("00:00:00")
        self.lbl_distance = QLabel("0 m")
        self.lbl_straight = QLabel("0 m")
        self.lbl_avg_speed = QLabel("- m/s")
        self.lbl_avg_vspeed = QLabel("- m/s")
        self.lbl_max_alt = QLabel("- m")
        self.lbl_temp = QLabel("-")
        self.lbl_pressure = QLabel("-")
        self.lbl_bmp_alt = QLabel("-")
        self.lbl_lat = QLabel("-")
        self.lbl_lon = QLabel("-")
        self.lbl_gps_alt = QLabel("-")
        self.lbl_speed = QLabel("-")
        self.lbl_course = QLabel("-")
        self.lbl_heat = QLabel("-")
        self.lbl_rssi = QLabel("-")

        big = QFont()
        big.setPointSize(11)
        for lbl in (self.lbl_time, self.lbl_travel_time, self.lbl_distance,
                    self.lbl_straight, self.lbl_avg_speed, self.lbl_avg_vspeed,
                    self.lbl_max_alt, self.lbl_temp, self.lbl_pressure,
                    self.lbl_bmp_alt, self.lbl_lat, self.lbl_lon,
                    self.lbl_gps_alt, self.lbl_speed, self.lbl_course,
                    self.lbl_heat, self.lbl_rssi):
            lbl.setFont(big)
            lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)

        stat_font = QFont()
        stat_font.setPointSize(13)
        stat_font.setBold(True)
        self.lbl_travel_time.setFont(stat_font)
        self.lbl_distance.setFont(stat_font)
        self.lbl_straight.setFont(stat_font)
        self.lbl_avg_speed.setFont(stat_font)
        self.lbl_avg_vspeed.setFont(stat_font)

        layout.addRow("Time:", self.lbl_time)
        layout.addRow(f"Travel time (>{int(ALT_THRESHOLD_M)} m):",
                      self.lbl_travel_time)
        layout.addRow("Distance traveled:", self.lbl_distance)
        layout.addRow("Straight-line from start:", self.lbl_straight)
        layout.addRow("Average speed:", self.lbl_avg_speed)
        layout.addRow("Average vertical speed:", self.lbl_avg_vspeed)
        layout.addRow("Max altitude:", self.lbl_max_alt)
        layout.addRow("Temperature (C):", self.lbl_temp)
        layout.addRow("Pressure (Pa):", self.lbl_pressure)
        layout.addRow("BMP Altitude (m):", self.lbl_bmp_alt)
        layout.addRow("Latitude:", self.lbl_lat)
        layout.addRow("Longitude:", self.lbl_lon)
        layout.addRow("GPS Altitude (m):", self.lbl_gps_alt)
        layout.addRow("Speed (m/s):", self.lbl_speed)
        layout.addRow("Course (deg):", self.lbl_course)
        layout.addRow("Heater (D6):", self.lbl_heat)
        layout.addRow("RSSI (dBm):", self.lbl_rssi)

        return group

    def _build_map_panel(self):
        group = QGroupBox("Map")
        layout = QVBoxLayout(group)
        self.map_view = QWebEngineView()
        self.map_view.setHtml(MAP_HTML)
        layout.addWidget(self.map_view)
        return group

    def _build_charts(self):
        if not HAVE_PG:
            msg = QLabel(
                "pyqtgraph not installed.\n\n"
                "Run:  pip install pyqtgraph"
            )
            msg.setAlignment(Qt.AlignCenter)
            return msg

        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        pg.setConfigOption("antialias", True)

        self.alt_plot = PopOutPlotWidget()
        self.alt_plot.setLabel("left", "Altitude (m)")
        self.alt_plot.setLabel("bottom", "Time (s)")
        self.alt_plot.showGrid(x=True, y=True, alpha=0.3)
        self.alt_curve = self.alt_plot.plot(
            pen=pg.mkPen("#cc6600", width=2), name="Altitude")
        self.alt_plot.doubleClicked.connect(self._open_alt_detail)

        self.spd_plot = PopOutPlotWidget()
        self.spd_plot.setLabel("left", "Speed (m/s)")
        self.spd_plot.setLabel("bottom", "Time (s)")
        self.spd_plot.showGrid(x=True, y=True, alpha=0.3)
        self.spd_curve = self.spd_plot.plot(
            pen=pg.mkPen("#0077cc", width=2), name="Horiz. speed")
        self.spd_plot.doubleClicked.connect(self._open_spd_detail)

        # Link x axes so panning/zooming syncs both time plots
        self.spd_plot.setXLink(self.alt_plot)

        # Speed-vs-altitude scatter (wind profile by altitude)
        self.sa_plot = PopOutPlotWidget()
        self.sa_plot.setLabel("left", "Speed (m/s)")
        self.sa_plot.setLabel("bottom", "Altitude (m)")
        self.sa_plot.showGrid(x=True, y=True, alpha=0.3)
        self.sa_curve = self.sa_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=5,
            symbolBrush="#7700cc",
            symbolPen=None,
            name="Speed vs altitude",
        )
        self.sa_plot.doubleClicked.connect(self._open_sa_detail)

        hint = QLabel("Double-click any chart to open it in a detail window.")
        hint.setStyleSheet("color: #555; font-style: italic; padding: 2px 4px;")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        layout.addWidget(hint)
        layout.addWidget(self.alt_plot)
        layout.addWidget(self.spd_plot)
        layout.addWidget(self.sa_plot)
        return container

    # ---- Pop-out chart dialogs ----
    def _open_alt_detail(self):
        def make(plot):
            return plot.plot(pen=pg.mkPen("#cc6600", width=2))
        dlg = ChartDialog(
            "Altitude vs Time", "Time (s)", "Altitude (m)",
            lambda: (self._t_alt, self._v_alt), make, self)
        dlg.setAttribute(Qt.WA_DeleteOnClose)
        dlg.show()

    def _open_spd_detail(self):
        def make(plot):
            return plot.plot(pen=pg.mkPen("#0077cc", width=2))
        dlg = ChartDialog(
            "Horizontal Speed vs Time", "Time (s)", "Speed (m/s)",
            lambda: (self._t_spd, self._v_spd), make, self)
        dlg.setAttribute(Qt.WA_DeleteOnClose)
        dlg.show()

    def _open_sa_detail(self):
        def make(plot):
            return plot.plot(
                pen=None, symbol="o", symbolSize=6,
                symbolBrush="#7700cc", symbolPen=None)
        dlg = ChartDialog(
            "Horizontal Speed vs Altitude", "Altitude (m)", "Speed (m/s)",
            lambda: (self._sa_alt, self._sa_spd), make, self)
        dlg.setAttribute(Qt.WA_DeleteOnClose)
        dlg.show()

    def _append_series(self, pkt):
        if not HAVE_PG:
            return
        if self._series_t0 is None:
            self._series_t0 = time.monotonic()
        t = time.monotonic() - self._series_t0

        alt = self._to_float(pkt.get("gps_alt"))
        if alt is None:
            alt = self._to_float(pkt.get("bmp_alt"))
        spd_kmh = self._to_float(pkt.get("speed"))
        spd = None if spd_kmh is None else spd_kmh / 3.6

        if alt is not None:
            self._t_alt.append(t)
            self._v_alt.append(alt)
            self.alt_curve.setData(self._t_alt, self._v_alt)
        if spd is not None:
            self._t_spd.append(t)
            self._v_spd.append(spd)
            self.spd_curve.setData(self._t_spd, self._v_spd)
        if alt is not None and spd is not None:
            self._sa_alt.append(alt)
            self._sa_spd.append(spd)
            self.sa_curve.setData(self._sa_alt, self._sa_spd)

    def _populate_charts_from_rows(self, rows):
        if not HAVE_PG:
            return
        self._t_alt = []
        self._v_alt = []
        self._t_spd = []
        self._v_spd = []
        self._sa_alt = []
        self._sa_spd = []
        self._series_t0 = None

        t0 = None
        for row in rows:
            ts = row.get("timestamp", "")
            try:
                t_abs = datetime.fromisoformat(ts).timestamp()
            except (ValueError, TypeError):
                continue
            if t0 is None:
                t0 = t_abs
            t = t_abs - t0

            alt = self._to_float(row.get("gps_alt_m"))
            if alt is None:
                alt = self._to_float(row.get("bmp_alt_m"))
            spd_kmh = self._to_float(row.get("speed_kmh"))
            spd = None if spd_kmh is None else spd_kmh / 3.6

            if alt is not None:
                self._t_alt.append(t)
                self._v_alt.append(alt)
            if spd is not None:
                self._t_spd.append(t)
                self._v_spd.append(spd)
            if alt is not None and spd is not None:
                self._sa_alt.append(alt)
                self._sa_spd.append(spd)

        self.alt_curve.setData(self._t_alt, self._v_alt)
        self.spd_curve.setData(self._t_spd, self._v_spd)
        self.sa_curve.setData(self._sa_alt, self._sa_spd)

    # ---- Serial port handling ----
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            label = f"{p.device} - {p.description}" if p.description else p.device
            self.port_combo.addItem(label, p.device)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def _toggle_connection(self):
        if self.reader and self.reader.isRunning():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if self.port_combo.count() == 0:
            self.statusBar().showMessage("No serial ports found.", 4000)
            return
        port = self.port_combo.currentData()
        baud = int(self.baud_combo.currentText())
        self.reader = SerialReader(port, baud)
        self.reader.line_received.connect(self._on_line)
        self.reader.packet_received.connect(self._on_packet)
        self.reader.error.connect(self._on_error)
        self.reader.connected.connect(self._on_connected)
        self.reader.start()

    def _disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.wait(2000)
            self.reader = None
        self.connect_btn.setText("Connect")
        self.statusBar().showMessage("Disconnected.", 3000)

    @pyqtSlot(bool)
    def _on_connected(self, ok):
        if ok:
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(
                f"Connected to {self.port_combo.currentData()} "
                f"@ {self.baud_combo.currentText()} baud.", 4000)
        else:
            self.connect_btn.setText("Connect")

    @pyqtSlot(str)
    def _on_error(self, msg):
        self.statusBar().showMessage(msg, 6000)
        self.log_view.appendPlainText(f"[ERROR] {msg}")

    # ---- Data flow ----
    @pyqtSlot(str)
    def _on_line(self, line):
        self.log_view.appendPlainText(line)

    @pyqtSlot(dict)
    def _on_packet(self, pkt):
        self.packet_count += 1
        self.status_count.setText(f"Packets: {self.packet_count}")

        # GPS outlier filter — drop positions clearly bogus
        pkt = self._filter_gps_outlier(pkt)

        self._update_flight_stats(pkt)

        self.lbl_time.setText(pkt.get("timestamp", "-"))
        self.lbl_temp.setText(self._clean(pkt.get("temp")))
        self.lbl_pressure.setText(self._clean(pkt.get("pressure")))
        self.lbl_bmp_alt.setText(self._clean(pkt.get("bmp_alt")))
        self.lbl_lat.setText(self._clean(pkt.get("lat")))
        self.lbl_lon.setText(self._clean(pkt.get("lon")))
        self.lbl_gps_alt.setText(self._clean(pkt.get("gps_alt")))
        self.lbl_speed.setText(self._kmh_to_ms_str(pkt.get("speed")))
        self.lbl_course.setText(self._clean(pkt.get("course")))
        self.lbl_heat.setText(self._clean(pkt.get("heat")))
        rssi = pkt.get("rssi")
        self.lbl_rssi.setText(str(rssi) if rssi is not None else "-")

        self.lbl_travel_time.setText(self._format_duration(self.travel_time_s))
        self.lbl_distance.setText(self._format_distance(self.total_distance_m))
        self.lbl_straight.setText(self._format_distance(self.straight_line_m))
        self.lbl_avg_speed.setText(f"{self.avg_speed_ms:.2f} m/s"
                                   if self.travel_time_s > 0 else "- m/s")
        self.lbl_avg_vspeed.setText(f"{self.avg_vspeed_ms:+.2f} m/s"
                                    if self._vspeed_count > 0 else "- m/s")
        self.lbl_max_alt.setText(f"{self.max_alt_m:.1f} m"
                                 if self.max_alt_m is not None else "- m")

        self._write_csv(pkt)
        self._update_map(pkt)
        self._append_series(pkt)

    def _update_flight_stats(self, pkt):
        now = time.monotonic()
        # Prefer GPS altitude, fall back to BMP
        alt = self._to_float(pkt.get("gps_alt"))
        if alt is None:
            alt = self._to_float(pkt.get("bmp_alt"))

        above = alt is not None and alt > ALT_THRESHOLD_M

        if above and self._was_above_threshold and self._last_packet_time is not None:
            delta = now - self._last_packet_time
            if 0 < delta < MAX_DELTA_S:
                self.travel_time_s += delta

        # Vertical speed: dh/dt vs previous valid altitude
        self._last_vspeed_ms = None
        if alt is not None and self._last_alt is not None and self._last_alt_t is not None:
            dt = now - self._last_alt_t
            if 0 < dt < MAX_DELTA_S:
                vspeed = (alt - self._last_alt) / dt
                self._last_vspeed_ms = vspeed
                if above:
                    self._vspeed_sum += vspeed
                    self._vspeed_count += 1

        if alt is not None:
            self._last_alt = alt
            self._last_alt_t = now
            if self.max_alt_m is None or alt > self.max_alt_m:
                self.max_alt_m = alt

        self._was_above_threshold = above
        self._last_packet_time = now

        cur_lat = self._to_float(pkt.get("lat"))
        cur_lon = self._to_float(pkt.get("lon"))

        # Capture launch point on first valid fix (mirrors transmitter behaviour)
        if (self._start_position is None and
                cur_lat is not None and cur_lon is not None):
            self._start_position = (cur_lat, cur_lon)

        # Cumulative distance — only while above threshold (in flight)
        if above and cur_lat is not None and cur_lon is not None:
            if self._last_position is not None:
                d = self._haversine_m(
                    self._last_position[0], self._last_position[1],
                    cur_lat, cur_lon,
                )
                self.total_distance_m += d
            self._last_position = (cur_lat, cur_lon)
        elif not above:
            # Below threshold -> reset segment anchor so distance only counts in flight
            self._last_position = None

        # Straight-line distance from launch — what the transmitter measures
        if (self._start_position is not None and
                cur_lat is not None and cur_lon is not None):
            self.straight_line_m = self._haversine_m(
                self._start_position[0], self._start_position[1],
                cur_lat, cur_lon,
            )

        # Running averages
        if self.travel_time_s > 0:
            self.avg_speed_ms = self.total_distance_m / self.travel_time_s
        if self._vspeed_count > 0:
            self.avg_vspeed_ms = self._vspeed_sum / self._vspeed_count

    @staticmethod
    def _clean(value):
        if value is None:
            return "-"
        s = str(value).strip()
        if not s or s.upper() == "N/A":
            return "-"
        return s

    @staticmethod
    def _kmh_to_ms_str(value, fmt="{:.2f}"):
        """Convert a km/h scalar (string or number) to an m/s display string."""
        if value is None:
            return "-"
        s = str(value).strip()
        if not s or s.upper() == "N/A":
            return "-"
        try:
            return fmt.format(float(s) / 3.6)
        except (TypeError, ValueError):
            return "-"

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        R = 6378137.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlmb = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2)
        return 2 * R * math.asin(math.sqrt(a))

    @staticmethod
    def _format_duration(seconds):
        s = int(seconds)
        h, rem = divmod(s, 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    @staticmethod
    def _format_distance(meters):
        if meters < 1000:
            return f"{meters:.0f} m"
        return f"{meters / 1000:.2f} km"

    # ---- CSV log ----
    def _open_log_file(self):
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(log_dir, f"lora_log_{stamp}.csv")
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "temp_C", "pressure_Pa", "bmp_alt_m",
            "lat", "lon", "gps_alt_m", "speed_kmh", "course_deg",
            "heater", "rssi_dBm", "travel_time_s", "distance_m",
            "straight_line_m", "raw",
        ])
        self.csv_file.flush()
        self.status_csv.setText(f"Logging to: {path}")

    def _write_csv(self, pkt):
        if not self.csv_writer:
            return

        def cell(key):
            v = pkt.get(key)
            if v is None:
                return ""
            s = str(v).strip()
            return "" if s.upper() == "N/A" else s

        self.csv_writer.writerow([
            pkt.get("timestamp", ""),
            cell("temp"),
            cell("pressure"),
            cell("bmp_alt"),
            cell("lat"),
            cell("lon"),
            cell("gps_alt"),
            cell("speed"),
            cell("course"),
            cell("heat"),
            "" if pkt.get("rssi") is None else pkt["rssi"],
            f"{self.travel_time_s:.1f}",
            f"{self.total_distance_m:.1f}",
            f"{self.straight_line_m:.1f}",
            pkt.get("raw", ""),
        ])
        self.csv_file.flush()

    # ---- Map ----
    def _update_map(self, pkt):
        lat = self._to_float(pkt.get("lat"))
        lon = self._to_float(pkt.get("lon"))
        if lat is None or lon is None:
            return
        vspeed_txt = ("-" if self._last_vspeed_ms is None
                      else f"{self._last_vspeed_ms:+.2f}")
        info = (
            f"<b>{pkt.get('timestamp','')}</b><br>"
            f"Alt (GPS): {pkt.get('gps_alt','-')} m<br>"
            f"Speed: {self._kmh_to_ms_str(pkt.get('speed'))} m/s<br>"
            f"V-speed: {vspeed_txt} m/s<br>"
            f"Course: {pkt.get('course','-')}&deg;<br>"
            f"Heater: {pkt.get('heat','-')}<br>"
            f"RSSI: {pkt.get('rssi','-')} dBm"
        )
        info_js = info.replace("\\", "\\\\").replace("'", "\\'")
        is_first = "false"
        if not self.first_fix_logged:
            is_first = "true"
            self.first_fix_logged = True
        speed = self._to_float(pkt.get("speed"))
        course = self._to_float(pkt.get("course"))
        speed_js = "null" if speed is None else f"{speed}"
        course_js = "null" if course is None else f"{course}"
        self.map_view.page().runJavaScript(
            f"updatePosition({lat}, {lon}, '{info_js}', {is_first}, "
            f"{speed_js}, {course_js});"
        )

    def _clear_trail(self):
        self.first_fix_logged = False
        self.travel_time_s = 0.0
        self.total_distance_m = 0.0
        self.straight_line_m = 0.0
        self.avg_speed_ms = 0.0
        self.avg_vspeed_ms = 0.0
        self.max_alt_m = None
        self._vspeed_sum = 0.0
        self._vspeed_count = 0
        self._last_alt = None
        self._last_alt_t = None
        self._last_vspeed_ms = None
        self._start_position = None
        self._last_packet_time = None
        self._was_above_threshold = False
        self._last_position = None
        self.lbl_travel_time.setText("00:00:00")
        self.lbl_distance.setText("0 m")
        self.lbl_straight.setText("0 m")
        self.lbl_avg_speed.setText("- m/s")
        self.lbl_avg_vspeed.setText("- m/s")
        self.lbl_max_alt.setText("- m")
        self._t_alt = []
        self._v_alt = []
        self._t_spd = []
        self._v_spd = []
        self._sa_alt = []
        self._sa_spd = []
        self._series_t0 = None
        if HAVE_PG and hasattr(self, "alt_curve"):
            self.alt_curve.setData([], [])
            self.spd_curve.setData([], [])
            self.sa_curve.setData([], [])
        self.map_view.page().runJavaScript("clearTrail();")

    def _load_log(self):
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        if not os.path.isdir(log_dir):
            log_dir = os.path.dirname(os.path.abspath(__file__))
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Log CSV", log_dir,
            "CSV files (*.csv);;All files (*)",
        )
        if not path:
            return

        if self.reader and self.reader.isRunning():
            self._disconnect()

        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                rows = list(csv.DictReader(f))
        except Exception as e:
            self.statusBar().showMessage(f"Error loading log: {e}", 6000)
            return

        if not rows:
            self.statusBar().showMessage("Log file is empty.", 4000)
            return

        # Reset map + stats
        self._clear_trail()

        # Annotate rows with vertical speed, detect outliers, interpolate.
        positions = self._build_positions(rows)

        # Build trail points (with per-point popup) and pick last valid row.
        points = []
        last_valid_row = None
        interp_count = 0
        for p in positions:
            if p["lat"] is None or p["lon"] is None:
                continue
            info = self._row_to_popup(p["row"])
            if p["interpolated"]:
                info = "<b>[Interpolated]</b><br>" + info
                interp_count += 1
            points.append({
                "lat": p["lat"],
                "lon": p["lon"],
                "info": info,
                "interpolated": p["interpolated"],
            })
            if not p["interpolated"]:
                last_valid_row = p["row"]

        last_row = last_valid_row or rows[-1]

        # Stats from CSV if available, else recompute
        self.travel_time_s = self._safe_float(last_row.get("travel_time_s"), 0.0)
        self.total_distance_m = self._safe_float(last_row.get("distance_m"), 0.0)
        self.straight_line_m = self._safe_float(last_row.get("straight_line_m"), 0.0)

        if self.travel_time_s == 0.0 and self.total_distance_m == 0.0:
            # Older log without stat columns - recompute from rows
            self._recompute_stats_from_rows(rows)

        # Always (re)compute straight-line from first valid fix in this log
        first_lat = first_lon = None
        for row in rows:
            la = self._to_float(row.get("lat"))
            lo = self._to_float(row.get("lon"))
            if la is not None and lo is not None:
                first_lat, first_lon = la, lo
                break
        last_lat = self._to_float(last_row.get("lat"))
        last_lon = self._to_float(last_row.get("lon"))
        if (first_lat is not None and last_lat is not None
                and first_lon is not None and last_lon is not None):
            self._start_position = (first_lat, first_lon)
            self.straight_line_m = self._haversine_m(
                first_lat, first_lon, last_lat, last_lon)

        # Update telemetry labels from last row
        self.lbl_time.setText(last_row.get("timestamp", "-") or "-")
        self.lbl_temp.setText(self._clean(last_row.get("temp_C")))
        self.lbl_pressure.setText(self._clean(last_row.get("pressure_Pa")))
        self.lbl_bmp_alt.setText(self._clean(last_row.get("bmp_alt_m")))
        self.lbl_lat.setText(self._clean(last_row.get("lat")))
        self.lbl_lon.setText(self._clean(last_row.get("lon")))
        self.lbl_gps_alt.setText(self._clean(last_row.get("gps_alt_m")))
        self.lbl_speed.setText(self._kmh_to_ms_str(last_row.get("speed_kmh")))
        self.lbl_course.setText(self._clean(last_row.get("course_deg")))
        self.lbl_heat.setText(self._clean(last_row.get("heater")))
        self.lbl_rssi.setText(self._clean(last_row.get("rssi_dBm")))

        # Derive averages + max altitude from loaded data
        self.avg_speed_ms = (self.total_distance_m / self.travel_time_s
                             if self.travel_time_s > 0 else 0.0)

        vs_sum, vs_count = 0.0, 0
        self.max_alt_m = None
        for row in rows:
            alt = self._to_float(row.get("gps_alt_m"))
            if alt is None:
                alt = self._to_float(row.get("bmp_alt_m"))
            if alt is not None:
                if self.max_alt_m is None or alt > self.max_alt_m:
                    self.max_alt_m = alt
            v = row.get("_vspeed_ms")
            if v is not None and alt is not None and alt > ALT_THRESHOLD_M:
                vs_sum += v
                vs_count += 1
        self.avg_vspeed_ms = vs_sum / vs_count if vs_count > 0 else 0.0

        self.lbl_travel_time.setText(self._format_duration(self.travel_time_s))
        self.lbl_distance.setText(self._format_distance(self.total_distance_m))
        self.lbl_straight.setText(self._format_distance(self.straight_line_m))
        self.lbl_avg_speed.setText(f"{self.avg_speed_ms:.2f} m/s"
                                   if self.travel_time_s > 0 else "- m/s")
        self.lbl_avg_vspeed.setText(f"{self.avg_vspeed_ms:+.2f} m/s"
                                    if vs_count > 0 else "- m/s")
        self.lbl_max_alt.setText(f"{self.max_alt_m:.1f} m"
                                 if self.max_alt_m is not None else "- m")

        self.packet_count = len(rows)
        self.status_count.setText(f"Packets: {self.packet_count}")

        # Plot the track on the map
        if points:
            popup = (
                f"<b>End</b><br>{last_row.get('timestamp', '')}<br>"
                f"Alt (GPS): {self._clean(last_row.get('gps_alt_m'))} m<br>"
                f"Speed: {self._clean(last_row.get('speed_kmh'))} km/h<br>"
                f"Heater: {self._clean(last_row.get('heater'))}"
            )
            speed = self._to_float(last_row.get("speed_kmh"))
            course = self._to_float(last_row.get("course_deg"))
            self.map_view.page().runJavaScript(
                f"loadTrack({json.dumps(points)}, {json.dumps(popup)}, "
                f"{'null' if speed is None else speed}, "
                f"{'null' if course is None else course});"
            )
            self.first_fix_logged = True

        self._populate_charts_from_rows(rows)

        extra = f", {interp_count} interpolated" if interp_count else ""
        self.statusBar().showMessage(
            f"Loaded {len(rows)} rows ({len(points)} fixes{extra}) from "
            f"{os.path.basename(path)}", 8000)

    def _recompute_stats_from_rows(self, rows):
        """Walk old log rows to derive travel_time_s and total_distance_m
        for logs that predate the stat columns. Skips GPS outliers."""
        self.travel_time_s = 0.0
        self.total_distance_m = 0.0
        # Outlier reference: first valid GPS position
        first_lat = first_lon = None
        for row in rows:
            la = self._to_float(row.get("lat"))
            lo = self._to_float(row.get("lon"))
            if la is not None and lo is not None:
                first_lat, first_lon = la, lo
                break

        last_t = None
        was_above = False
        last_pos = None
        for row in rows:
            ts = row.get("timestamp", "")
            try:
                t = datetime.fromisoformat(ts).timestamp()
            except (ValueError, TypeError):
                t = None
            alt = self._to_float(row.get("gps_alt_m"))
            if alt is None:
                alt = self._to_float(row.get("bmp_alt_m"))
            above = alt is not None and alt > ALT_THRESHOLD_M

            if above and was_above and last_t is not None and t is not None:
                delta = t - last_t
                if 0 < delta < MAX_DELTA_S:
                    self.travel_time_s += delta

            if above:
                lat = self._to_float(row.get("lat"))
                lon = self._to_float(row.get("lon"))
                if lat is not None and lon is not None and first_lat is not None:
                    if self._haversine_m(first_lat, first_lon,
                                         lat, lon) > OUTLIER_THRESHOLD_M:
                        lat = lon = None
                if lat is not None and lon is not None:
                    if last_pos is not None:
                        self.total_distance_m += self._haversine_m(
                            last_pos[0], last_pos[1], lat, lon)
                    last_pos = (lat, lon)
            else:
                last_pos = None

            was_above = above
            if t is not None:
                last_t = t

    def _row_to_popup(self, row):
        vs = row.get("_vspeed_ms")
        vs_txt = "-" if vs is None else f"{vs:+.2f}"
        return (
            f"<b>{row.get('timestamp', '')}</b><br>"
            f"Temp: {self._clean(row.get('temp_C'))} C<br>"
            f"Pressure: {self._clean(row.get('pressure_Pa'))} Pa<br>"
            f"BMP alt: {self._clean(row.get('bmp_alt_m'))} m<br>"
            f"GPS alt: {self._clean(row.get('gps_alt_m'))} m<br>"
            f"Speed: {self._kmh_to_ms_str(row.get('speed_kmh'))} m/s<br>"
            f"V-speed: {vs_txt} m/s<br>"
            f"Course: {self._clean(row.get('course_deg'))}&deg;<br>"
            f"Heater: {self._clean(row.get('heater'))}<br>"
            f"RSSI: {self._clean(row.get('rssi_dBm'))} dBm"
        )

    def _filter_gps_outlier(self, pkt):
        """If lat/lon is >OUTLIER_THRESHOLD_M from start, blank them out so
        downstream stats and the map ignore the bogus fix."""
        lat = self._to_float(pkt.get("lat"))
        lon = self._to_float(pkt.get("lon"))
        if lat is None or lon is None or self._start_position is None:
            return pkt
        d = self._haversine_m(self._start_position[0], self._start_position[1],
                              lat, lon)
        if d <= OUTLIER_THRESHOLD_M:
            return pkt
        # Outlier — null out position and flag in the status bar
        clone = dict(pkt)
        clone["lat"] = "N/A"
        clone["lon"] = "N/A"
        self.statusBar().showMessage(
            f"GPS outlier filtered: {d / 1000:.0f} km from start", 4000)
        return clone

    def _build_positions(self, rows):
        """Annotate each row with _vspeed_ms, detect outliers vs the first
        valid fix, and linearly interpolate outlier positions between the
        nearest non-outlier neighbours. Returns a list of dicts:
            {lat, lon, interpolated: bool, row: <csv row>}
        Outlier rows with no non-outlier neighbours get lat/lon = None."""
        # Vertical speed per row (uses timestamp deltas)
        prev_alt = None
        prev_t = None
        for row in rows:
            alt = self._to_float(row.get("gps_alt_m"))
            if alt is None:
                alt = self._to_float(row.get("bmp_alt_m"))
            try:
                t = datetime.fromisoformat(row.get("timestamp", "")).timestamp()
            except (ValueError, TypeError):
                t = None
            v = None
            if (alt is not None and prev_alt is not None
                    and t is not None and prev_t is not None):
                dt = t - prev_t
                if 0 < dt < MAX_DELTA_S:
                    v = (alt - prev_alt) / dt
            row["_vspeed_ms"] = v
            if alt is not None and t is not None:
                prev_alt = alt
                prev_t = t

        # First valid position is our outlier reference
        first_lat = first_lon = None
        for row in rows:
            la = self._to_float(row.get("lat"))
            lo = self._to_float(row.get("lon"))
            if la is not None and lo is not None:
                first_lat, first_lon = la, lo
                break

        positions = []
        for row in rows:
            la = self._to_float(row.get("lat"))
            lo = self._to_float(row.get("lon"))
            is_outlier = False
            if (la is not None and lo is not None
                    and first_lat is not None):
                d = self._haversine_m(first_lat, first_lon, la, lo)
                if d > OUTLIER_THRESHOLD_M:
                    is_outlier = True
            positions.append({
                "lat": None if is_outlier else la,
                "lon": None if is_outlier else lo,
                "is_outlier_raw": is_outlier,
                "interpolated": False,
                "row": row,
            })

        # Interpolate outliers by walking the list and finding non-outlier
        # neighbours on either side.
        for i, p in enumerate(positions):
            if not p["is_outlier_raw"]:
                continue
            prev_i = next_i = None
            for j in range(i - 1, -1, -1):
                q = positions[j]
                if not q["is_outlier_raw"] and q["lat"] is not None:
                    prev_i = j
                    break
            for j in range(i + 1, len(positions)):
                q = positions[j]
                if not q["is_outlier_raw"] and q["lat"] is not None:
                    next_i = j
                    break
            if prev_i is not None and next_i is not None:
                frac = (i - prev_i) / (next_i - prev_i)
                a = positions[prev_i]
                b = positions[next_i]
                p["lat"] = a["lat"] + frac * (b["lat"] - a["lat"])
                p["lon"] = a["lon"] + frac * (b["lon"] - a["lon"])
                p["interpolated"] = True
            elif prev_i is not None:
                p["lat"] = positions[prev_i]["lat"]
                p["lon"] = positions[prev_i]["lon"]
                p["interpolated"] = True
            elif next_i is not None:
                p["lat"] = positions[next_i]["lat"]
                p["lon"] = positions[next_i]["lon"]
                p["interpolated"] = True

        return positions

    @staticmethod
    def _safe_float(value, default=0.0):
        try:
            s = str(value).strip()
            if not s or s.upper() == "N/A":
                return default
            return float(s)
        except (TypeError, ValueError):
            return default

    def _set_autocenter(self, on):
        self.map_view.page().runJavaScript(
            f"setAutoCenter({'true' if on else 'false'});"
        )

    @staticmethod
    def _to_float(s):
        if s is None:
            return None
        try:
            return float(s)
        except (TypeError, ValueError):
            return None

    # ---- Shutdown ----
    def closeEvent(self, event):
        self._disconnect()
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
