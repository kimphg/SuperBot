#!/usr/bin/env python3
"""
NiclaViewer — PyQt5 port of the C++ NiclaView application.
Receives IMU data and video from a Nicla Vision board via USB CDC serial.

Install:
    pip install PyQt5
    # PyQt5 5.15+ includes QtChart and QtSerialPort.
    # If QtChart is missing separately: pip install PyQtChart
"""

import sys
import os
import struct
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QHBoxLayout,
    QVBoxLayout, QFrame, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSizePolicy, QStatusBar,
)
from PyQt5.QtCore import Qt, QTimer, QIODevice, QUrl, QMargins, pyqtSlot
from PyQt5.QtGui import (
    QPixmap, QPen, QColor, QFont, QPainter, QTextDocument, QDesktopServices,
)
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtPrintSupport import QPrinter

try:
    from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
    _HAS_CHART = True
except ImportError:
    _HAS_CHART = False
    print("WARNING: PyQt5.QtChart not found. Install PyQtChart.")


# ─── Frame protocol constants (must match firmware) ───────────────────────────
_MAGIC       = b"NICL"
_HEADER_SIZE = 9          # 4 magic + 1 type + 4 length (big-endian)
_TYPE_IMU    = 0x01
_TYPE_VIDEO  = 0x02

# ─── USB vendor IDs ───────────────────────────────────────────────────────────
_VID_STM32   = 0x0483
_VID_ARDUINO = 0x2341

# ─── Stylesheets ─────────────────────────────────────────────────────────────
_SS_PANEL  = "background-color:rgb(222,255,222);color:rgb(0,0,0);border:2px solid gray;"
_SS_ACTIVE = "background-color:rgb(0,150,0);color:white;font:10pt 'MS Shell Dlg 2';border:3px solid gray;"
_SS_DANGER = "background-color:rgb(180,0,0);color:white;font:10pt 'MS Shell Dlg 2';border:3px solid gray;"
_SS_NORMAL = "background-color:rgb(32,64,128);color:white;font:10pt 'MS Shell Dlg 2';border:3px solid gray;"
_SS_GRAY   = "background-color:rgb(96,96,96);color:rgb(160,160,160);font:10pt 'MS Shell Dlg 2';border:3px solid gray;"
_SS_INPUT  = "background-color:white;color:black;"
_SS_VIDEO  = "border:1px solid #aaa;background:#222;"


# ─── PDF / report ─────────────────────────────────────────────────────────────
class PdfReport:
    """Generates cumulative HTML-to-PDF reports."""

    def __init__(self):
        self.last_filename = ""
        self._count = 0
        self._reset()

    def _reset(self):
        date_str = datetime.now().strftime("%d/%m/%Y")
        self._html = f"""
<head><style>* {{margin:0;padding:0;}}</style></head>
<table border="0" width="100%" cellpadding="5" cellspacing="0">
  <tr><th>BỆNH VIỆN TRUNG ƯƠNG QUÂN ĐỘI 108<br>
          KHOA CHẤN THƯƠNG CHỈNH HÌNH CỘT SỐNG<br>
          Số 1 Trần Hưng Đạo, Hai Bà Trưng, Hà Nội</th></tr>
</table>
<div>Hà Nội, {date_str}</div>
<h1 align="center">KẾT QUẢ ĐO</h1><br>
<table border="1" width="100%" cellpadding="5" cellspacing="0">
  <tr>
    <th>STT</th><th>Tên BN</th><th>Mã BN</th><th>Năm sinh</th>
    <th>Thời gian</th><th>Biên độ cúi ngẩng</th><th>Biên độ xoay ngang</th>
  </tr>
"""

    def get_count(self) -> int:
        return self._count + 1

    def insert_record(self, name: str, code: str, year: str,
                      tilt: str, pan: str):
        self._count += 1
        now  = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
        base = datetime.now().strftime("%d_%m_%Y_%H_%M") + f"_{name}"
        self._html += (
            f"<tr><td>{self._count}</td><td>{name}</td><td>{code}</td>"
            f"<td>{year}</td><td>{now}</td><td>{tilt}</td><td>{pan}</td></tr>"
            f"</table><br><h3>Kết quả chi tiết:</h3>"
            f"<div><img src='{base}.png'></div><br>"
            f"<h3>Ảnh chụp trường nhìn:</h3>"
            f"<div><img src='{base}video.png'></div>"
            f"<div align='right'>Nhân viên đo</div>"
        )
        self.last_filename = base + ".pdf"
        self._save_pdf(self.last_filename)

    def _save_pdf(self, filename: str):
        doc = QTextDocument()
        doc.setHtml(self._html)
        printer = QPrinter(QPrinter.HighResolution)
        printer.setOutputFormat(QPrinter.PdfFormat)
        printer.setOutputFileName(filename)
        printer.setPageSize(QPrinter.A4)
        printer.setPageMargins(15, 15, 15, 15, QPrinter.Millimeter)
        doc.print_(printer)

    def reset(self):
        self._count = 0
        self._reset()


# ─── Helpers ──────────────────────────────────────────────────────────────────
def _video_label(placeholder: str = "") -> QLabel:
    lbl = QLabel(placeholder)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setScaledContents(True)
    lbl.setMinimumSize(160, 120)
    lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    lbl.setStyleSheet(_SS_VIDEO)
    return lbl


def _btn(text: str, style: str = _SS_NORMAL) -> QPushButton:
    b = QPushButton(text)
    b.setStyleSheet(style)
    b.setMinimumHeight(40)
    return b


# ─── Main window ─────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Phần mềm kiểm tra góc cột sống")

        # ── measurement state ─────────────────────────────────────────────
        self._report      = PdfReport()
        self._pixmap      = QPixmap()
        self._is_measuring = False
        self._is_recording = False
        self._roll  = 0.0
        self._pitch = 0.0
        self._roll_min  =  1000.0;  self._roll_max  = -1000.0
        self._pitch_min =  1000.0;  self._pitch_max = -1000.0
        self._img_ready = False

        # ── chart state ───────────────────────────────────────────────────
        self._first_ts   = -1
        self._max_elapsed = 10.0

        # ── serial state ──────────────────────────────────────────────────
        self._serial_buf = b""

        self._build_ui()
        self._build_chart()
        self._build_serial()
        self._update_buttons()
        self.showMaximized()

    # ══════════════════════════════════════════════════════════════════════
    # UI construction
    # ══════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        outer = QGridLayout(central)
        outer.setSpacing(4)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setColumnStretch(0, 3)
        outer.setColumnStretch(1, 2)
        outer.setRowStretch(0, 4)   # video / chart — tallest row

        # ── left col, rows 0-3: video cross ──────────────────────────────
        self._video_frame = QFrame()
        self._video_frame.setStyleSheet(
            "background-color:rgb(222,255,222);"
            "border:4px solid blue;")
        vgrid = QGridLayout(self._video_frame)
        vgrid.setSpacing(2);  vgrid.setContentsMargins(2, 2, 2, 2)

        self.lbl_top    = _video_label("top")
        self.lbl_left   = _video_label("left")
        self.lbl_center = _video_label("video")
        self.lbl_right  = _video_label("right")
        self.lbl_bot    = _video_label("bottom")

        vgrid.addWidget(self.lbl_top,    0, 1)
        vgrid.addWidget(self.lbl_left,   1, 0)
        vgrid.addWidget(self.lbl_center, 1, 1)
        vgrid.addWidget(self.lbl_right,  1, 2)
        vgrid.addWidget(self.lbl_bot,    2, 1)
        outer.addWidget(self._video_frame, 0, 0, 4, 1)

        # ── left col, row 4: patient info ─────────────────────────────────
        patient_box = QGroupBox("Thông tin bệnh nhân")
        patient_box.setStyleSheet(_SS_PANEL)
        pgrid = QGridLayout(patient_box)

        self._edit_name = QLineEdit(); self._edit_name.setStyleSheet(_SS_INPUT); self._edit_name.setMinimumHeight(36)
        self._edit_code = QLineEdit(); self._edit_code.setStyleSheet(_SS_INPUT); self._edit_code.setMinimumHeight(36)
        self._edit_year = QLineEdit(); self._edit_year.setStyleSheet(_SS_INPUT); self._edit_year.setMinimumHeight(36)

        for row, (lbl_text, edit) in enumerate([
            ("Tên BN:", self._edit_name),
            ("Mã BN:", self._edit_code),
            ("Năm sinh:", self._edit_year),
        ]):
            pgrid.addWidget(QLabel(lbl_text), row, 0)
            pgrid.addWidget(edit,             row, 1)

        outer.addWidget(patient_box, 4, 0)

        # ── right col, row 0: chart placeholder (filled in _build_chart) ─
        self._chart_container = QWidget()
        self._chart_container.setMinimumHeight(280)
        sp = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sp.setVerticalStretch(3)
        self._chart_container.setSizePolicy(sp)
        outer.addWidget(self._chart_container, 0, 1)

        # ── right col, row 1: measurement controls ────────────────────────
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet(_SS_PANEL)
        cgrid = QGridLayout(ctrl_frame)

        self._lbl_count = QLabel("--")
        cgrid.addWidget(QLabel("Lần đo:"), 0, 0)
        cgrid.addWidget(self._lbl_count,   0, 1)

        self._btn_start = _btn("Bắt đầu đo", _SS_ACTIVE)
        self._btn_stop  = _btn("Dừng",        _SS_GRAY)
        self._btn_reset = _btn("Đo lại từ đầu", _SS_NORMAL)

        self._btn_start.clicked.connect(self._start)
        self._btn_stop.clicked.connect(self._stop)
        self._btn_reset.clicked.connect(self._reset)

        cgrid.addWidget(self._btn_start, 1, 0, 1, 2)
        cgrid.addWidget(self._btn_stop,  2, 0, 1, 2)
        cgrid.addWidget(self._btn_reset, 3, 0, 1, 2)
        outer.addWidget(ctrl_frame, 1, 1)

        # ── right col, row 2: instructions ────────────────────────────────
        instr = QLabel(
            "Hướng dẫn: Thiết bị đo được đeo lên mắt bệnh nhân sao cho "
            "hình ảnh camera trùng khớp với hướng nhìn của bệnh nhân.\n"
            "Người kiểm tra đứng phía sau bệnh nhân để hướng dẫn và giám sát.\n"
            "Bệnh nhân lần lượt thực hiện các động tác cúi, ngẩng, xoay trái "
            "và xoay phải đến tới hạn cho phép.\n"
            "Người kiểm tra nhấn nút hoàn thành để lưu kết quả đo."
        )
        instr.setWordWrap(True)
        instr.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        instr.setFont(QFont("MS Shell Dlg 2", 10))
        outer.addWidget(instr, 2, 1)

        # ── right col, row 3: note ────────────────────────────────────────
        note = QLabel(
            "Chú thích: giá trị đo được có đơn vị là độ, giá trị tăng dần "
            "về hướng tương ứng với chiều từ trái qua phải, tăng dần về tầm "
            "tương ứng chiều từ dưới lên trên."
        )
        note.setWordWrap(True)
        note.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        note.setFont(QFont("MS Shell Dlg 2", 10))
        outer.addWidget(note, 3, 1)

        # ── right col, row 4: results ─────────────────────────────────────
        res_box = QGroupBox("Kết quả:")
        res_box.setStyleSheet("background-color:rgb(222,255,222);color:rgb(0,0,0);")
        rgrid = QGridLayout(res_box)
        rgrid.setContentsMargins(4, 15, 4, 4)

        self._lbl_pitch     = QLabel("--")
        self._lbl_roll      = QLabel("--")
        self._lbl_pitch_min = QLabel("--")
        self._lbl_pitch_max = QLabel("--")
        self._lbl_roll_min  = QLabel("--")
        self._lbl_roll_max  = QLabel("--")

        for col, (header, val) in enumerate([
            ("Góc ngẩng hiện tại:", self._lbl_pitch),
            ("Góc cúi thấp nhất:", self._lbl_pitch_min),
            ("Góc ngẩng cao nhất:", self._lbl_pitch_max),
        ]):
            rgrid.addWidget(QLabel(header), 0, col * 2)
            rgrid.addWidget(val,             0, col * 2 + 1)

        for col, (header, val) in enumerate([
            ("Góc xoay trái phải:", self._lbl_roll),
            ("Tới hạn trái:",      self._lbl_roll_min),
            ("Tới hạn phải:",      self._lbl_roll_max),
        ]):
            rgrid.addWidget(QLabel(header), 1, col * 2)
            rgrid.addWidget(val,             1, col * 2 + 1)

        self._btn_save     = _btn("Lưu kết quả")
        self._btn_open_pdf = _btn("Mở file báo cáo")
        self._btn_save.clicked.connect(self._save)
        self._btn_open_pdf.clicked.connect(self._open_pdf)
        rgrid.addWidget(self._btn_save,     2, 0, 1, 5)
        rgrid.addWidget(self._btn_open_pdf, 2, 5)

        outer.addWidget(res_box, 4, 1)

        self.statusBar().showMessage("Nicla not connected")

    # ══════════════════════════════════════════════════════════════════════
    # Chart
    # ══════════════════════════════════════════════════════════════════════
    def _build_chart(self):
        if not _HAS_CHART:
            lbl = QLabel("QtChart not available.\nRun: pip install PyQtChart")
            lbl.setAlignment(Qt.AlignCenter)
            lay = QHBoxLayout(self._chart_container)
            lay.addWidget(lbl)
            self._roll_series = self._pitch_series = None
            self._x_axis = self._y_axis = None
            return

        self._roll_series  = QLineSeries()
        self._pitch_series = QLineSeries()
        self._roll_series.setName("Góc xoay (roll)")
        self._pitch_series.setName("Góc ngẩng (pitch)")

        rp = QPen(QColor(220, 0, 0));  rp.setWidth(2); self._roll_series.setPen(rp)
        pp = QPen(QColor(0, 0, 200));  pp.setWidth(2); self._pitch_series.setPen(pp)

        self._chart = QChart()
        self._chart.addSeries(self._roll_series)
        self._chart.addSeries(self._pitch_series)
        self._chart.setTitle("")
        self._chart.setMargins(QMargins(4, 4, 4, 4))
        self._chart.legend().setVisible(True)
        self._chart.legend().setAlignment(Qt.AlignTop)

        self._x_axis = QValueAxis()
        self._x_axis.setTitleText("Thời gian (s)")
        self._x_axis.setRange(0, 10)
        self._x_axis.setTickCount(11)
        self._x_axis.setLabelFormat("%.0f")
        self._x_axis.setGridLineVisible(True)

        self._y_axis = QValueAxis()
        self._y_axis.setTitleText("Góc (°)")
        self._y_axis.setRange(-180, 180)
        self._y_axis.setTickCount(13)
        self._y_axis.setLabelFormat("%.0f")
        self._y_axis.setGridLineVisible(True)

        self._chart.addAxis(self._x_axis, Qt.AlignBottom)
        self._chart.addAxis(self._y_axis, Qt.AlignLeft)
        self._roll_series.attachAxis(self._x_axis)
        self._roll_series.attachAxis(self._y_axis)
        self._pitch_series.attachAxis(self._x_axis)
        self._pitch_series.attachAxis(self._y_axis)

        self._chart_view = QChartView(self._chart)
        self._chart_view.setRenderHint(QPainter.Antialiasing)

        lay = QHBoxLayout(self._chart_container)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._chart_view)

    def _chart_reset(self):
        if self._roll_series:
            self._roll_series.clear()
            self._pitch_series.clear()
        self._first_ts    = -1
        self._max_elapsed = 10.0
        if self._x_axis:
            self._x_axis.setRange(0, 10)
            self._x_axis.setTickCount(11)

    def _chart_append(self, ts_ms: int, roll: float, pitch_display: float):
        if not self._roll_series or not self._is_recording:
            return
        if self._first_ts < 0:
            self._first_ts = ts_ms
        elapsed = (ts_ms - self._first_ts) / 1000.0
        self._roll_series.append(elapsed, roll)
        self._pitch_series.append(elapsed, pitch_display)
        if elapsed >= self._max_elapsed:
            self._max_elapsed = elapsed + 10.0
            ticks = min(int(self._max_elapsed / 10) + 1, 20)
            self._x_axis.setRange(0, self._max_elapsed)
            self._x_axis.setTickCount(ticks + 1)

    # ══════════════════════════════════════════════════════════════════════
    # Serial port
    # ══════════════════════════════════════════════════════════════════════
    def _build_serial(self):
        self._serial = QSerialPort(self)
        self._serial.setBaudRate(QSerialPort.Baud115200)
        self._serial.readyRead.connect(self._on_serial_data)
        self._serial.errorOccurred.connect(self._on_serial_error)

        self._retry_timer = QTimer(self)
        self._retry_timer.timeout.connect(self._retry)
        self._auto_connect()

    def _auto_connect(self):
        ports = QSerialPortInfo.availablePorts()
        # Pass 1: match by known VID
        for info in ports:
            if info.vendorIdentifier() in (_VID_STM32, _VID_ARDUINO):
                self._connect(info.portName())
                return
        # Pass 2: match by description keyword
        for info in ports:
            desc = info.description().lower()
            if any(k in desc for k in ("openmv", "nicla", "stm32", "virtual com")):
                self._connect(info.portName())
                return
        self.statusBar().showMessage(
            "Nicla not found — install OpenMV IDE driver, then plug in USB")
        self._retry_timer.start(2000)

    def _connect(self, port_name: str):
        self._serial.setPortName(port_name)
        if self._serial.open(QIODevice.ReadWrite):
            self._serial.setDataTerminalReady(True)
            self._retry_timer.stop()
            self.statusBar().showMessage(f"Connected: {port_name}")
        else:
            self.statusBar().showMessage(f"Cannot open {port_name} — retrying…")
            self._retry_timer.start(2000)

    @pyqtSlot()
    def _retry(self):
        if not self._serial.isOpen():
            self._auto_connect()

    @pyqtSlot(QSerialPort.SerialPortError)
    def _on_serial_error(self, err):
        if err == QSerialPort.ResourceError:
            self._serial.close()
            self._serial_buf = b""
            self.statusBar().showMessage("Nicla disconnected — reconnecting…")
            self._retry_timer.start(2000)

    # ── Frame parsing ─────────────────────────────────────────────────────
    @pyqtSlot()
    def _on_serial_data(self):
        self._serial_buf += bytes(self._serial.readAll())
        self._parse_buffer()

    def _parse_buffer(self):
        buf = self._serial_buf
        while True:
            idx = buf.find(_MAGIC)
            if idx < 0:
                buf = buf[-3:] if len(buf) > 3 else buf
                break
            if idx > 0:
                buf = buf[idx:]
                continue
            if len(buf) < _HEADER_SIZE:
                break
            ftype  = buf[4]
            length = struct.unpack(">I", buf[5:9])[0]
            end    = _HEADER_SIZE + length
            if len(buf) < end:
                break
            payload = buf[_HEADER_SIZE:end]
            buf     = buf[end:]
            if ftype == _TYPE_IMU:
                self._parse_imu(payload)
            elif ftype == _TYPE_VIDEO:
                self._parse_video(payload)
        self._serial_buf = buf

    def _parse_imu(self, payload: bytes):
        try:
            parts = payload.decode().strip().split(",")
            if len(parts) < 6:
                return
            ts_ms = int(parts[2])
            roll  = -float(parts[3])
            pitch =  float(parts[4])
        except (ValueError, IndexError):
            return

        if pitch > 85:
            pitch -= 10
        if abs(roll) > 90:
            pitch = 180 - pitch
            while roll < -180:
                roll += 360

        self._roll  = roll
        self._pitch = pitch
        display_pitch = pitch - 90   # 0 = head level

        self._lbl_pitch.setText(f"{display_pitch:.1f}")
        self._lbl_roll.setText(f"{roll:.1f}")
        self._chart_append(ts_ms, roll, display_pitch)

        if not self._is_measuring:
            return

        if pitch < self._pitch_min:
            self._pitch_min = pitch
            if not self._pixmap.isNull():
                self.lbl_bot.setPixmap(self._pixmap)
        if pitch > self._pitch_max:
            self._pitch_max = pitch
            if not self._pixmap.isNull():
                self.lbl_top.setPixmap(self._pixmap)
        if roll < self._roll_min:
            self._roll_min = roll
            if not self._pixmap.isNull():
                self.lbl_left.setPixmap(self._pixmap)
        if roll > self._roll_max:
            self._roll_max = roll
            if not self._pixmap.isNull():
                self.lbl_right.setPixmap(self._pixmap)

        self._lbl_pitch_min.setText(f"{self._pitch_min - 90:.1f}")
        self._lbl_pitch_max.setText(f"{self._pitch_max - 90:.1f}")
        self._lbl_roll_min.setText(f"{self._roll_min:.1f}")
        self._lbl_roll_max.setText(f"{self._roll_max:.1f}")

    def _parse_video(self, payload: bytes):
        pix = QPixmap()
        if pix.loadFromData(payload):
            self._pixmap = pix
            self.lbl_center.setPixmap(pix)
            self._img_ready = True

    # ══════════════════════════════════════════════════════════════════════
    # Button state management
    # ══════════════════════════════════════════════════════════════════════
    def _update_buttons(self):
        if self._is_measuring:
            self._btn_start.setStyleSheet(_SS_GRAY);   self._btn_start.setEnabled(False)
            self._btn_stop.setStyleSheet(_SS_DANGER);  self._btn_stop.setEnabled(True)
            self._btn_reset.setStyleSheet(_SS_GRAY);   self._btn_reset.setEnabled(False)
        else:
            self._btn_start.setStyleSheet(_SS_ACTIVE); self._btn_start.setEnabled(True)
            self._btn_stop.setStyleSheet(_SS_GRAY);    self._btn_stop.setEnabled(False)
            self._btn_reset.setStyleSheet(_SS_NORMAL); self._btn_reset.setEnabled(True)

    # ══════════════════════════════════════════════════════════════════════
    # Button handlers
    # ══════════════════════════════════════════════════════════════════════
    def _start(self):
        self._is_measuring  = True
        self._is_recording  = True
        self._roll_min  =  1000.0;  self._roll_max  = -1000.0
        self._pitch_min =  1000.0;  self._pitch_max = -1000.0
        self._chart_reset()
        self._lbl_count.setText(str(self._report.get_count()))
        self._update_buttons()

    def _stop(self):
        self._is_measuring = False
        self._is_recording = False
        self._update_buttons()

    def _save(self):
        name     = self._edit_name.text()
        code     = self._edit_code.text()
        year     = self._edit_year.text()
        tilt     = f"{self._pitch_min - 90:.1f}/{self._pitch_max - 90:.1f}"
        pan      = f"{self._roll_min:.1f}/{self._roll_max:.1f}"
        base     = datetime.now().strftime("%d_%m_%Y_%H_%M") + f"_{name}"

        # Screenshot of chart
        if _HAS_CHART:
            chart_pix = self._chart_container.grab().scaled(700, 350)
            chart_pix.save(base + ".png", "PNG")

        # Screenshot of video frame
        video_pix = self._video_frame.grab().scaled(700, 350)
        video_pix.save(base + "video.png", "PNG")

        self._report.insert_record(name, code, year, tilt, pan)
        self._lbl_count.setText(str(self._report.get_count()))
        self._save_csv(base)

    def _reset(self):
        self._report.reset()
        self._update_buttons()

    def _open_pdf(self):
        if self._report.last_filename:
            QDesktopServices.openUrl(
                QUrl.fromLocalFile(
                    os.path.abspath(self._report.last_filename)))

    # ── CSV export ────────────────────────────────────────────────────────
    def _save_csv(self, base: str):
        if not _HAS_CHART or not self._roll_series:
            return
        filename = base + ".csv"
        n = min(self._roll_series.count(), self._pitch_series.count())
        with open(filename, "w", newline="", encoding="utf-8") as f:
            f.write("STT,Time(s),Roll(deg),Pitch(deg)\n")
            for i in range(n):
                rp = self._roll_series.at(i)
                pp = self._pitch_series.at(i)
                f.write(f"{i+1},{rp.x():.3f},{rp.y():.2f},{pp.y():.2f}\n")


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
