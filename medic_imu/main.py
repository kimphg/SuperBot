import sensor, time, network, pyb, socket, ujson, math, machine
from network import WLAN
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI

# ─── Tuning constants ─────────────────────────────────────────────────────────
HEARTBEAT_TIMEOUT_MS   = 3000
IMU_BUFFER_DURATION_MS = 5000
STA_CONNECT_TIMEOUT_MS = 10000  # give up on STA after 10 s
BEACON_INTERVAL_MS     = 2000   # discovery broadcast period

# ─── Persistent config (saved to flash) ──────────────────────────────────────
CONFIG_FILE = "wifi_config.json"

def load_config():
    try:
        with open(CONFIG_FILE) as f:
            return ujson.load(f)
    except:
        return {"ssid": "Maychi", "password": "maychi21"}

def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        ujson.dump(cfg, f)

# ─── Hardware init ────────────────────────────────────────────────────────────
lsmimu = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
ledpin = Pin("PF4", Pin.OUT_PP, Pin.PULL_UP)

def led_blink(on_ms=100, off_ms=100):
    ledpin.on(); time.sleep_ms(on_ms)
    ledpin.off(); time.sleep_ms(off_ms)

# ─── URL decode (for HTTP form body) ─────────────────────────────────────────
def url_decode(s):
    s = s.replace("+", " ")
    out, i = [], 0
    while i < len(s):
        if s[i] == "%" and i + 2 < len(s):
            out.append(chr(int(s[i+1:i+3], 16)))
            i += 3
        else:
            out.append(s[i])
            i += 1
    return "".join(out)

# ─── Web GUI templates ────────────────────────────────────────────────────────
_SETUP_HTML = """\
<!DOCTYPE html><html>
<head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nicla WiFi Setup</title>
<style>
body{{font-family:sans-serif;max-width:360px;margin:30px auto;padding:16px;background:#f0f4f8}}
h2{{color:#1a3a6a;margin-bottom:4px}}
.sub{{color:#555;font-size:13px;margin-bottom:20px}}
label{{display:block;margin-top:14px;font-weight:bold;color:#333;font-size:14px}}
input{{width:100%;padding:10px;margin-top:5px;box-sizing:border-box;
       border:1px solid #bbb;border-radius:5px;font-size:15px}}
.btn{{margin-top:22px;width:100%;padding:13px;background:#1a3a6a;
      color:#fff;border:none;border-radius:6px;font-size:16px;cursor:pointer}}
.btn:active{{background:#0e2244}}
.note{{color:#888;font-size:12px;margin-top:14px;text-align:center}}
</style></head>
<body>
<h2>&#128246; Nicla Vision</h2>
<p class="sub">WiFi Configuration</p>
<form method="POST" action="/save">
<label>WiFi Network (SSID):</label>
<input type="text" name="ssid" value="{ssid}" required autocomplete="off" placeholder="Network name">
<label>Password:</label>
<input type="password" name="password" value="" autocomplete="off" placeholder="Leave blank if open">
<button class="btn" type="submit">&#128190; Save &amp; Connect</button>
</form>
<p class="note">Device will restart and connect to the new network.<br>
(Setup AP password: <b>niclasetup</b>)</p>
</body></html>"""

_SAVED_HTML = """\
<!DOCTYPE html><html>
<head><meta charset="UTF-8"><title>Saved</title>
<style>body{{font-family:sans-serif;max-width:360px;margin:30px auto;padding:16px}}</style>
</head><body>
<h2 style="color:#1a6a1a">&#10003; Settings saved!</h2>
<p>Connecting to <b>{ssid}</b>&hellip;</p>
<p style="color:#555;font-size:13px">Reconnect your computer to that WiFi network,
then open the NiclaViewer app.</p>
</body></html>"""

# ─── Setup AP + HTTP server ───────────────────────────────────────────────────
def run_setup_ap(cfg):
    """Block here and serve config web page until user saves credentials."""
    ap = WLAN(network.AP_IF)
    ap.active(True)
    ap.config(ssid="NiclaSetup", key="niclasetup", channel=6, security=3)  # WPA2
    time.sleep_ms(600)
    ap_ip = ap.ifconfig()[0]
    print("Setup AP: join 'NiclaSetup' (password: niclasetup), open http://{}".format(ap_ip))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    srv.bind(("", 80))
    srv.listen(2)
    srv.settimeout(1)          # non-blocking accept — allows LED blinking

    saved = False
    while not saved:
        led_blink(40, 960)    # slow blink = setup mode

        try:
            conn, _ = srv.accept()
        except OSError:
            continue           # timeout, loop again

        try:
            conn.settimeout(3)
            raw = b""
            while b"\r\n\r\n" not in raw:
                chunk = conn.recv(512)
                if not chunk:
                    break
                raw += chunk
            req = raw.decode("utf-8", "ignore")

            body    = req.split("\r\n\r\n", 1)[1] if "\r\n\r\n" in req else ""
            is_post = req.startswith("POST /save")

            if is_post:
                params = {}
                for part in body.split("&"):
                    if "=" in part:
                        k, v = part.split("=", 1)
                        params[url_decode(k)] = url_decode(v)
                new_ssid = params.get("ssid", "").strip()
                new_pass = params.get("password", "")
                if new_ssid:
                    new_cfg = {"ssid": new_ssid, "password": new_pass}
                    save_config(new_cfg)
                    html = _SAVED_HTML.format(ssid=new_ssid)
                    saved = True

            if not saved:
                html = _SETUP_HTML.format(ssid=cfg.get("ssid", ""))

            hdr = ("HTTP/1.1 200 OK\r\n"
                   "Content-Type: text/html; charset=utf-8\r\n"
                   "Content-Length: {}\r\n"
                   "Connection: close\r\n\r\n").format(len(html))
            conn.sendall(hdr + html)
        except Exception as e:
            print("HTTP:", e)
        finally:
            conn.close()

    time.sleep_ms(1500)
    machine.reset()            # restart into STA mode with new credentials

# ─── WiFi: STA first, fall back to setup AP ──────────────────────────────────
cfg = load_config()

sta = WLAN(network.STA_IF)
sta.active(True)
sta.connect(cfg["ssid"], cfg["password"])
print("Connecting to WiFi '{}'…".format(cfg["ssid"]))

t0 = pyb.millis()
while not sta.isconnected():
    if pyb.elapsed_millis(t0) > STA_CONNECT_TIMEOUT_MS:
        break
    led_blink(150, 150)       # fast blink = trying to connect

if sta.isconnected():
    ledpin.off()
    ip_info = sta.ifconfig()
    print("WiFi connected — IP:", ip_info[0])
else:
    print("WiFi failed, starting setup AP")
    sta.active(False)
    run_setup_ap(cfg)         # blocks until saved + reset; never returns

# ─── Streaming sockets (STA mode) ─────────────────────────────────────────────
svideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sudp   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

shb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
shb.bind(("", 31002))
shb.setblocking(False)

# Broadcast socket used to announce presence until PC's IP is discovered
sbcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sbcast.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except Exception:
    pass

# Both resolved on first heartbeat (auto-discovered from PING sender)
addr      = None
addrvideo = None

q0, q1, q2, q3    = 1.0, 0.0, 0.0, 0.0
B_madgwick         = 0.02
last_heartbeat_ms  = 0
has_ever_connected = False
imu_buffer         = []   # list of (ts_ms: int, packet: bytes)
imu_seq            = 0
old_roll           = 0
last_beacon_ms     = 0

# ─── Madgwick 6-DOF filter ────────────────────────────────────────────────────
def invSqrt(x):
    return 1.0 / math.sqrt(x)

def Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq):
    gx *= 0.0174533; gy *= 0.0174533; gz *= 0.0174533
    global q0, q1, q2, q3
    qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
    qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy)
    qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx)
    qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx)
    if ax * ay * az != 0:
        rN = invSqrt(ax*ax + ay*ay + az*az)
        ax *= rN; ay *= rN; az *= rN
        _2q0=2*q0; _2q1=2*q1; _2q2=2*q2; _2q3=2*q3
        _4q0=4*q0; _4q1=4*q1; _4q2=4*q2
        _8q1=8*q1; _8q2=8*q2
        q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = (_4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay
              - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az)
        s2 = (4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
              - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az)
        s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay
        rN = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        s0*=rN; s1*=rN; s2*=rN; s3*=rN
        qDot1 -= B_madgwick*s0; qDot2 -= B_madgwick*s1
        qDot3 -= B_madgwick*s2; qDot4 -= B_madgwick*s3
    q0 += qDot1*invSampleFreq; q1 += qDot2*invSampleFreq
    q2 += qDot3*invSampleFreq; q3 += qDot4*invSampleFreq
    rN = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0*=rN; q1*=rN; q2*=rN; q3*=rN
    roll_IMU  = math.atan2(q0*q1+q2*q3, 0.5-q1*q1-q2*q2)  * 57.29577951
    pitch_IMU = -math.asin(-2*(q1*q3-q0*q2))               * 57.29577951
    yaw_IMU   = -math.atan2(q1*q2+q0*q3, 0.5-q2*q2-q3*q3) * 57.29577951
    return (roll_IMU, pitch_IMU, yaw_IMU)

# ─── Protocol helpers ─────────────────────────────────────────────────────────
def send_beacon():
    """Broadcast a small discovery packet so the Qt app can find our IP.
    Stops once the PC's address is known."""
    global last_beacon_ms
    if addr is not None:
        return
    now = pyb.millis()
    if now - last_beacon_ms >= BEACON_INTERVAL_MS:
        try:
            # HELLO on port 31000 — Qt's udpDataReceive captures sender IP,
            # discards the payload (< 6 comma fields), then sends heartbeats back.
            sbcast.sendto(b"HELLO\n", ("255.255.255.255", 31000))
        except OSError:
            pass
        last_beacon_ms = now

def check_heartbeat():
    """Non-blocking recv on the heartbeat port.
    First PING reveals the PC's IP; subsequent PINGs reset the timeout."""
    global last_heartbeat_ms, has_ever_connected, addr, addrvideo
    try:
        data, sender = shb.recvfrom(32)
        if data.startswith(b"PING"):
            last_heartbeat_ms = pyb.millis()
            if not has_ever_connected:
                client_ip = sender[0]
                addr      = socket.getaddrinfo(client_ip, 31000)[0][4]
                addrvideo = socket.getaddrinfo(client_ip, 31001)[0][4]
                print("PC discovered:", client_ip)
            has_ever_connected = True
    except OSError:
        pass  # EAGAIN — nothing to read

def is_connected():
    return has_ever_connected and (pyb.millis() - last_heartbeat_ms < HEARTBEAT_TIMEOUT_MS)

def flush_imu_buffer():
    """Replay buffered IMU packets in order; stop on send failure."""
    global imu_buffer
    if addr is None:
        return
    sent = 0
    for _, pkt in imu_buffer:
        try:
            sudp.sendto(pkt, addr)
            sent += 1
        except OSError:
            break
    imu_buffer = imu_buffer[sent:]

def updateIMU(fps):
    global imu_seq, old_roll, imu_buffer
    dtus = 1.0 / fps
    (rx, ry, rz) = lsmimu.gyro()
    (ax, ay, az) = lsmimu.accel()
    ax *= 9.8; ay *= 9.8; az *= 9.8
    (roll, pitch, yaw) = Madgwick6DOF(rx, ry, rz, ax, ay, az, dtus)
    if abs(roll) > 90:
        pitch = 180 - pitch
    if pitch > 50:
        roll = old_roll + rx * dtus
    old_roll = roll

    imu_seq += 1
    ts_ms  = pyb.millis()
    packet = bytes("IMU,{},{},{:.3f},{:.3f},{:.3f}\n".format(
                   imu_seq, ts_ms, roll, pitch, yaw), "utf-8")

    if is_connected():
        if imu_buffer:
            flush_imu_buffer()
        try:
            sudp.sendto(packet, addr)
        except OSError:
            imu_buffer.append((ts_ms, packet))
    elif has_ever_connected:
        imu_buffer.append((ts_ms, packet))
        cutoff = ts_ms - IMU_BUFFER_DURATION_MS
        while imu_buffer and imu_buffer[0][0] < cutoff:
            imu_buffer.pop(0)

# ─── Main streaming loop ──────────────────────────────────────────────────────
def start_streaming():
    clock = time.clock()
    while True:
        clock.tick()
        send_beacon()
        check_heartbeat()

        if is_connected():
            frame  = sensor.snapshot()
            cframe = frame.compress(quality=30)
            try:
                svideo.sendto(b"\x02\x03\x05\x07", addrvideo)
                k = 0
                while k < cframe.size():
                    k1 = k
                    k  = k + 1400
                    if k > cframe.size():
                        k = cframe.size()
                    svideo.sendto(bytes(cframe[k1:k]), addrvideo)
            except OSError:
                pass  # drop frame on error
        else:
            sensor.snapshot()  # keep sensor pipeline alive

        updateIMU(clock.fps())

while True:
    try:
        start_streaming()
    except OSError as e:
        print("socket error:", e)
