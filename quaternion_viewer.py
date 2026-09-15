#!/usr/bin/env python3
"""
Cortex quaternion viewer — live head orientation from an EMOTIV headset.

Connects to the EMOTIV Launcher's Cortex service (wss://localhost:6868), lets you
pick a headset, subscribes to its `mot` (motion) stream and draws it live: a head
that follows yours, the quaternion, the raw sensors, and nod / shake / wobble gestures.

    pip install -r requirements.txt
    python quaternion_viewer.py          # real headset: EMOTIV Launcher running and signed in
    python quaternion_viewer.py --demo   # no headset: synthetic motion

The Client ID / Client secret are the ones of the Cortex app you registered with
your EmotivID. Set CORTEX_CLIENT_ID and CORTEX_CLIENT_SECRET to prefill them.

See README.md for how it works, where data is saved and the measurements behind
the thresholds.
"""
from __future__ import annotations

import argparse
import collections
import itertools
import json
import math
import os
import platform
import queue
import random
import ssl
import subprocess
import sys
import threading
import time
import tkinter as tk
import zipfile

CORTEX_URL = "wss://localhost:6868"
APP_DIR = os.path.dirname(os.path.abspath(__file__))
APP_VERSION = "2026-09-14"
CALIBRATION_FILE = os.path.join(APP_DIR, "head_axes.json")
LOG_DIR = os.path.join(APP_DIR, "logs")

# Colour palette (dark purple card with peach accents).
BG = "#1b0b20"
CARD = "#2a1030"
FACE = "#321537"
BORDER = "#4a2a50"
TEXT = "#f1e6f0"
DIM = "#a38fa8"
FAINT = "#6f5a76"
ACCENT = "#f2a07b"
ACCENT_DIM = "#b9785f"
TRACK = "#45304b"
BADGE_BG = "#45356f"
BADGE_TEXT = "#d3c9ff"
ERROR = "#ff8a80"
OK = "#7fd8c8"
BUTTON = "#3a2140"

MONO = "Consolas"
SANS = "Segoe UI"

W = 780          # canvas width
ROW = 18         # bar row height
QUAT = ("Q0", "Q1", "Q2", "Q3")


# ── quaternion maths ─────────────────────────────────────────────────────────

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 1e-9 else (1.0, 0.0, 0.0, 0.0)


def nlerp(a, b, t):
    if sum(x * y for x, y in zip(a, b)) < 0:  # q and -q are the same rotation
        b = tuple(-c for c in b)
    return quat_normalize(tuple(x + (y - x) * t for x, y in zip(a, b)))


def euler_from_quat(q):
    """(roll, pitch, yaw) in radians: rotations about x, y and z (Z-Y-X order)."""
    w, x, y, z = quat_normalize(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def rotate_into(q, v):
    """A world vector expressed in the frame of orientation q."""
    return quat_mul(quat_mul(quat_conj(q), (0.0, *v)), q)[1:]


def dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def normalize3(v):
    n = math.sqrt(dot3(v, v))
    return tuple(c / n for c in v) if n > 1e-9 else (0.0, 0.0, 0.0)


def rotation_vector(q):
    """Axis × angle (radians) of a rotation quaternion."""
    w, x, y, z = quat_normalize(q)
    if w < 0:  # take the short way round
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(max(0.0, 1 - w * w))
    if s < 1e-6:
        return (0.0, 0.0, 0.0)
    angle = 2 * math.acos(min(1.0, w))
    return (x / s * angle, y / s * angle, z / s * angle)


# Cortex does not document the IMU's axis frame, and each headset holds its sensor
# differently. The MN8 even sits at an angle in the ear, so one head turn rotates about a
# MIX of sensor axes and no axis swap can fix it. Two layers (see README, "Head axes"):
#   1. HEAD_AXES: a per-family starting point, measured on a real head.
#   2. Calibrate: the wearer turns right and nods down; those measured rotation axes become
#      an exact sensor→head rotation for this wearer and this fit.
# A mapping is a 3×3 rotation whose ROWS are the head's roll, pitch and yaw axes written in
# sensor coordinates. Head convention: x = nose (roll), y = ear to ear (pitch), z = vertical
# (yaw), with +yaw = turn right and +pitch = nod down.
HEAD_AXES = {
    # 2026-09-14, INSIGHT2-A3D2002C calibrated on a head: turn = −X, nod = +Z, tilt = −Y, all
    # within ~11° of the sensor axes. (The first axis-swap table had turn and tilt backwards.)
    "INSIGHT": ((0.080, -0.984, 0.158), (0.088, 0.165, 0.982), (-0.993, -0.064, 0.100)),
    # 2026-09-14, MN8-A001 calibrated on a head (one wearer, one fit): the earbud's sensor is
    # rotated ~37° off the axes, so a turn is −0.80Y −0.57X. Fit varies — still Calibrate per wearer.
    "MN8": ((0.820, -0.564, 0.098), (0.027, 0.210, 0.977), (-0.572, -0.799, 0.187)),
    # 2026-09-14, EPOCX-E5020FF0: the band pivots at the ears, so its two positions put the
    # sensor ~82° apart about the ear-to-ear axis — turn and tilt trade places. One row set per fit.
    "EPOCX:horizontal": ((-0.242, -0.970, 0.034), (0.062, 0.019, 0.998), (-0.968, 0.244, 0.055)),
    "EPOCX:vertical": ((-0.992, 0.095, 0.078), (0.071, -0.079, 0.994), (0.101, 0.992, 0.072)),
}
HEAD_ORDER = ("roll", "pitch", "yaw")
FULLY_MEASURED = {"INSIGHT", "MN8", "EPOCX:horizontal", "EPOCX:vertical"}  # checked on a head
# headsets that can be worn more than one way; each way needs its own axes (first = default)
FITS = {"EPOCX": ("horizontal", "vertical")}


def headset_family(headset_id):
    family = headset_id.split("-")[0].upper()
    if family.startswith("INSIGHT"):
        return "INSIGHT"  # INSIGHT2 holds its sensor like INSIGHT
    if family.startswith(("EPOCFLEX", "FLEX")):
        return "FLEX"     # EPOC Flex IDs appear with either prefix
    return family


def axes_for(headset_id, fit=None):
    """(axes, measured?) from the built-in table — unmeasured families borrow Insight's axes."""
    key = setup_key(headset_family(headset_id), fit)
    return HEAD_AXES.get(key, HEAD_AXES["INSIGHT"]), key in FULLY_MEASURED


def setup_key(name, fit=None):
    """'EPOCX-E5020FF0' + 'vertical' → 'EPOCX-E5020FF0:vertical'; headsets with one fit keep their ID."""
    return f"{name}:{fit}" if fit else name


def to_head_frame(q, axes):
    """Sensor-frame rotation → the same rotation in head axes (conjugation by the mapping)."""
    v = q[1:]
    return (q[0], *(dot3(row, v) for row in axes))


def from_head_frame(h, axes):
    v = h[1:]
    return (h[0], *(sum(axes[r][c] * v[r] for r in range(3)) for c in range(3)))


def axes_from_moves(turn_right, nod_down):
    """Rotation vectors (sensor frame) of a right turn and a downward nod → head-axis rows.

    The turn fixes the yaw axis exactly; the nod keeps only its part perpendicular to
    that (people rarely nod perfectly straight), and roll completes a right-handed frame.
    """
    yaw = normalize3(turn_right)
    k = dot3(nod_down, yaw)
    pitch = normalize3(tuple(n - k * y for n, y in zip(nod_down, yaw)))
    return (cross3(pitch, yaw), pitch, yaw)


def describe_axis(row, smallest=0.05):
    """(-0.30, 0.93, 0.21) → '+0.93Y −0.30X +0.21Z'"""
    parts = sorted(zip(row, "XYZ"), key=lambda p: -abs(p[0]))
    return " ".join(f"{'+' if v >= 0 else '−'}{abs(v):.2f}{name}" for v, name in parts if abs(v) >= smallest)


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


# ── gestures ─────────────────────────────────────────────────────────────────

class GestureDetector:
    """Nod, headshake and head wobble from calibrated head angles, in real time.

    All three are an OSCILLATION about one head axis: wobble = roll (ear toward alternate
    shoulders), nod = pitch, shake = yaw. Over the last WINDOW seconds, for each axis:
    subtract the mean (so a held pose or slow drift is not motion), count swings — sign
    changes of what's left, with hysteresis so sensor noise can't flip it — and measure
    the size. A gesture needs enough swings, enough size, and its axis must clearly
    dominate the other two (a wobble drags some yaw along; a shake shouldn't read as one).
    Thresholds come from simulated gestures at 6.4/32/64 Hz (see README, "Gestures").
    """

    NAMES = ("wobble", "nod", "shake")  # same order as HEAD_ORDER: roll, pitch, yaw
    WINDOW = 1.5                         # seconds of history judged at once
    MIN_SWINGS = 3                       # sign changes: one dip-and-return makes only 2
    # peak-to-peak degrees per axis (wobble, nod, shake). Measured trade-off: 8/8/10 fired on
    # 100% of ±4° rhythmic bobs (talking, laughing); 10/10/12 fires on 0–20% of them and still
    # catches ±8° gestures 87–100% (±6°: 93–98% at 32/64 Hz, 63% on MN8's 6.4 Hz).
    MIN_SIZE = (10.0, 10.0, 12.0)
    DOMINANCE = 1.4                      # RMS of the gesture axis vs the next largest
    NOISE = 1.5                          # hysteresis floor, degrees
    RELEASE = 0.5                        # seconds without a match that end a gesture

    def __init__(self):
        self.counts = dict.fromkeys(self.NAMES, 0)
        self.reset()

    def reset(self):
        self.samples = collections.deque()
        self.unwrapped = None
        self.current = None
        self.last_match = None

    def update(self, t, angles):
        """Feed one sample (roll, pitch, yaw in radians). Returns the gesture in progress or None."""
        deg = [math.degrees(a) for a in angles]
        if self.unwrapped is None:
            self.unwrapped = deg
        else:  # keep yaw continuous across ±180°
            self.unwrapped = [u + (d - u + 180) % 360 - 180 for d, u in zip(deg, self.unwrapped)]
        self.samples.append((t, *self.unwrapped))
        while t - self.samples[0][0] > self.WINDOW:
            self.samples.popleft()

        match = self.classify()
        if match:
            if match != self.current:
                self.counts[match] += 1
                print(f"[gesture] {match}", flush=True)
            self.current, self.last_match = match, t
        elif self.current and t - self.last_match > self.RELEASE:
            self.current = None
        return self.current

    def classify(self):
        if len(self.samples) < 6 or self.samples[-1][0] - self.samples[0][0] < 0.8 * self.WINDOW:
            return None
        stats = []
        for axis in range(3):
            xs = [s[axis + 1] for s in self.samples]
            mean = sum(xs) / len(xs)
            size = max(xs) - min(xs)
            rms = math.sqrt(sum((x - mean) ** 2 for x in xs) / len(xs))
            band = max(self.NOISE, 0.15 * size)
            swings, side = 0, 0
            for x in xs:
                now = 1 if x - mean > band else -1 if x - mean < -band else 0
                if now:
                    swings += side not in (0, now)
                    side = now
            stats.append((swings, size, rms))
        axis = max(range(3), key=lambda a: stats[a][2])
        swings, size, rms = stats[axis]
        runner_up = max(stats[a][2] for a in range(3) if a != axis)
        if swings >= self.MIN_SWINGS and size >= self.MIN_SIZE[axis] and rms >= self.DOMINANCE * runner_up:
            return self.NAMES[axis]
        return None


GESTURE_LABELS = {"nod": "nod · yes", "shake": "shake · no", "wobble": "wobble"}


# ── Cortex ───────────────────────────────────────────────────────────────────

class CortexError(Exception):
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


FRIENDLY_ERRORS = {
    -32021: "Cortex rejected the Client ID / Client secret. Copy them again from your Cortex app.",
    -32102: "This app is not approved yet. Approve it in the EMOTIV Launcher, then Connect again.",
}


class Cortex:
    """Tiny Cortex JSON-RPC client.

    A reader thread routes replies (they carry an `id`) back to the waiting
    caller and hands stream samples and warnings to callbacks. `call` blocks,
    so never use it on the UI thread for long.
    """

    def __init__(self, on_sample, on_warning, on_closed):
        import websocket  # pip install websocket-client

        try:
            # Cortex serves a certificate signed by EMOTIV's own CA, not a public one.
            self._ws = websocket.create_connection(
                CORTEX_URL, timeout=5,
                sslopt={"cert_reqs": ssl.CERT_NONE, "check_hostname": False})
        except Exception as e:
            raise CortexError("Cannot reach Cortex on wss://localhost:6868. "
                              "Start the EMOTIV Launcher and sign in.") from e
        self._ws.settimeout(None)
        self._ids = itertools.count(1)
        self._pending = {}
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._on_sample, self._on_warning, self._on_closed = on_sample, on_warning, on_closed
        self.closed = False
        threading.Thread(target=self._read_loop, daemon=True).start()

    def call(self, method, timeout=15.0, **params):
        rid = next(self._ids)
        slot = {"done": threading.Event()}
        with self._lock:
            self._pending[rid] = slot
        try:
            with self._send_lock:
                self._ws.send(json.dumps({"jsonrpc": "2.0", "id": rid, "method": method, "params": params}))
        except Exception as e:
            with self._lock:
                self._pending.pop(rid, None)
            raise CortexError("The connection to Cortex is closed.") from e
        if not slot["done"].wait(timeout):
            with self._lock:
                self._pending.pop(rid, None)
            raise CortexError(f"Cortex did not answer {method} in time.")
        reply = slot.get("reply")
        if reply is None:
            raise CortexError("The connection to Cortex is closed.")
        if "error" in reply:
            err = reply["error"]
            code = err.get("code")
            raise CortexError(FRIENDLY_ERRORS.get(code, f"{method}: {err.get('message')} ({code})"), code)
        return reply.get("result")

    def close(self):
        self.closed = True
        try:
            self._ws.close()
        except Exception:
            pass

    def _read_loop(self):
        try:
            while True:
                raw = self._ws.recv()
                if not raw:
                    break
                msg = json.loads(raw)
                if "id" in msg:
                    with self._lock:
                        slot = self._pending.pop(msg["id"], None)
                    if slot:
                        slot["reply"] = msg
                        slot["done"].set()
                elif "warning" in msg:
                    self._on_warning(msg["warning"])
                elif "sid" in msg:
                    self._on_sample(msg)
        except Exception:
            pass
        finally:
            self.closed = True
            with self._lock:
                pending, self._pending = self._pending, {}
            for slot in pending.values():
                slot["done"].set()
            self._on_closed(self)


class DemoCortex:
    """Stands in for Cortex: one headset with a synthetic 32 Hz `mot` stream."""

    COLS = ["COUNTER_MEMS", "INTERPOLATED_MEMS", "Q0", "Q1", "Q2", "Q3",
            "ACCX", "ACCY", "ACCZ", "MAGX", "MAGY", "MAGZ"]

    def __init__(self, on_sample, on_warning, on_closed):
        self._on_sample = on_sample
        self._connected = False
        self._generation = 0
        self.closed = False

    def call(self, method, timeout=15.0, **params):
        time.sleep(0.05)
        if method == "getUserLogin":
            return [{"username": "demo"}]
        if method == "hasAccessRight":
            return {"accessGranted": True}
        if method == "authorize":
            return {"cortexToken": "demo"}
        if method == "controlDevice":
            self._connected = self._connected or params.get("command") == "connect"
            return {}
        if method == "queryHeadsets":
            status = "connected" if self._connected else "discovered"
            return [{"id": "INSIGHT-DEMO0001", "customName": "Demo Insight", "status": status},
                    {"id": "EPOCX-DEMO0002", "customName": "Demo EPOC X", "status": status}]
        if method == "createSession":
            return {"id": "demo-session"}
        if method == "subscribe":
            self._generation += 1
            threading.Thread(target=self._stream, args=(self._generation,), daemon=True).start()
            return {"success": [{"streamName": "mot", "cols": self.COLS, "sid": "demo-session"}], "failure": []}
        if method == "updateSession":
            self._generation += 1
        return {}

    def close(self):
        self.closed = True

    def _stream(self, generation):
        counter, t0 = 0, time.time()
        while generation == self._generation and not self.closed:
            t = time.time() - t0
            head = quat_from_euler(0.25 * math.sin(0.5 * t + 1), 0.3 * math.sin(1.1 * t), 0.7 * math.sin(0.6 * t))
            q = from_head_frame(head, HEAD_AXES["INSIGHT"])
            acc = rotate_into(q, (0.0, 0.0, 1.0))
            mag = rotate_into(q, (0.35, 0.0, -0.94))
            noisy = [c + random.gauss(0, 0.004) for c in (*acc, *mag)]
            self._on_sample({"sid": "demo-session", "time": time.time(), "mot": [counter, 0, *q, *noisy]})
            counter = (counter + 1) % 128
            time.sleep(1 / 32)


# ── session log ──────────────────────────────────────────────────────────────

class SessionLog:
    """Everything needed to analyse a headset remotely, one JSON object per line.

    Motion samples, headset metadata and app events only. Never the Client ID / secret, the
    Cortex token or the Launcher's user names. One file per app run, in logs/.
    """

    def __init__(self, demo=False):
        os.makedirs(LOG_DIR, exist_ok=True)
        name = time.strftime(("demo-" if demo else "") + "session-%Y%m%d-%H%M%S.jsonl")
        self.path = os.path.join(LOG_DIR, name)
        self._file = open(self.path, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self._last_flush = 0.0

    def write(self, kind, **fields):
        line = json.dumps({"type": kind, "t_local": round(time.time(), 4), **fields}, default=str)
        with self._lock:
            if self._file.closed:
                return
            self._file.write(line + "\n")
            now = time.time()
            if kind != "mot" or now - self._last_flush > 1.0:  # samples are batched, events land at once
                self._file.flush()
                self._last_flush = now

    def flush(self):
        with self._lock:
            if not self._file.closed:
                self._file.flush()

    def close(self):
        with self._lock:
            self._file.close()


# A labelled recording for testers: each step is logged as a marker, so detected gestures
# can later be scored against what the person was actually asked to do.
PROTOCOL = [
    ("ready", "Get ready — face the screen", 4),
    ("still", "Hold still, facing the screen", 10),
    ("nod", "Nod YES about 5 times, at a relaxed pace", 12),
    ("rest", "Rest, facing the screen", 5),
    ("shake", "Shake your head NO about 5 times, at a relaxed pace", 12),
    ("rest", "Rest, facing the screen", 5),
    ("wobble", "Wobble your head side to side (ear toward each shoulder) about 5 times", 12),
    ("rest", "Rest, facing the screen", 5),
    ("look", "Look around the room naturally — no gestures", 20),
    ("talk", "Talk out loud (read something, or count) — no gestures", 20),
]


# ── app ──────────────────────────────────────────────────────────────────────

def round_rect(c, x0, y0, x1, y1, r, **kw):
    r = min(r, (x1 - x0) / 2, (y1 - y0) / 2)
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return c.create_polygon(pts, smooth=True, **kw)


class Picker(tk.Menubutton):
    """Dark drop-down list built on a classic Menubutton.

    ttk.Combobox turns its field white once it shows a value on some Windows Tk builds,
    whatever its style says; a Menubutton always uses the colours it is given. It mirrors
    the few Combobox calls this app uses: config(values=, state=), current(), get(), set().
    """

    def __init__(self, parent, width, on_select):
        self._var = tk.StringVar(value="")
        self._values = []
        self._on_select = on_select
        super().__init__(parent, textvariable=self._var, width=width, anchor="w", relief="flat",
                         indicatoron=True, bg=BG, fg=TEXT, activebackground=BUTTON, activeforeground=TEXT,
                         disabledforeground=FAINT, highlightthickness=1, highlightbackground=BORDER,
                         font=(SANS, 10), padx=6, pady=3, cursor="hand2", state="disabled")
        self.menu = tk.Menu(self, tearoff=False, bg=CARD, fg=TEXT, activebackground=ACCENT_DIM,
                            activeforeground=TEXT, font=(SANS, 10), bd=0)
        self["menu"] = self.menu

    def configure(self, cnf=None, **kw):
        kw.update(cnf or {})
        values, state = kw.pop("values", None), kw.pop("state", None)
        if values is not None:
            self._values = list(values)
            self.menu.delete(0, "end")
            for i, value in enumerate(self._values):
                self.menu.add_command(label=value, command=lambda i=i: self._choose(i))
        if state is not None:
            enabled = state != "disabled"
            kw.update(state="normal" if enabled else "disabled", bg=CARD if enabled else BG)
        return super().configure(**kw) if kw else None

    config = configure

    def _choose(self, index):
        self._var.set(self._values[index])
        self._on_select()

    def current(self, index=None):
        if index is None:
            return self._values.index(self._var.get()) if self._var.get() in self._values else -1
        self._var.set(self._values[index])

    def get(self):
        return self._var.get()

    def set(self, value):
        self._var.set(value)


def headset_label(h):
    # the ID first: it is what tells two headsets of the same model apart
    label = f"{h['id']} · {h.get('status', '?')}"
    return f"{label} · {h['customName']}" if h.get("customName") else label


class App:
    def __init__(self, root, demo=False):
        self.root = root
        self.demo = demo
        self.events = queue.Queue()  # (kind, payload) from background threads → UI thread
        self.jobs = queue.Queue()    # Cortex work, run one job at a time on the worker thread

        self.cortex = None
        self.token = None
        self.session = None
        self.sid = None
        self.headsets = []
        self.device_name = "no headset"
        self.cols = []
        self.axes, self.axes_measured = HEAD_AXES["INSIGHT"], True
        self.axes_source = "default"
        self.headset_id = None
        self.fit = None  # band position, for headsets listed in FITS
        self.last_fits = {}  # headset ID → band position it was last worn in
        self.calibrations = {} if demo else self._load_calibrations()  # demo never touches the saved file
        self.streaming = False

        self.sample_lock = threading.Lock()
        self.latest = None                            # (seq, cortex_time, raw values)
        self.seq = 0
        self.arrivals = collections.deque(maxlen=512)  # wall-clock arrival times, for the Hz badge
        self.stream_started = None
        self.no_data_warned = False
        self.protocol_step = None   # index into PROTOCOL while the test runs
        self.protocol_step_end = 0.0
        self.protocol_hits = {}
        self.last_rate_log = 0.0
        self.log = SessionLog(demo)
        self.log.write("session", app_version=APP_VERSION, python=platform.python_version(),
                       os=platform.platform(), demo=demo)
        self._reset_motion()

        self._build_ui()
        threading.Thread(target=self._worker, daemon=True).start()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(33, self._tick)

    def _reset_motion(self):
        self.values = {}
        self.last_seq = 0
        self.neutral = None
        self.rel = None
        self.shown = None
        self.prev_euler = None
        self.prev_time = None
        self.axis_rate = [0.0, 0.0, 0.0]
        self.group_scale = {}
        self.activity = collections.defaultdict(float)
        self.prev_values = {}
        self.prev_sample_time = None
        self.hz = None
        self.calibration = None                         # None | "forward" | "right" | "down"
        self.recent_quats = collections.deque(maxlen=600)  # (cortex time, sensor quaternion)
        self.cal_neutral, self.cal_moves, self.cal_gravity = None, {}, None
        self.gestures = GestureDetector()  # counts start over per headset
        self.cal_returned = False

    # ── widgets ──

    def _build_ui(self):
        root = self.root
        root.title("Cortex quaternion viewer")
        root.configure(bg=BG)
        root.resizable(False, False)

        creds = tk.Frame(root, bg=BG)
        creds.pack(fill="x", padx=16, pady=(14, 0))
        self.client_id = tk.StringVar(value=os.environ.get("CORTEX_CLIENT_ID", "demo" if self.demo else ""))
        self.client_secret = tk.StringVar(value=os.environ.get("CORTEX_CLIENT_SECRET", "demo" if self.demo else ""))
        self._label(creds, "Client ID").pack(side="left")
        self._entry(creds, self.client_id).pack(side="left", padx=(6, 14))
        self._label(creds, "Client secret").pack(side="left")
        self._entry(creds, self.client_secret, secret=True).pack(side="left", padx=(6, 14))
        self.connect_btn = self._button(creds, "Connect", self._on_connect)
        self.connect_btn.pack(side="left")

        pick = tk.Frame(root, bg=BG)
        pick.pack(fill="x", padx=16, pady=(10, 0))
        self._label(pick, "Headset").pack(side="left")
        self.headset_box = Picker(pick, width=40, on_select=self._on_pick_headset)
        self.headset_box.pack(side="left", padx=(6, 10))
        self._label(pick, "Band").pack(side="left")
        self.fit_box = Picker(pick, width=10, on_select=self._on_pick_fit)
        self.fit_box.pack(side="left", padx=(6, 10))
        self.rescan_btn = self._button(pick, "Rescan", lambda: self._queue_scan())
        self.rescan_btn.pack(side="left", padx=(0, 8))
        self.rescan_btn.config(state="disabled")
        self.zero_btn = self._button(pick, "Zero pose", self._on_zero)
        self.zero_btn.pack(side="left", padx=(0, 8))
        self._button(pick, "Calibrate", self._on_calibrate).pack(side="left")

        tools = tk.Frame(root, bg=BG)
        tools.pack(fill="x", padx=16, pady=(10, 0))
        self.protocol_btn = self._button(tools, "Test protocol", self._on_protocol)
        self.protocol_btn.pack(side="left", padx=(0, 8))
        self._button(tools, "Export log", self._on_export).pack(side="left", padx=(0, 12))
        tk.Label(tools, text=f"logging to logs/{os.path.basename(self.log.path)}", bg=BG, fg=FAINT,
                 font=(MONO, 8)).pack(side="left")

        self.status = tk.Label(root, bg=BG, fg=DIM, font=(SANS, 9), anchor="w")
        self.status.pack(fill="x", padx=16, pady=(8, 0))
        self._set_status("Start the EMOTIV Launcher, then Connect." if not self.demo
                         else "Demo mode: press Connect, then pick the demo headset.")

        self.canvas = tk.Canvas(root, width=W, height=320, bg=BG, highlightthickness=0)
        self.canvas.pack(padx=8, pady=(6, 10))

    def _label(self, parent, text):
        return tk.Label(parent, text=text, bg=BG, fg=DIM, font=(SANS, 9))

    def _entry(self, parent, var, secret=False):
        return tk.Entry(parent, textvariable=var, width=24, show="•" if secret else "",
                        bg=CARD, fg=TEXT, insertbackground=TEXT, relief="flat", font=(MONO, 10),
                        highlightthickness=1, highlightbackground=BORDER, highlightcolor=ACCENT)

    def _button(self, parent, text, command):
        return tk.Button(parent, text=text, command=command, bg=BUTTON, fg=TEXT, relief="flat",
                         activebackground=ACCENT_DIM, activeforeground=TEXT, disabledforeground=FAINT,
                         font=(SANS, 9), padx=12, pady=2, cursor="hand2", bd=0)

    def _set_status(self, text, color=DIM):
        self.status.config(text=text, fg=color)

    def _post_status(self, text, color=DIM):
        self.events.put(("status", (text, color)))

    # ── UI actions ──

    def _on_connect(self):
        cid, secret = self.client_id.get().strip(), self.client_secret.get().strip()
        if not (cid and secret):
            self._set_status("Enter the Client ID and Client secret of your Cortex app.", ERROR)
            return
        self.connect_btn.config(state="disabled")
        self.jobs.put(lambda: self._connect(cid, secret))

    def _on_protocol(self):
        if self.protocol_step is not None:
            self.log.write("marker", label="protocol_cancelled")
            self.protocol_step = None
            self.protocol_btn.config(text="Test protocol")
            self._set_status("Test stopped.", ERROR)
            return
        if not self.streaming:
            self._set_status("Pick a headset first, then run the test.", ERROR)
            return
        if self.calibration:
            self._set_status("Finish the calibration first (follow the prompts), then run the test.", ERROR)
            return
        self.protocol_hits = {}
        self.log.write("marker", label="protocol_start", setup=self._setup_name(), axes_source=self.axes_source)
        self.protocol_btn.config(text="Stop test")
        self._protocol_enter(0)

    def _protocol_enter(self, index):
        if index >= len(PROTOCOL):
            self.protocol_step = None
            self.protocol_btn.config(text="Test protocol")
            summary = {label: dict(hits) for label, hits in self.protocol_hits.items()}
            self.log.write("marker", label="protocol_end", gestures_per_step=summary)
            print(f"[protocol] gestures detected per step: {summary}", flush=True)
            self._set_status("Test finished — thank you!  Now press Export log and send the zip file.", OK)
            return
        label, prompt, seconds = PROTOCOL[index]
        self.protocol_step, self.protocol_step_end = index, time.time() + seconds
        self.log.write("marker", label=f"step_start:{label}", prompt=prompt, seconds=seconds)

    def _protocol_tick(self):
        if self.protocol_step is None:
            return
        label, prompt, _ = PROTOCOL[self.protocol_step]
        left = self.protocol_step_end - time.time()
        if left <= 0:
            self.log.write("marker", label=f"step_end:{label}")
            self._protocol_enter(self.protocol_step + 1)
            return
        self._set_status(f"TEST {self.protocol_step + 1}/{len(PROTOCOL)}:  {prompt}   ({math.ceil(left)} s)", ACCENT)

    def _on_export(self):
        self.log.write("export")
        self.log.flush()
        stamp = time.strftime("%Y%m%d-%H%M%S")
        who = (self.headset_id or "no-headset").replace(":", "_")
        path = os.path.join(APP_DIR, f"cortex-motion-log-{who}-{stamp}.zip")
        try:
            with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
                z.write(self.log.path, os.path.join("logs", os.path.basename(self.log.path)))
                if os.path.exists(CALIBRATION_FILE):
                    z.write(CALIBRATION_FILE, "head_axes.json")
        except OSError as e:
            self._set_status(f"Could not write the log zip: {e}", ERROR)
            return
        self._set_status(f"Saved {os.path.basename(path)} in {APP_DIR} — send that file.", OK)
        try:  # show the file so it is easy to attach
            if sys.platform.startswith("win"):
                subprocess.Popen(["explorer", "/select,", path])
            elif sys.platform == "darwin":
                subprocess.Popen(["open", "-R", path])
            else:
                subprocess.Popen(["xdg-open", APP_DIR])
        except OSError:
            pass

    def _on_pick_fit(self, _event=None):
        fit = self.fit_box.get()
        if self.streaming and fit != self.fit:
            self.log.write("fit", headset=self.headset_id, fit=fit)
            self.fit = fit
            self.last_fits[self.headset_id] = fit  # so the next session starts in the same position
            self._write_calibrations()
            self._apply_setup()

    def _setup_name(self):
        return f"{self.headset_id} ({self.fit} band)" if self.fit else self.headset_id

    def _apply_setup(self, lead=""):
        """Load the axes for this headset + band position; calibrate first if none are saved."""
        self.axes, self.axes_source = self._axes_for(self.headset_id, self.fit)
        self.axes_measured = self.axes_source != "guess"
        self.log.write("setup", headset=self.headset_id, fit=self.fit, axes_source=self.axes_source,
                       axes=[list(row) for row in self.axes])
        self.neutral = self.shown = self.prev_euler = self.calibration = None
        self.gestures.reset()
        band_hint = "  Set Band above to how you wear it." if self.fit else ""
        if self.axes_source == "saved":
            when = self.calibrations[setup_key(self.headset_id, self.fit)].get("calibrated", "")
            self._set_status(f"{lead}Using your calibration for {self._setup_name()} from {when}.{band_hint}  "
                             "Press Calibrate to redo it.", OK)
        elif not any(k in self.cols for k in QUAT):
            self._set_status(f"{lead}Streaming motion from {self.headset_id}.", OK)
        else:
            # first time on this headset + fit: measure its axes before trusting the words
            self._calibration_advance("forward", lead=f"{lead}First use of {self._setup_name()}.{band_hint}  ")

    def _on_pick_headset(self, _event=None):
        i = self.headset_box.current()
        if 0 <= i < len(self.headsets):
            headset_id = self.headsets[i]["id"]
            self.jobs.put(lambda: self._start_stream(headset_id))

    def _queue_scan(self):
        self.rescan_btn.config(state="disabled")
        self.jobs.put(self._scan)

    # step → (prompt, word under the head, minimum angle in degrees to accept the hold)
    CALIBRATION_STEPS = {
        "forward": ("1/5 — face the screen and hold still…", "hold still", 0),
        "up": ("2/5 — look UP (tilt your head back) and hold…", "look up", 15),
        "down": ("3/5 — back to forward, then look DOWN (chin toward chest) and hold…", "look down", 15),
        "left": ("4/5 — back to forward, then turn your head LEFT and hold…", "turn left", 25),
        "right": ("5/5 — back to forward, then turn your head RIGHT and hold…", "turn right", 25),
    }
    CALIBRATION_ORDER = list(CALIBRATION_STEPS)

    def _on_calibrate(self):
        if not self.streaming or self.rel is None:
            self._set_status("Pick a headset that streams quaternions first, then Calibrate.", ERROR)
            return
        self._calibration_advance("forward")

    def _calibration_advance(self, step, lead=""):
        self.calibration = step
        # after "up", every move must start back at forward, so holds can't chain together
        self.cal_returned = step in ("forward", "up")
        self.recent_quats.clear()  # every step needs its own fresh hold
        self.log.write("calibration_step", setup=self._setup_name(), step=step)
        self._set_status(f"{lead}Calibrating head axes {self.CALIBRATION_STEPS[step][0]}", ACCENT)

    def _still_for(self, seconds, now, tolerance_deg=4.0):
        """Every pose in the last `seconds` is within tolerance of the newest one."""
        if not self.recent_quats or now - self.recent_quats[0][0] < seconds:
            return False
        newest = self.recent_quats[-1][1]
        limit = math.cos(math.radians(tolerance_deg) / 2)
        return all(abs(sum(a * b for a, b in zip(q, newest))) >= limit
                   for t, q in self.recent_quats if now - t <= seconds)

    def _calibration_sample(self, t, q):
        if not self.calibration:
            return
        self.recent_quats.append((t, q))
        if not self._still_for(1.0, t):
            return
        step = self.calibration
        if step == "forward":
            self.cal_neutral, self.cal_moves = q, {}
            # gravity in sensor coordinates while facing forward — enough saved fits of this
            # could later tell the band positions apart automatically
            acc = [self.values.get(k) for k in ("ACCX", "ACCY", "ACCZ")]
            self.cal_gravity = list(normalize3(acc)) if all(isinstance(v, (int, float)) for v in acc) else None
            self._calibration_advance("up")
            return
        move = rotation_vector(quat_mul(quat_conj(self.cal_neutral), q))
        angle = math.degrees(math.sqrt(dot3(move, move)))
        if not self.cal_returned:
            self.cal_returned = angle < 10  # back at forward: now the next move counts
            return
        if angle < self.CALIBRATION_STEPS[step][2]:
            return
        self.cal_moves[step] = move
        following = self.CALIBRATION_ORDER.index(step) + 1
        if following < len(self.CALIBRATION_ORDER):
            self._calibration_advance(self.CALIBRATION_ORDER[following])
        else:
            self._calibration_finish()

    def _calibration_finish(self):
        self.calibration = None
        m = self.cal_moves
        # opposite moves rotate about the same axis in opposite directions; their
        # difference points along +pitch (look down) and +yaw (turn right)
        nod = tuple(d - u for d, u in zip(m["down"], m["up"]))
        turn = tuple(r - l for r, l in zip(m["right"], m["left"]))
        if abs(dot3(normalize3(turn), normalize3(nod))) > 0.7:
            self.log.write("calibration_failed", setup=self._setup_name(), reason="turn and nod overlap",
                           moves={k: list(v) for k, v in m.items()})
            self._set_status("Up/down and left/right looked like the same movement. Press Calibrate and try again.", ERROR)
            return
        self.axes, self.axes_measured = axes_from_moves(turn, nod), True
        self.neutral, self.shown, self.prev_euler = self.cal_neutral, None, None
        self.gestures.reset()
        self._save_calibration(self.headset_id, self.fit, self.axes, self.cal_gravity)
        yaw_off = math.degrees(math.acos(min(1.0, max(abs(c) for c in self.axes[2]))))
        summary = "  ·  ".join(f"{name} = {describe_axis(row)}" for name, row in zip(("tilt", "nod", "turn"), self.axes))
        print(f"[calibration] {self._setup_name()}: {summary}  (turn axis {yaw_off:.0f}° off the nearest sensor axis)", flush=True)
        self.log.write("calibration", setup=self._setup_name(), headset=self.headset_id, fit=self.fit,
                       axes=[list(row) for row in self.axes], moves={k: list(v) for k, v in m.items()},
                       neutral=list(self.cal_neutral), gravity_forward=self.cal_gravity,
                       turn_axis_offset_deg=round(yaw_off, 1))
        self._set_status(f"Calibrated and saved for {self._setup_name()}.  {summary}  "
                         f"(turn axis {yaw_off:.0f}° off the nearest sensor axis)", OK)

    # ── saved calibrations: head_axes.json next to this script ──

    def _load_calibrations(self):
        try:
            with open(CALIBRATION_FILE, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return {}
        self.last_fits = data.get("last_fit", {})
        return data.get("headsets", {})

    def _write_calibrations(self):
        if self.demo:
            return
        try:
            with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
                json.dump({"headsets": self.calibrations, "last_fit": self.last_fits}, f, indent=2)
        except OSError as e:
            print(f"[calibration] could not save {CALIBRATION_FILE}: {e}", flush=True)

    def _save_calibration(self, headset_id, fit, axes, gravity=None):
        # the newest calibration is the one used; earlier ones are kept (newest first) so a
        # re-run never erases a measurement
        key = setup_key(headset_id, fit)
        previous = dict(self.calibrations.get(key) or {})
        history = previous.pop("history", [])
        if previous:
            history.insert(0, previous)
        entry = {
            "headset": headset_id,
            "family": headset_family(headset_id),
            "axes": {name: list(row) for name, row in zip(HEAD_ORDER, axes)},
            "calibrated": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if fit:
            entry["fit"] = fit
        if gravity:
            entry["gravity_forward"] = gravity
        entry["history"] = history[:20]
        self.calibrations[key] = entry
        self._write_calibrations()

    def _axes_for(self, headset_id, fit=None):
        """(axes, where they came from): this headset's calibration for this fit, else another
        headset of the same family worn the same way, else the built-in table."""
        family = headset_family(headset_id)
        saved = self.calibrations.get(setup_key(headset_id, fit))
        source = "saved"
        if not saved:
            same_family = [c for c in self.calibrations.values()
                           if c.get("family") == family and c.get("fit") == fit]
            saved = max(same_family, key=lambda c: c.get("calibrated", ""), default=None)
            source = "family"
        if saved:
            try:
                return tuple(tuple(float(v) for v in saved["axes"][name]) for name in HEAD_ORDER), source
            except (KeyError, TypeError, ValueError):
                pass
        axes, measured = axes_for(headset_id, fit)
        return axes, "default" if measured else "guess"

    def _on_zero(self):
        self.neutral = None  # the next frame becomes "forward"
        self.prev_euler = None
        self.gestures.reset()
        self.log.write("zero_pose")

    def _on_close(self):
        cx, self.cortex = self.cortex, None
        if cx:
            if self.session:
                try:
                    cx.call("updateSession", timeout=2, cortexToken=self.token, session=self.session, status="close")
                except Exception:
                    pass
            cx.close()
        self.log.write("end")
        self.log.close()
        self.root.destroy()

    # ── worker thread: everything that talks to Cortex ──

    def _worker(self):
        while True:
            job = self.jobs.get()
            try:
                job()
            except CortexError as e:
                self.events.put(("error", str(e)))
            except Exception as e:  # keep the worker alive whatever happens
                self.events.put(("error", f"Unexpected error: {e}"))

    def _connect(self, cid, secret):
        self._teardown()
        self._post_status("Connecting to Cortex…")
        cx = None
        try:
            cx = (DemoCortex if self.demo else Cortex)(self._on_sample, self._on_warning, self._on_socket_closed)
            logins = cx.call("getUserLogin") or []
            if not any(login.get("username") for login in logins):
                raise CortexError("Nobody is signed in to the EMOTIV Launcher. Sign in with your EmotivID, then Connect again.")
            if not cx.call("hasAccessRight", clientId=cid, clientSecret=secret).get("accessGranted"):
                cx.call("requestAccess", clientId=cid, clientSecret=secret)
                self._post_status("Approve this app in the EMOTIV Launcher (asked once)…", ACCENT)
                deadline = time.time() + 120
                while not cx.call("hasAccessRight", clientId=cid, clientSecret=secret).get("accessGranted"):
                    if time.time() > deadline:
                        raise CortexError("The app was not approved in the EMOTIV Launcher. Approve it, then Connect again.")
                    time.sleep(2)
            token = cx.call("authorize", clientId=cid, clientSecret=secret)["cortexToken"]
        except CortexError as e:
            if cx:
                cx.close()
            self.events.put(("connect_failed", str(e)))
            return
        self.cortex, self.token = cx, token
        try:
            info = cx.call("getCortexInfo") or {}
            self.log.write("cortex_info", **{k: info.get(k) for k in ("version", "buildNumber", "buildDate")})
        except CortexError:
            pass
        self.events.put(("connected", None))
        self._scan()

    def _scan(self):
        cx = self.cortex
        if not cx:
            return
        self._post_status("Looking for headsets… (a Bluetooth scan takes up to ~20 s)")
        try:
            cx.call("controlDevice", command="refresh")
        except CortexError:
            pass
        seen = None
        for attempt in range(14):
            if not self.jobs.empty():  # the user picked a headset — do that first
                break
            headsets = [h for h in (cx.call("queryHeadsets") or []) if h.get("id")]
            key = [(h["id"], h.get("status")) for h in headsets]
            if key != seen:
                seen = key
                self.log.write("headsets", list=headsets)  # firmware, motionSensors, settings…
                self.events.put(("headsets", headsets))
            if attempt < 13:
                time.sleep(1.5)
        self.events.put(("scan_done", None))

    def _find_headset(self, headset_id):
        return next((h for h in (self.cortex.call("queryHeadsets") or []) if h.get("id") == headset_id), None)

    def _start_stream(self, headset_id):
        if not self.cortex:
            return
        self._close_session()
        self.events.put(("stream_reset", headset_id))
        headset = self._find_headset(headset_id)
        if not headset:
            raise CortexError(f"{headset_id} is no longer available. Press Rescan.")
        if headset.get("status") != "connected":
            self._post_status(f"Connecting {headset_id}…")
            if headset.get("status") == "discovered":
                try:
                    self.cortex.call("controlDevice", command="connect", headset=headset_id)
                except CortexError:
                    pass
            for _ in range(20):
                time.sleep(1)
                headset = self._find_headset(headset_id)
                if headset and headset.get("status") == "connected":
                    break
            else:
                raise CortexError(f"Could not connect {headset_id}. Check it in the EMOTIV Launcher and pick it again.")

        self._post_status(f"Opening a session on {headset_id}…")
        self.session = self.cortex.call("createSession", cortexToken=self.token, headset=headset_id, status="open")["id"]
        result = self.cortex.call("subscribe", cortexToken=self.token, session=self.session, streams=["mot"])
        mot = next((s for s in result.get("success", []) if s.get("streamName") == "mot"), None)
        self.log.write("subscribe", headset=headset, success=result.get("success"), failure=result.get("failure"))
        if not mot:
            failure = (result.get("failure") or [{}])[0]
            raise CortexError(f"Cortex refused the motion stream: {failure.get('message', 'no reason given')}")
        self.headset_id = headset_id  # axes are chosen on the UI thread, with the band position
        self.cols = mot["cols"]
        self.sid = mot.get("sid")
        self.events.put(("streaming", headset))

    def _close_session(self):
        session, self.session, self.sid = self.session, None, None
        if self.cortex and session:
            try:
                self.cortex.call("updateSession", timeout=5, cortexToken=self.token, session=session, status="close")
            except CortexError:
                pass

    def _teardown(self):
        cx = self.cortex
        if not cx:
            return
        self._close_session()
        self.cortex, self.token = None, None
        cx.close()
        self.events.put(("reset", None))

    # ── reader thread callbacks ──

    def _on_sample(self, msg):
        if "mot" not in msg or msg.get("sid") != self.sid:
            return
        with self.sample_lock:
            self.seq += 1
            self.latest = (self.seq, msg.get("time", time.time()), msg["mot"])
            self.arrivals.append(time.time())
        self.log.write("mot", t=msg.get("time"), v=msg["mot"])

    def _on_warning(self, warning):
        self.log.write("warning", code=warning.get("code"), message=warning.get("message"))
        code = warning.get("code")
        if not self.sid:  # warnings about a session we closed ourselves
            return
        if code == 103:
            self.events.put(("lost", "The headset disconnected. Reconnect it in the EMOTIV Launcher, then pick it again."))
        elif code in (0, 1):
            self.events.put(("lost", "Cortex closed the session. Pick the headset again."))

    def _on_socket_closed(self, cx):
        self.events.put(("socket_closed", cx))

    # ── UI thread: events, motion, drawing ──

    def _tick(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            self._handle(kind, payload)
        self._update_motion()
        self._watch_stream()
        self._protocol_tick()
        self._draw()
        self.root.after(33, self._tick)

    def _watch_stream(self):
        now = time.time()
        if not self.streaming:
            return
        if self.seq == 0 and not self.no_data_warned and now - self.stream_started > 5:
            # Cortex accepts the subscription even when motion is switched off on the headset
            self.no_data_warned = True
            self.log.write("no_data", headset=self.headset_id, seconds=5,
                           settings=next((h.get("settings") for h in self.headsets if h["id"] == self.headset_id), None))
            self._set_status(f"No motion samples from {self.headset_id} after 5 s. Motion may be switched off in "
                             "this headset's own configuration (EPOC X / EPOC Flex can disable it). "
                             "Press Export log and send it anyway — the headset settings are in it.", ERROR)
        if now - self.last_rate_log > 5:
            self.last_rate_log = now
            self.log.write("rate", headset=self.headset_id, hz=round(self.hz, 2) if self.hz else None, samples=self.seq)

    def _handle(self, kind, payload):
        if kind in ("error", "connect_failed", "lost"):
            self.log.write(kind, message=payload)
        if kind == "status":
            self._set_status(*payload)
        elif kind == "error":
            self._set_status(payload, ERROR)
            self.rescan_btn.config(state="normal" if self.cortex else "disabled")
        elif kind == "connect_failed":
            self._set_status(payload, ERROR)
            self.connect_btn.config(state="normal")
        elif kind == "connected":
            self.connect_btn.config(state="normal", text="Reconnect")
            self.headset_box.config(state="readonly")
        elif kind == "headsets":
            self.headsets = payload
            self.headset_box.config(values=[headset_label(h) for h in payload])
            if not self.streaming:
                self._set_status("Pick your headset above." if payload else "Looking for headsets…")
        elif kind == "scan_done":
            self.rescan_btn.config(state="normal")
            if not self.headsets:
                self._set_status("No headset found. Turn it on, check it appears in the EMOTIV Launcher, "
                                 "then press Rescan.", ERROR)
        elif kind == "stream_reset":
            self.streaming = False
            with self.sample_lock:
                self.latest, self.seq = None, 0
                self.arrivals.clear()
            self._reset_motion()
            self.device_name = payload
            if self.protocol_step is not None:  # a test can't continue on another headset
                self._on_protocol()
        elif kind == "streaming":
            self.streaming = True
            h = payload
            # the picker still shows the status from the scan ("discovered"); refresh it
            self.headsets = [h if x["id"] == h["id"] else x for x in self.headsets]
            if not any(x["id"] == h["id"] for x in self.headsets):  # e.g. a rescan dropped it meanwhile
                self.headsets.append(h)
            self.headset_box.config(values=[headset_label(x) for x in self.headsets])
            self.headset_box.current(next(i for i, x in enumerate(self.headsets) if x["id"] == h["id"]))
            self.device_name = h.get("customName") or h["id"]
            if h.get("isVirtual"):
                self.device_name += " (virtual)"
            fits = FITS.get(headset_family(h["id"]))
            if fits:
                remembered = self.last_fits.get(h["id"])
                self.fit = remembered if remembered in fits else fits[0]
                self.fit_box.config(values=fits, state="readonly")
                self.fit_box.set(self.fit)
            else:
                self.fit = None
                self.fit_box.set("")
                self.fit_box.config(values=[], state="disabled")
            self.stream_started, self.no_data_warned = time.time(), False
            self._apply_setup(lead=f"Streaming {h['id']}.  ")
            self.rescan_btn.config(state="normal")
        elif kind == "lost":
            self.streaming = False
            self._set_status(payload, ERROR)
        elif kind == "reset":
            self.streaming = False
            self.headsets, self.cols, self.device_name = [], [], "no headset"
            self.headset_box.set("")
            self.headset_box.config(values=[], state="disabled")
            self.fit_box.set("")
            self.fit_box.config(values=[], state="disabled")
            self.rescan_btn.config(state="disabled")
        elif kind == "socket_closed" and payload is self.cortex:
            self.cortex, self.session, self.sid = None, None, None
            self._handle("reset", None)
            self._set_status("Cortex closed the connection. Make sure the EMOTIV Launcher is running, then Connect.", ERROR)
            self.connect_btn.config(state="normal", text="Connect")

    def _update_motion(self):
        with self.sample_lock:
            latest = self.latest
            arrivals = list(self.arrivals)

        now = time.time()
        recent = [t for t in arrivals if now - t < 3]
        self.hz = (len(recent) - 1) / (recent[-1] - recent[0]) \
            if len(recent) >= 3 and recent[-1] - recent[0] > 0.3 and now - recent[-1] < 1.5 else None

        if not latest or not self.cols or len(latest[2]) != len(self.cols):
            return
        seq, t, raw = latest
        self.values = dict(zip(self.cols, raw))
        is_new = seq != self.last_seq
        self.last_seq = seq

        q = [self.values.get(k) for k in QUAT]
        if all(isinstance(c, (int, float)) for c in q) and math.sqrt(sum(c * c for c in q)) > 0.2:
            q = quat_normalize(q)
            if is_new and self.calibration:
                self._calibration_sample(t, q)
            if self.neutral is None:
                self.neutral = q
                self.shown = None
            self.rel = to_head_frame(quat_normalize(quat_mul(quat_conj(self.neutral), q)), self.axes)
            # Ease toward the newest pose so slow streams (MN8 sends 6.4 Hz) don't jump.
            self.shown = self.rel if self.shown is None else nlerp(self.shown, self.rel, 0.35)

            if is_new:
                euler = euler_from_quat(self.rel)
                if self.prev_euler is not None and t > self.prev_time:
                    dt = t - self.prev_time
                    for i in range(3):
                        rate = abs(angle_diff(euler[i], self.prev_euler[i])) / dt
                        self.axis_rate[i] = 0.7 * self.axis_rate[i] + 0.3 * rate
                self.prev_euler, self.prev_time = euler, t
                if not self.calibration:  # calibration's up/down/left/right would read as gestures
                    before = self.gestures.current
                    gesture = self.gestures.update(t, euler)
                    if gesture and gesture != before:
                        self.log.write("gesture", name=gesture, t=t)
                        if self.protocol_step is not None:
                            step = PROTOCOL[self.protocol_step][0]
                            hits = self.protocol_hits.setdefault(step, collections.Counter())
                            hits[gesture] += 1

        if is_new:
            dt = t - self.prev_sample_time if self.prev_sample_time and t > self.prev_sample_time else None
            self.prev_sample_time = t
            for name, value in self.values.items():
                if not isinstance(value, (int, float)):
                    continue
                scale = self._scale_for(name)
                prev = self.prev_values.get(name, value)
                self.prev_values[name] = value
                if dt:
                    # share of full scale per second, so the highlight means the same at 6.4 Hz and 64 Hz
                    speed = abs(value - prev) / scale / dt
                    self.activity[name] = 0.8 * self.activity[name] + 0.2 * speed

    def _group(self, name):
        return name[:-1] if name[-1] in "XYZ" else name

    def _scale_for(self, name):
        if name in QUAT:
            return 1.0
        group = self._group(name)
        biggest = max((abs(v) for k, v in self.values.items()
                       if isinstance(v, (int, float)) and k.startswith(group) and k[-1] in "XYZ"), default=0.0)
        self.group_scale[group] = max(self.group_scale.get(group, 0.0) * 0.999, biggest, 1e-9)
        return self.group_scale[group]

    def _bar_names(self):
        if not self.cols:
            return list(QUAT) + ["ACCX", "ACCY", "ACCZ", "MAGX", "MAGY", "MAGZ"]
        order = [c for c in self.cols if c in QUAT]
        for group in ("GYRO", "ACC", "MAG"):
            order += [c for c in self.cols if c.startswith(group)]
        return order

    def _motion_word(self):
        if not self.streaming:
            return "waiting"
        if self.hz is None:
            return "no data"
        if self.rel is None:
            return "no quaternion"
        if self.calibration:
            return self.CALIBRATION_STEPS[self.calibration][1] if self.cal_returned else "face forward"
        if self.gestures.current:
            return GESTURE_LABELS[self.gestures.current]
        fastest = self._fastest_axis()
        if fastest is None:
            return "still"
        return ("tilting", "nodding", "turning")[fastest]

    def _fastest_axis(self):
        """Index into HEAD_ORDER of the axis moving fastest, or None when the head is still."""
        fastest = max(range(3), key=lambda i: self.axis_rate[i])
        return fastest if self.axis_rate[fastest] >= math.radians(12) else None

    def _draw(self):
        c = self.canvas
        c.delete("all")

        names = self._bar_names()
        top = 104
        rows, y = [], top
        for i, name in enumerate(names):
            if i and name not in QUAT and names[i - 1] in QUAT:
                y += 10  # gap between the quaternion and the raw sensors
            rows.append((name, y))
            y += ROW
        bottom = y
        height = max(370, bottom + 114)
        if int(c["height"]) != height:
            c.config(height=height)

        round_rect(c, 6, 4, W - 6, height - 4, 16, fill=CARD, outline=BORDER)

        # header: stream · device · rate badge
        x = 28
        item = c.create_text(x, 36, text="mot", anchor="w", fill=TEXT, font=(MONO, 13, "bold"))
        item = c.create_text(c.bbox(item)[2] + 10, 36, text="motion", anchor="w", fill=DIM, font=(SANS, 11))
        item = c.create_text(c.bbox(item)[2] + 10, 36, text="·", anchor="w", fill=FAINT, font=(SANS, 11))
        c.create_text(c.bbox(item)[2] + 10, 36, text=self.device_name, anchor="w",
                      fill=TEXT if self.streaming else FAINT, font=(SANS, 11, "bold"))
        badge = "— Hz" if self.hz is None else (f"{self.hz:.1f} Hz" if self.hz < 10 else f"{self.hz:.0f} Hz")
        item = c.create_text(W - 32, 36, text=badge, anchor="e", fill=BADGE_TEXT, font=(SANS, 9, "bold"))
        bx0, by0, bx1, by1 = c.bbox(item)
        c.tag_lower(round_rect(c, bx0 - 10, by0 - 3, bx1 + 10, by1 + 3, 10, fill=BADGE_BG, outline=""), item)
        c.create_text(28, 68, anchor="w", fill=DIM, font=(SANS, 10),
                      text="Head orientation from the built-in motion sensor — live quaternions, no mental-command training needed.")

        # head
        hx, hy = 112, top + 64
        self._draw_head(hx, hy, 44)
        roll, pitch, yaw = euler_from_quat(self.shown) if self.shown else (0.0, 0.0, 0.0)
        gesture = self.gestures.current if self.streaming and not self.calibration else None
        c.create_text(hx, hy + 88, text=self._motion_word(), fill=OK if gesture else ACCENT,
                      font=(SANS, 14 if gesture else 11, "bold"))
        deg = lambda a: int(round(math.degrees(a)))
        c.create_text(hx, hy + 108, fill=DIM, font=(MONO, 9),
                      text=f"pitch {deg(pitch)}° · yaw {deg(yaw)}° · roll {deg(roll)}°")
        # which raw sensor axis the movement is about — what you need to measure a new headset
        fastest = self._fastest_axis() if self.streaming and self.rel is not None and self.hz else None
        axis = "—" if fastest is None else describe_axis(self.axes[fastest], smallest=0.3)
        c.create_text(hx, hy + 126, fill=FAINT, font=(MONO, 9), text=f"sensor axis {axis}")

        # bars
        lx, tx0, tx1 = 232, 286, W - 32
        mid = (tx0 + tx1) / 2
        for name, y in rows:
            value = self.values.get(name)
            active = self.streaming and self.activity.get(name, 0.0) > 0.12
            if active:
                c.create_rectangle(lx - 12, y - 7, lx - 10, y + 7, fill=ACCENT, width=0)
            c.create_text(lx, y, text=name, anchor="w", fill=TEXT if active else DIM,
                          font=(MONO, 9, "bold" if active else "normal"))
            round_rect(c, tx0, y - 4, tx1, y + 4, 4, fill=TRACK, outline="")
            c.create_line(mid, y - 6, mid, y + 6, fill=FAINT)
            if isinstance(value, (int, float)):
                scale = 1.0 if name in QUAT else self.group_scale.get(self._group(name), 1.0)
                frac = max(-1.0, min(1.0, value / scale))
                end = mid + frac * (tx1 - mid)
                color = ACCENT if name in QUAT or active else ACCENT_DIM
                c.create_rectangle(min(mid, end), y - 4, max(mid, end), y + 4, fill=color, width=0)

        if any(k in self.values for k in QUAT):
            text = "  ·  ".join(f"{k} {self.values[k]:.2f}" for k in QUAT)
            color = TEXT
        elif self.cols:
            text, color = "This headset sends no quaternion (Q0–Q3) — only the raw sensors above.", ERROR
        else:
            text, color = "  ·  ".join(f"{k} —" for k in QUAT), FAINT
        c.create_text(lx, bottom + 12, text=text, anchor="w", fill=color, font=(MONO, 11, "bold"))

        # gestures: one chip per kind with its count; the one happening now lights up
        gy = bottom + 46
        item = c.create_text(lx, gy, text="gestures", anchor="w", fill=FAINT, font=(SANS, 9))
        x = c.bbox(item)[2] + 14
        for name in ("nod", "shake", "wobble"):
            active = name == gesture
            item = c.create_text(x + 12, gy, text=f"{GESTURE_LABELS[name]}  ×{self.gestures.counts[name]}", anchor="w",
                                 fill=CARD if active else DIM, font=(SANS, 10, "bold" if active else "normal"))
            x0, y0, x1, y1 = c.bbox(item)
            chip = round_rect(c, x0 - 12, y0 - 4, x1 + 12, y1 + 4, 12,
                              fill=OK if active else CARD, outline=OK if active else BORDER)
            c.tag_lower(chip, item)
            x = x1 + 24

        cols = " · ".join(self.cols) if self.cols else "COUNTER_MEMS · INTERPOLATED_MEMS · Q0-Q3 · ACCX/Y/Z · MAGX/Y/Z"
        c.create_text(28, height - 38, text=cols, anchor="w", fill=FAINT, font=(MONO, 8))
        c.create_text(28, height - 20, anchor="w", fill=FAINT, font=(SANS, 8),
                      text="Quaternion bars span −1…1.  ACC / MAG / GYRO have no documented unit, so each group is scaled to its largest value.")

    def _draw_head(self, cx, cy, s):
        c = self.canvas
        roll, pitch, yaw = euler_from_quat(self.shown) if self.shown else (0.0, 0.0, 0.0)
        cr, sr, cp, sp, cyw, syw = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        stretch = 1.15  # the head is a slightly tall ellipsoid

        def rotate(p):
            x, y, z = p
            x, y = x * cr - y * sr, x * sr + y * cr    # roll: about the nose axis
            y, z = y * cp - z * sp, y * sp + z * cp    # pitch: about the ear-to-ear axis
            x, z = x * cyw + z * syw, -x * syw + z * cyw  # yaw: about the vertical axis
            return x, y, z

        def surface(x, y):
            return x, y * stretch, math.sqrt(max(0.0, 1 - x * x - y * y))

        def screen(p):
            return cx + p[0] * s, cy - p[1] * s

        def polyline(points, **kw):
            pts = [rotate(p) for p in points]
            for a, b in zip(pts, pts[1:]):
                if a[2] > 0.05 and b[2] > 0.05:
                    c.create_line(*screen(a), *screen(b), **kw)

        # neck
        for side in (-1, 1):
            c.create_line(cx + side * 0.34 * s, cy + 0.95 * s, cx + side * 0.40 * s, cy + 1.35 * s,
                          fill=ACCENT_DIM, width=2)

        ears = [rotate((side, 0.05 * stretch, -0.05)) for side in (-1, 1)]

        def ear(p):
            ex, ey = screen(p)
            c.create_oval(ex - 0.15 * s, ey - 0.26 * s, ex + 0.15 * s, ey + 0.26 * s,
                          fill=FACE, outline=ACCENT, width=2)

        for p in ears:
            if p[2] <= 0.25:
                ear(p)

        outline = []
        for k in range(48):
            a = 2 * math.pi * k / 48
            x, y = math.cos(a), math.sin(a) * stretch
            outline += screen((x * cr - y * sr, x * sr + y * cr))
        c.create_polygon(outline, smooth=True, fill=FACE, outline=ACCENT, width=2)

        for p in ears:
            if p[2] > 0.25:
                ear(p)

        for side in (-1, 1):
            eye = rotate(surface(side * 0.34, 0.14))
            if eye[2] > 0.05:
                ex, ey = screen(eye)
                c.create_oval(ex - 3, ey - 3, ex + 3, ey + 3, fill=ACCENT, outline="")
            polyline([surface(side * 0.46, 0.34), surface(side * 0.22, 0.38)], fill=ACCENT_DIM, width=2)

        polyline([surface(0, 0.08), (0.0, -0.14 * stretch, 1.07), surface(0.09, -0.17)], fill=ACCENT, width=2)
        polyline([surface(x, -0.42 - 0.08 * (1 - (x / 0.28) ** 2)) for x in [i * 0.07 - 0.28 for i in range(9)]],
                 fill=ACCENT, width=2)


def main():
    parser = argparse.ArgumentParser(description="Live quaternion viewer for EMOTIV Cortex.")
    parser.add_argument("--demo", action="store_true", help="use a synthetic headset instead of Cortex")
    args = parser.parse_args()
    if not args.demo:
        try:
            import websocket  # noqa: F401
        except ImportError:
            raise SystemExit("Missing dependency: pip install -r requirements.txt")
    root = tk.Tk()
    App(root, demo=args.demo)
    root.mainloop()


if __name__ == "__main__":
    main()
