#!/usr/bin/env python3
"""
Manual control API, running on this PC. Does NOT talk MAVLink directly --
sends a small JSON command protocol over the RFD900x radio's serial port to
the Pi, which decodes it and drives the Cube locally over its own USB link
(see rover_control.py / radio_bridge.py on the Pi).

The UI is the React app in ../frontend (run separately via `npm run dev`
during development; once built to ../frontend/dist, the `/` route below
serves it statically -- for a packaged/exe build).

Also runs Shadow Mode: a second, independent MAVLink connection (see
leader_link.py) to a LEADER vehicle's Cube (Orange/+, driven by a person),
used purely as a GPS/heading/speed source. While enabled, periodically
computes a follow point some distance directly behind the leader's heading
(see geo_utils.py) and sends it to the follower over the SAME "goto"
message the manual waypoint-click feature uses -- Shadow Mode is not a
different navigation mechanism, just a repeated goto() aimed at a
continuously-updated point. See README.md "Shadow Mode" section.

Protocol (newline-delimited JSON):
  PC -> Pi:  {"type":"cmd","steer":-1..1,"throttle":-1..1,"arm":bool}
  PC -> Pi:  {"type":"goto","lat":...,"lon":...,"speed":<m/s, optional>}
  PC -> Pi:  {"type":"stop_nav"}
  Pi -> PC:  {"type":"status", ...RoverController.get_status() fields...}

Run: python3 server.py [http_port] [radio_port] [baud]
"""
import json
import sys
import threading
import time
from pathlib import Path

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

from geo_utils import destination_point, haversine_m
from leader_link import LeaderLink

HTTP_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5050
RADIO_PORT_OVERRIDE = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None
RADIO_BAUD = int(sys.argv[3]) if len(sys.argv) > 3 else 57600

# RFD900x-US shows up as this FTDI chip. Auto-detecting by VID:PID means we
# find it no matter what /dev/ttyUSBx number it gets -- that number is NOT
# stable across unplug/replug.
RADIO_VID = 0x0403
RADIO_PID = 0x6001

SEND_RATE_HZ = 10

# Shadow Mode: default distance behind the leader (along its heading) the
# follower targets -- adjustable live via /shadow/distance, see _shadow
# dict below -- and how often to recompute/resend that target.
DEFAULT_FOLLOW_DISTANCE_M = 7.0
# Was 3.0s -- far too slow for walking pace: the follow point (and the
# WP_SPEED commanded alongside it) was going stale for up to 3s at a time,
# and if the follower reached WP_RADIUS of the previous point before the
# next update landed, it just sat there waiting. 1.0s is a middle ground:
# much more responsive than 3s, without hammering the Cube's persistent
# parameter storage with param writes at a rate that once froze the whole
# Pi-side control script mid-test (goto() below also caches WP_SPEED/
# WP_RADIUS writes, skipping ones that haven't materially changed).
FOLLOW_INTERVAL_S = 1.0
# Default (and hard outer bound, see /shadow/max_speed) ceiling on the
# speed commanded to the follower, regardless of what the leader's GPS
# reports -- GPS-derived groundspeed is Doppler-based and noisy at walking
# pace; a real test briefly reported 18 m/s (~40mph) from noise alone
# while the leader was just walking. Live-adjustable via /shadow/max_speed
# (see _shadow dict below), same pattern as follow distance -- this is
# still a safety clamp, not an unbounded tuning knob: MAX_SPEED_LIMIT_MPS
# below is the hard ceiling nothing can override.
DEFAULT_MAX_FOLLOW_SPEED_MPS = 3.0
MAX_SPEED_LIMIT_MPS = 6.0
# If the follower has fallen more than this far behind where it should be
# (i.e. further from the follow point than the follow distance itself), it
# needs to catch up rather than just plod along at the leader's current
# pace -- matching the leader's speed alone barely closes a large gap since
# both are moving at roughly the same rate. CATCHUP_GAIN converts each
# meter of that excess into extra commanded speed (m/s), on top of the
# leader's own clamped pace. Catch-up is capped at CATCHUP_SPEED_FACTOR
# times the current max follow speed (so it scales with the live-adjustable
# max instead of a separate fixed constant), itself never exceeding
# MAX_SPEED_LIMIT_MPS. This also covers "come to me" when Shadow Mode is
# first enabled from far away -- no separate step needed, the same
# distance-based logic naturally drives there quickly then settles into
# pace-matching once caught up.
CATCHUP_GAIN = 0.3
CATCHUP_SPEED_FACTOR = 1.5
# If the leader's telemetry goes stale/invalid for this long while Shadow
# Mode is active, stop the follower rather than keep driving toward an
# increasingly outdated point.
LEADER_STALE_TIMEOUT_S = 10.0

# In dev, this is just ../frontend/dist relative to this source file. When
# frozen into a standalone .exe (PyInstaller), __file__ instead points into
# the bundle's temp extraction dir (sys._MEIPASS) -- the frontend build gets
# placed there too, under "frontend_dist" (see the --add-data mapping in
# the GitHub Actions build workflow).
if getattr(sys, "frozen", False):
    FRONTEND_DIST = Path(sys._MEIPASS) / "frontend_dist"
else:
    FRONTEND_DIST = Path(__file__).resolve().parent.parent / "frontend" / "dist"

app = Flask(__name__, static_folder=str(FRONTEND_DIST), static_url_path="/")
CORS(app)  # frontend dev server runs on a different port (5173) during development

_lock = threading.Lock()
_target = {"steer": 0.0, "throttle": 0.0, "arm": False}
_last_status = {"connected": False, "error": None}
_ser = None

leader = LeaderLink()
_shadow_lock = threading.Lock()
_shadow = {
    "enabled": False, "stopped_reason": None,
    "distance_m": DEFAULT_FOLLOW_DISTANCE_M,
    "max_speed_mps": DEFAULT_MAX_FOLLOW_SPEED_MPS,
}


def find_radio_port():
    """Auto-detect the RFD900x by USB VID:PID. Falls back to an explicit
    override (CLI arg) if given and no auto-detected match is found."""
    for p in serial.tools.list_ports.comports():
        if p.vid == RADIO_VID and p.pid == RADIO_PID:
            return p.device
    return RADIO_PORT_OVERRIDE


def send_json(obj):
    """Writes one JSON line to the radio, if connected. Used both for the
    continuous cmd heartbeat below and for one-shot goto/stop_nav requests
    from Flask handler threads -- locked so the two can't interleave
    mid-line on the shared serial connection."""
    with _lock:
        ser = _ser
        if ser is None:
            return False
        try:
            ser.write((json.dumps(obj) + "\n").encode())
            return True
        except serial.SerialException:
            return False


def radio_worker():
    global _ser
    while True:
        try:
            port = find_radio_port()
            if port is None:
                raise serial.SerialException(
                    f"No RFD900x radio found (looking for USB VID:PID {RADIO_VID:04x}:{RADIO_PID:04x})"
                )
            with _lock:
                _ser = serial.Serial(port, RADIO_BAUD, timeout=0.2)
            _last_status["error"] = None
            _last_status["port"] = port
            last_send = 0
            buf = b""
            while True:
                now = time.time()
                if now - last_send > 1.0 / SEND_RATE_HZ:
                    with _lock:
                        msg = {"type": "cmd", **_target}
                    send_json(msg)
                    last_send = now

                # drain any incoming status lines
                waiting = _ser.in_waiting
                if waiting:
                    buf += _ser.read(waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            obj = json.loads(line.decode(errors="ignore"))
                            if obj.get("type") == "status":
                                obj["connected"] = True
                                _last_status.update(obj)
                        except (json.JSONDecodeError, UnicodeDecodeError):
                            pass
                time.sleep(0.02)
        except Exception as e:
            with _lock:
                _ser = None
            _last_status["connected"] = False
            _last_status["error"] = str(e)
            time.sleep(2)


def shadow_worker():
    """While Shadow Mode is enabled, every FOLLOW_INTERVAL_S recomputes a
    point FOLLOW_DISTANCE_M behind the leader (along its current heading)
    and sends it as an ordinary goto() -- Shadow Mode is just repeated
    single-waypoint navigation aimed at a continuously-updated point, not a
    separate control mechanism. Stops the follower (and disables itself)
    if the leader's telemetry goes stale, rather than keep aiming at an
    increasingly outdated point with no way to know if that's still safe."""
    last_sent = 0.0
    last_good_leader = 0.0
    while True:
        with _shadow_lock:
            enabled = _shadow["enabled"]
            distance_m = _shadow["distance_m"]
            max_speed_mps = _shadow["max_speed_mps"]
        if not enabled:
            last_sent = 0.0
            last_good_leader = 0.0
            time.sleep(0.5)
            continue

        now = time.time()
        ls = leader.get_status()
        leader_ok = (
            ls["connected"] and ls["lat"] is not None and ls["lon"] is not None
            and ls["gps_fix"] is not None and ls["gps_fix"] >= 3
        )

        if leader_ok:
            last_good_leader = now
        elif last_good_leader and now - last_good_leader > LEADER_STALE_TIMEOUT_S:
            send_json({"type": "stop_nav"})
            with _shadow_lock:
                _shadow["enabled"] = False
                _shadow["stopped_reason"] = "leader telemetry lost"
            last_sent = 0.0
            last_good_leader = 0.0
            time.sleep(0.5)
            continue

        if leader_ok and now - last_sent > FOLLOW_INTERVAL_S:
            heading = ls["heading_deg"] if ls["heading_deg"] is not None else 0.0
            follow_lat, follow_lon = destination_point(
                ls["lat"], ls["lon"], (heading + 180) % 360, distance_m
            )
            msg = {"type": "goto", "lat": follow_lat, "lon": follow_lon}
            # max_speed_mps is the live-adjustable pace-matching ceiling
            # (from /shadow/max_speed, defaults to DEFAULT_MAX_FOLLOW_SPEED_MPS)
            # -- clamped again here to MAX_SPEED_LIMIT_MPS, the hard outer
            # bound nothing (including a bad value sent to that endpoint)
            # can exceed.
            max_speed_mps = max(0.0, min(max_speed_mps, MAX_SPEED_LIMIT_MPS))
            catchup_max_mps = min(max_speed_mps * CATCHUP_SPEED_FACTOR, MAX_SPEED_LIMIT_MPS)

            # Base pace: the leader's current speed, INCLUDING exactly 0
            # (stopped) -- `is not None` rather than a truthy check, so
            # "leader stopped" reliably tells the follower to slow to a
            # stop too, not just rely on WP_RADIUS arrival as a side effect
            # of the follow point barely moving. Clamped to max_speed_mps
            # -- GPS-derived groundspeed is Doppler-based and genuinely
            # unreliable at walking pace; a real burst hit 18 m/s (~40mph)
            # from GPS noise alone with the leader just walking.
            base_speed = None
            if ls["groundspeed_mps"] is not None:
                base_speed = max(0.0, min(ls["groundspeed_mps"], max_speed_mps))

            # Catch-up: if the follower is further from the follow point
            # than the follow distance itself calls for, matching the
            # leader's pace alone barely closes that gap (both end up
            # moving at about the same speed). Drive faster in proportion
            # to how far behind it is, so enabling Shadow Mode from far
            # away drives straight there at a reasonable clip instead of
            # crawling, and it also recovers if it falls behind mid-walk.
            follower_lat, follower_lon = _last_status.get("lat"), _last_status.get("lon")
            if follower_lat is not None and follower_lon is not None:
                gap_m = haversine_m(follower_lat, follower_lon, follow_lat, follow_lon)
                excess_m = max(0.0, gap_m - distance_m)
                catchup_speed = min(excess_m * CATCHUP_GAIN, catchup_max_mps)
                msg["speed"] = max(base_speed or 0.0, catchup_speed)
            elif base_speed is not None:
                msg["speed"] = base_speed
            send_json(msg)
            last_sent = now

        time.sleep(0.5)


threading.Thread(target=radio_worker, daemon=True).start()
threading.Thread(target=shadow_worker, daemon=True).start()


@app.route("/")
def index():
    if (FRONTEND_DIST / "index.html").exists():
        return send_from_directory(FRONTEND_DIST, "index.html")
    return (
        "Frontend not built. Run `npm run dev` in frontend/ for development, "
        "or `npm run build` there to serve it from here.",
        200,
    )


@app.route("/control", methods=["POST"])
def control():
    data = request.get_json(force=True)
    with _lock:
        _target["steer"] = max(-1.0, min(1.0, float(data.get("steer", 0.0))))
        _target["throttle"] = max(-1.0, min(1.0, float(data.get("throttle", 0.0))))
    return jsonify(ok=True)


@app.route("/arm", methods=["POST"])
def arm():
    with _lock:
        _target["arm"] = True
    return jsonify(ok=True)


@app.route("/disarm", methods=["POST"])
def disarm():
    with _lock:
        _target["arm"] = False
        _target["steer"] = 0.0
        _target["throttle"] = 0.0
    return jsonify(ok=True)


@app.route("/goto", methods=["POST"])
def goto():
    """One-shot autonomous drive-to-waypoint request -- NOT part of the
    continuous _target heartbeat (that would re-send it forever). Vehicle
    must already be armed; this endpoint doesn't arm it."""
    data = request.get_json(force=True)
    try:
        lat = float(data["lat"])
        lon = float(data["lon"])
    except (KeyError, TypeError, ValueError):
        return jsonify(ok=False, error="lat/lon required"), 400
    speed = data.get("speed")
    msg = {"type": "goto", "lat": lat, "lon": lon}
    if speed is not None:
        msg["speed"] = float(speed)
    sent = send_json(msg)
    return jsonify(ok=sent)


@app.route("/path", methods=["POST"])
def path():
    """Multi-waypoint autonomous path -- an ordered list of points the
    follower drives through via ArduPilot's own AUTO-mode mission
    execution (pi/rover_control.py's follow_path()), not a repeated
    goto() loop. Same one-shot-request shape as /goto: not part of the
    continuous _target heartbeat. Vehicle must already be armed."""
    data = request.get_json(force=True)
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 1:
        return jsonify(ok=False, error="waypoints (non-empty list) required"), 400
    try:
        clean = [{"lat": float(w["lat"]), "lon": float(w["lon"])} for w in waypoints]
    except (KeyError, TypeError, ValueError):
        return jsonify(ok=False, error="each waypoint needs numeric lat/lon"), 400
    speed = data.get("speed")
    msg = {"type": "path", "waypoints": clean}
    if speed is not None:
        msg["speed"] = float(speed)
    sent = send_json(msg)
    return jsonify(ok=sent)


@app.route("/stop_nav", methods=["POST"])
def stop_nav():
    sent = send_json({"type": "stop_nav"})
    return jsonify(ok=sent)


@app.route("/go_home", methods=["POST"])
def go_home():
    """One-shot 'come to wherever the leader currently is' -- just an
    ordinary goto() aimed at the leader Cube's live GPS position, reusing
    the exact same GUIDED-mode navigation (including its reverse-vs-heading
    check, so it backs straight up if the leader ends up behind it) as a
    manual map-click target. Not continuous tracking like Shadow Mode --
    a single fresh target sent once, same as clicking a point on the map.
    Requires the leader Cube to have a GPS fix."""
    checklist, ls = _shadow_checklist()
    if not checklist["gps_ok"]:
        return jsonify(ok=False, error="leader has no GPS fix"), 400
    data = request.get_json(silent=True) or {}
    speed = data.get("speed")
    msg = {"type": "goto", "lat": ls["lat"], "lon": ls["lon"]}
    if speed is not None:
        msg["speed"] = float(speed)
    sent = send_json(msg)
    return jsonify(ok=sent)


def _shadow_checklist():
    """The three things Shadow Mode requires on this PC: the radio (to
    reach the follower), the leader's Cube (to read its position from),
    and a valid GPS fix on that Cube (no fix = no usable position)."""
    with _lock:
        radio_ok = _ser is not None
    ls = leader.get_status()
    leader_ok = ls["connected"]
    gps_ok = leader_ok and ls["gps_fix"] is not None and ls["gps_fix"] >= 3
    return {"radio_ok": radio_ok, "leader_ok": leader_ok, "gps_ok": gps_ok}, ls


@app.route("/shadow/status")
def shadow_status():
    checklist, leader_status = _shadow_checklist()
    with _shadow_lock:
        shadow = dict(_shadow)
    return jsonify(**shadow, **checklist, leader=leader_status)


@app.route("/shadow/enable", methods=["POST"])
def shadow_enable():
    checklist, _ = _shadow_checklist()
    if not all(checklist.values()):
        return jsonify(ok=False, error="checklist not satisfied", **checklist), 400
    # Deliberately does NOT force a disarm here. It used to (as a "start from
    # a clean state" safety measure), but that meant enabling Shadow Mode
    # while already armed -- or arming, then enabling -- silently disarmed
    # the vehicle again with no clear cause, which was confusing and made it
    # look "spotty"/broken. The vehicle can't actually drive while disarmed
    # regardless (that's ArduPilot's own safety, not app logic), so there's
    # no safety loss in leaving whatever arm state was already set alone --
    # same non-intrusive pattern as the "Autonomy" GO button, which also
    # just requires `armed` to already be true rather than forcing it.
    with _shadow_lock:
        _shadow["enabled"] = True
        _shadow["stopped_reason"] = None
    return jsonify(ok=True)


@app.route("/shadow/disable", methods=["POST"])
def shadow_disable():
    with _shadow_lock:
        _shadow["enabled"] = False
        _shadow["stopped_reason"] = None
    send_json({"type": "stop_nav"})
    return jsonify(ok=True)


@app.route("/shadow/distance", methods=["POST"])
def shadow_distance():
    """Live-adjustable follow distance -- takes effect on the next follow-
    point computation, whether Shadow Mode is currently enabled or not."""
    data = request.get_json(force=True)
    try:
        distance_m = float(data["distance_m"])
    except (KeyError, TypeError, ValueError):
        return jsonify(ok=False, error="distance_m required"), 400
    if not (1.0 <= distance_m <= 50.0):
        return jsonify(ok=False, error="distance_m must be between 1 and 50"), 400
    with _shadow_lock:
        _shadow["distance_m"] = distance_m
    return jsonify(ok=True)


@app.route("/shadow/max_speed", methods=["POST"])
def shadow_max_speed():
    """Live-adjustable pace-matching speed ceiling -- takes effect on the
    next follow-point computation, whether Shadow Mode is currently enabled
    or not. Still bounded by MAX_SPEED_LIMIT_MPS regardless of what's sent
    here -- this is a tuning knob within a fixed safety envelope, not a way
    to remove the envelope."""
    data = request.get_json(force=True)
    try:
        max_speed_mps = float(data["max_speed_mps"])
    except (KeyError, TypeError, ValueError):
        return jsonify(ok=False, error="max_speed_mps required"), 400
    if not (0.5 <= max_speed_mps <= MAX_SPEED_LIMIT_MPS):
        return jsonify(ok=False, error=f"max_speed_mps must be between 0.5 and {MAX_SPEED_LIMIT_MPS}"), 400
    with _shadow_lock:
        _shadow["max_speed_mps"] = max_speed_mps
    return jsonify(ok=True)


@app.route("/emergency_stop", methods=["POST"])
def emergency_stop():
    """The one "make everything stop right now" action, available from
    both the home screen and Shadow Mode: disarms the follower, cancels
    any in-progress navigation, and disables Shadow Mode so nothing
    re-engages it a few seconds later. Bundled into one endpoint (rather
    than three separate frontend calls) so it's atomic from the caller's
    perspective -- no risk of a dropped request leaving it half-done."""
    with _lock:
        _target["arm"] = False
        _target["steer"] = 0.0
        _target["throttle"] = 0.0
    with _shadow_lock:
        _shadow["enabled"] = False
        _shadow["stopped_reason"] = None
    send_json({"type": "stop_nav"})
    send_json({"type": "cmd", "steer": 0.0, "throttle": 0.0, "arm": False})
    return jsonify(ok=True)


@app.route("/status")
def status():
    return jsonify(_last_status)


if __name__ == "__main__":
    detected = find_radio_port()
    print(f"Radio: auto-detecting RFD900x (VID:PID {RADIO_VID:04x}:{RADIO_PID:04x})"
          f"{f' -- currently at {detected}' if detected else ' -- none found yet, will keep retrying'}"
          f" @ {RADIO_BAUD}  |  HTTP: http://localhost:{HTTP_PORT}/")
    if getattr(sys, "frozen", False):
        # Packaged .exe has no separate dev server to open manually --
        # launch the UI in the default browser automatically. threading.Timer
        # so it fires after app.run() below has actually started listening,
        # not before.
        import webbrowser
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{HTTP_PORT}/")).start()
    app.run(host="127.0.0.1", port=HTTP_PORT, threaded=True)
