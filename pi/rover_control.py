#!/usr/bin/env python3
"""
RoverController: clean Python API for driving the RC car (Cube Blue via
MAVLink over USB) with steering + variable-speed throttle.

Usage:
    from rover_control import RoverController

    rover = RoverController()
    rover.connect()
    rover.arm()

    rover.set_steering(-0.5)   # half left
    rover.forward(0.3)         # 30% forward speed
    time.sleep(2)
    rover.brake()
    rover.backward(0.2)        # 20% reverse (handles brake-before-reverse)
    time.sleep(2)
    rover.stop()
    rover.disarm()
    rover.close()

A background thread continuously re-sends the current steering/throttle
command to the vehicle (ArduPilot reverts RC override if it isn't refreshed
regularly), and a watchdog forces neutral if the calling code stops updating
commands for longer than `command_timeout`.

Autonomous single-waypoint navigation (`goto`/`stop_navigation`) hands
steering and speed control off to ArduPilot's own GUIDED-mode navigation
(via MAV_CMD_DO_REPOSITION) rather than reimplementing a bearing/distance
control loop here -- the firmware's controller is the tested, correct one.
Calling any manual driving method (`set_steering`/`set_throttle`) resumes
MANUAL mode automatically, so touching the joystick always immediately
regains control from an in-progress autonomous run.
"""
import math
import os
import threading
import time

import serial
from pymavlink import mavutil


def _haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(d_lon / 2) ** 2
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)
    y = math.sin(d_lon) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(d_lon)
    return math.degrees(math.atan2(y, x)) % 360


def _angle_diff(a, b):
    """Smallest signed difference a-b in degrees, normalized to (-180, 180]."""
    return (a - b + 180) % 360 - 180


class RoverController:
    # goto(): if the target requires turning more than this many degrees
    # from the vehicle's current heading, drive straight backward to it
    # (MAV_CMD_DO_SET_REVERSE) instead of turning around to approach
    # forward. See goto()'s docstring for why this exists.
    REVERSE_THRESHOLD_DEG = 120.0

    def __init__(self,
                 port="/dev/ttyACM0",
                 baud=115200,
                 steer_min=1100, steer_max=1900,
                 throttle_min=1100, throttle_max=1900,
                 neutral=1500,
                 send_rate_hz=20,
                 command_timeout=1.0,
                 reverse_brake_time=0.3):
        self.port = port
        self.baud = baud
        self.steer_min = steer_min
        self.steer_max = steer_max
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max
        self.neutral = neutral
        self.send_period = 1.0 / send_rate_hz
        self.command_timeout = command_timeout
        self.reverse_brake_time = reverse_brake_time

        self.master = None
        self._modes = {}
        self._mode_names = {}
        self._lock = threading.Lock()
        self._target_steer_pwm = neutral
        self._target_throttle_pwm = neutral
        self._last_command_time = 0.0
        self._arm_request = None  # None / "arm" / "disarm"
        self._last_throttle_sign = 0  # -1, 0, +1 -- for brake-before-reverse
        self._reverse_brake_until = 0.0  # worker holds neutral until this time.time() deadline
        self._nav_target = None  # {"lat":..., "lon":...} while a goto() is active, else None

        # Multi-waypoint path state (follow_path()) -- all of this is only
        # ever touched from within the worker thread EXCEPT `_pending_path`,
        # which follow_path() sets from the caller's thread under `_lock`.
        # The actual mission upload happens as a small state machine inside
        # _worker_loop rather than a blocking call, because that thread
        # already owns all reads from the MAVLink connection -- a second
        # thread also calling recv_match() to wait for MISSION_REQUEST
        # would race it and could silently steal/drop messages.
        self._pending_path = None
        self._mission_upload_state = None  # None | "sending"
        self._mission_upload_deadline = 0.0
        self._mission_items = []  # [(lat, lon), ...] indexed by mission seq (0 = placeholder)
        self._mission_path_waypoints = []  # the real waypoints, 0-indexed (seq-1)
        self._mission_path_speed = None
        self._mission_path_radius = None

        # goto() is called frequently by Shadow Mode (repeated re-targeting
        # as the leader moves) -- these track the last WP_SPEED/WP_RADIUS
        # actually sent so goto() can skip the PARAM_SET when the value
        # hasn't materially changed, rather than writing to the Cube's
        # parameter storage on every single call.
        self._last_wp_speed = None
        self._last_wp_radius = None

        self._status_lock = threading.Lock()
        self._status = {
            "connected": False, "armed": False, "mode": None, "mode_name": None,
            "steer_pwm": None, "throttle_pwm": None,
            "gps_fix": None, "gps_sats": None, "lat": None, "lon": None,
            "cog_deg": None, "groundspeed_mps": None, "heading_deg": None,
            "nav_target": None, "nav_distance_m": None,
            "path_status": None, "path_index": None, "path_total": None,
        }

        self._stop_event = threading.Event()
        self._thread = None

    # ---------------------------------------------------------------- connection

    def connect(self, timeout=15):
        """Blocking: connects and waits for a valid ArduPilot heartbeat,
        then starts the background send/telemetry thread."""
        master = mavutil.mavlink_connection(self.port, baud=self.baud)
        hb = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if m and m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and m.get_srcSystem() != 0:
                hb = m
                master.target_system = m.get_srcSystem()
                master.target_component = m.get_srcComponent()
                break
        if hb is None:
            raise TimeoutError(f"No valid ArduPilot heartbeat on {self.port}")

        self.master = master
        self._modes = master.mode_mapping()
        self._mode_names = {v: k for k, v in self._modes.items()}

        if hb.custom_mode != self._modes.get("MANUAL"):
            master.set_mode(self._modes["MANUAL"])

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def close(self):
        """Stops the background thread and disarms/neutrals on the way out."""
        self.stop()
        self.disarm()
        time.sleep(0.3)
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        if self.master:
            self.master.close()

    # ---------------------------------------------------------------- arming

    def arm(self):
        with self._lock:
            self._arm_request = "arm"

    def disarm(self):
        with self._lock:
            self._arm_request = "disarm"
            self._target_steer_pwm = self.neutral
            self._target_throttle_pwm = self.neutral

    def is_armed(self):
        return self.get_status()["armed"]

    # ---------------------------------------------------------------- driving primitives

    def _resume_manual_if_needed(self, active):
        """A *non-neutral* manual driving call forces MANUAL mode -- so
        grabbing the joystick always immediately overrides an in-progress
        goto(), no separate "cancel autonomy" action required. Gated on
        `active` (the caller passing a genuinely non-zero value) rather than
        running on every call: radio_bridge.py calls set_steering/
        set_throttle continuously at ~10Hz even while idle (steer=throttle=0,
        the dead-man's-switch heartbeat) to keep the RC-override watchdog
        fed -- if idle calls also forced MANUAL, that heartbeat would yank
        control back from an in-progress goto() within ~100ms of it starting."""
        if not active:
            return
        manual_mode = self._modes.get("MANUAL")
        if manual_mode is None or not self.master:
            return
        with self._status_lock:
            current_mode = self._status.get("mode")
        if current_mode != manual_mode:
            self.master.set_mode(manual_mode)
            with self._lock:
                self._nav_target = None
            with self._status_lock:
                self._status["nav_target"] = None
                self._status["nav_distance_m"] = None
                self._status["path_status"] = None
                self._status["path_index"] = None
                self._status["path_total"] = None

    def set_steering(self, value):
        """value: -1.0 (full left) .. 0.0 (center) .. +1.0 (full right)"""
        value = max(-1.0, min(1.0, value))
        self._resume_manual_if_needed(active=abs(value) > 0.02)
        pwm = self.neutral + value * ((self.steer_max - self.neutral) if value >= 0
                                       else (self.neutral - self.steer_min))
        with self._lock:
            self._target_steer_pwm = pwm
            self._last_command_time = time.time()

    def set_throttle(self, value):
        """value: -1.0 (full reverse) .. 0.0 (neutral) .. +1.0 (full forward).
        Handles the brake-before-reverse transition automatically. Non-
        blocking: the brief neutral pulse is timed by the background send
        thread, not by sleeping here -- a very short call (e.g. a quick
        arrow-key tap) still gets the pulse applied correctly instead of
        being cut off before it takes effect."""
        value = max(-1.0, min(1.0, value))
        sign = 1 if value > 0.02 else (-1 if value < -0.02 else 0)
        self._resume_manual_if_needed(active=sign != 0)

        pwm = self.neutral + value * ((self.throttle_max - self.neutral) if value >= 0
                                       else (self.neutral - self.throttle_min))
        with self._lock:
            if sign < 0 and self._last_throttle_sign >= 0:
                # was stopped/forward, now asked to reverse: the worker thread
                # will hold neutral until this deadline, then start sending
                # the real reverse pwm -- so the ESC's forward/brake/reverse
                # logic actually engages reverse instead of treating this as
                # just another brake pulse.
                self._reverse_brake_until = time.time() + self.reverse_brake_time
            self._target_throttle_pwm = pwm
            self._last_command_time = time.time()
        self._last_throttle_sign = sign

    def forward(self, speed):
        """speed: 0.0 .. 1.0"""
        self.set_throttle(abs(speed))

    def backward(self, speed):
        """speed: 0.0 .. 1.0 (magnitude; brake-before-reverse handled internally)"""
        self.set_throttle(-abs(speed))

    def brake(self):
        """Neutral throttle. Also resets the reverse-engagement state so the
        next backward() call re-does the brake pulse (matches ESC semantics)."""
        with self._lock:
            self._target_throttle_pwm = self.neutral
            self._last_command_time = time.time()
            self._reverse_brake_until = 0.0
        self._last_throttle_sign = 0

    def stop(self):
        """Neutral steering and throttle. Does not force MANUAL mode (same
        "only non-neutral calls grab control back" rule as set_steering/
        set_throttle) -- use disarm() or stop_navigation() to actually
        interrupt an autonomous run."""
        with self._lock:
            self._target_steer_pwm = self.neutral
            self._target_throttle_pwm = self.neutral
            self._last_command_time = time.time()
            self._reverse_brake_until = 0.0
        self._last_throttle_sign = 0

    # ---------------------------------------------------------------- autonomous navigation

    def goto(self, lat, lon, speed=None, radius=4.0):
        """Drive autonomously to (lat, lon), handing steering and speed
        control to ArduPilot's own GUIDED-mode navigation via
        MAV_CMD_DO_REPOSITION (the same command GCS "click to drive here"
        features use) rather than a hand-rolled control loop. `speed` is in
        m/s; None/omitted keeps the vehicle's current default (WP_SPEED).
        `radius` (meters) is the "close enough, stop here" acceptance
        radius around the target (WP_RADIUS) -- too tight and the vehicle
        can pass the geometric arrival check late (GPS noise, turning
        geometry at speed) and just drive through the point instead of
        stopping there; defaults to 4m. Safe to call again with new
        coordinates/speed/radius to retarget mid-transit. Vehicle must
        already be armed -- this does not arm it.

        NOTE: on this firmware, DO_REPOSITION's own speed field (param1) is
        rejected outright (MAV_RESULT_FAILED) for any value other than -1
        (verified empirically: 0.5 through 2.5 m/s all failed, only -1 and
        0.0 were accepted) -- looks like a firmware bug/quirk in this
        ArduPilot Rover version's handling of that field. Workaround: set
        the WP_SPEED parameter directly first, then send DO_REPOSITION with
        speed left at -1 (unchanged) so it picks up the new WP_SPEED. Same
        approach used for WP_RADIUS, for consistency (and it's a plain
        vehicle-wide parameter, not something DO_REPOSITION takes inline).

        REVERSE: ArduPilot Rover's GUIDED navigation does not automatically
        decide to drive backward toward a target that's behind the vehicle
        -- left alone it just drives forward (pivoting first if
        WP_PIVOT_ANGLE allows), which for a target close to directly behind
        can mean driving forward into whatever's in front before it's
        turned enough, confirmed as a real crash. So before every
        reposition, this computes the bearing from the vehicle's own last-
        known position to the target, compares it against its own last-
        known heading, and explicitly commands MAV_CMD_DO_SET_REVERSE if
        the target requires more than REVERSE_THRESHOLD_DEG of turn --
        driving straight backward to it instead of turning around."""
        if not self.master:
            return
        with self._lock:
            self._nav_target = {"lat": lat, "lon": lon}

        with self._status_lock:
            cur_lat = self._status["lat"]
            cur_lon = self._status["lon"]
            cur_heading = self._status["heading_deg"]
        use_reverse = False
        if cur_lat is not None and cur_lon is not None and cur_heading is not None:
            bearing_to_target = _bearing_deg(cur_lat, cur_lon, lat, lon)
            use_reverse = abs(_angle_diff(bearing_to_target, cur_heading)) > self.REVERSE_THRESHOLD_DEG
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, 0,
            1 if use_reverse else 0, 0, 0, 0, 0, 0, 0
        )

        # `is not None` (not a truthy check) -- speed=0.0 is a real, meaningful
        # value (Shadow Mode commanding "stop" when the leader stops) and must
        # still reach WP_SPEED, not get silently skipped like a falsy 0 would.
        # Only actually sent when it differs from the last value written by
        # more than this threshold -- goto() is called frequently by Shadow
        # Mode as the leader moves, and PARAM_SET writes to the Cube's
        # persistent parameter storage, so re-sending on every call would
        # mean needless (and, at high call rates, genuinely disruptive --
        # this once froze the whole Pi-side control script mid-test) flash
        # writes. 0.3 m/s specifically because the leader's GPS-derived
        # speed is Doppler-noisy at walking pace and would otherwise defeat
        # a tighter threshold almost every call.
        if speed is not None and (self._last_wp_speed is None or abs(speed - self._last_wp_speed) > 0.3):
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b"WP_SPEED", float(speed), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self._last_wp_speed = speed
        if radius is not None and (self._last_wp_radius is None or abs(radius - self._last_wp_radius) > 0.1):
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b"WP_RADIUS", float(radius), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self._last_wp_radius = radius
        change_mode_bitmask = 1  # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        self.master.mav.command_int_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0,
            -1.0, change_mode_bitmask, 0, float("nan"),
            int(lat * 1e7), int(lon * 1e7), 0
        )

    def stop_navigation(self):
        """Cancels an in-progress goto() or follow_path() by switching to
        HOLD (ArduPilot immediately brakes and holds position) -- does not
        disarm."""
        with self._lock:
            self._nav_target = None
            self._pending_path = None
        self._mission_upload_state = None
        with self._status_lock:
            self._status["nav_target"] = None
            self._status["nav_distance_m"] = None
            self._status["path_status"] = None
            self._status["path_index"] = None
            self._status["path_total"] = None
        hold_mode = self._modes.get("HOLD")
        if self.master and hold_mode is not None:
            self.master.set_mode(hold_mode)

    def follow_path(self, waypoints, speed=None, radius=4.0):
        """Drive through an ORDERED list of waypoints (each a dict with
        "lat"/"lon") autonomously, using ArduPilot's own AUTO-mode mission
        execution -- same "let the firmware navigate" approach as goto(),
        just for a sequence of points instead of one. Uploads a standard
        MAVLink mission (MAV_CMD_NAV_WAYPOINT per point) and switches to
        AUTO once it's accepted.

        Runs ASYNCHRONOUSLY: this call only queues the request (returns
        immediately); the actual upload happens inside the background
        worker thread as it processes MISSION_REQUEST/MISSION_ACK
        messages, because that thread already owns all reads from the
        MAVLink connection -- calling recv_match() from this thread too,
        to wait for the upload handshake, would race it and could drop
        messages. Check get_status()'s "path_status" field
        ("uploading" -> "active" -> "complete", or an error string) and
        "path_index"/"path_total" to track progress.

        NOTE on reverse driving: only the FIRST leg (current position ->
        first waypoint) gets the same bearing-vs-heading check goto() uses
        to decide whether to back straight to it (see goto()'s docstring).
        Once AUTO mode is driving the uploaded mission, ArduPilot handles
        every waypoint-to-waypoint transition internally and this code
        doesn't get a chance to intervene per leg -- if some middle leg of
        the path also needs to reverse, that's not currently handled.
        Keep drawn paths to turns a normal forward drive (with pivot
        turns, since WP_PIVOT_ANGLE is set) can actually make until this
        is extended to watch MISSION_CURRENT and reapply the check live
        as each new leg starts."""
        if not self.master or not waypoints:
            return
        with self._lock:
            self._pending_path = {
                "waypoints": [dict(w) for w in waypoints],
                "speed": speed,
                "radius": radius,
            }

    # ---------------------------------------------------------------- status

    def get_status(self):
        with self._status_lock:
            return dict(self._status)

    # ---------------------------------------------------------------- internals

    def _send_raw(self, steer_pwm, throttle_pwm):
        if self.master:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                int(steer_pwm), 0, int(throttle_pwm), 0, 0, 0, 0, 0
            )

    def _start_mission_upload(self, path):
        """Kicks off a mission upload -- only called from the worker
        thread. Item 0 is a placeholder ArduPilot expects as the first
        mission item (conventionally "home"); duplicating the first real
        waypoint there is harmless since navigation is started at item 1,
        never item 0."""
        waypoints = path["waypoints"]
        self._mission_items = [(waypoints[0]["lat"], waypoints[0]["lon"])]
        self._mission_items += [(w["lat"], w["lon"]) for w in waypoints]
        self._mission_path_waypoints = waypoints
        self._mission_path_speed = path["speed"]
        self._mission_path_radius = path["radius"]
        self._mission_upload_state = "sending"
        self._mission_upload_deadline = time.time() + 10.0
        with self._status_lock:
            self._status["path_status"] = "uploading"
            self._status["path_index"] = None
            self._status["path_total"] = len(waypoints)
        self.master.mav.mission_count_send(
            self.master.target_system, self.master.target_component,
            len(self._mission_items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    def _on_mission_accepted(self):
        """Called from the worker thread once MISSION_ACK confirms the
        upload succeeded: point execution at item 1 (skip the item-0
        placeholder), apply speed/radius the same way goto() does, run the
        same reverse-vs-heading check goto() does but only for this first
        leg (see follow_path()'s docstring for why later legs aren't
        covered), and switch to AUTO to actually start driving."""
        self.master.mav.mission_set_current_send(
            self.master.target_system, self.master.target_component, 1
        )
        if self._mission_path_speed:
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b"WP_SPEED", float(self._mission_path_speed), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
        if self._mission_path_radius:
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b"WP_RADIUS", float(self._mission_path_radius), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

        first_wp = self._mission_path_waypoints[0]
        with self._status_lock:
            cur_lat = self._status["lat"]
            cur_lon = self._status["lon"]
            cur_heading = self._status["heading_deg"]
        use_reverse = False
        if cur_lat is not None and cur_lon is not None and cur_heading is not None:
            bearing = _bearing_deg(cur_lat, cur_lon, first_wp["lat"], first_wp["lon"])
            use_reverse = abs(_angle_diff(bearing, cur_heading)) > self.REVERSE_THRESHOLD_DEG
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, 0,
            1 if use_reverse else 0, 0, 0, 0, 0, 0, 0
        )

        auto_mode = self._modes.get("AUTO")
        if auto_mode is not None:
            self.master.set_mode(auto_mode)

        with self._lock:
            self._nav_target = dict(first_wp)
        with self._status_lock:
            self._status["path_status"] = "active"
            self._status["path_index"] = 0
            self._status["nav_target"] = dict(first_wp)

    def _worker(self):
        try:
            self._worker_loop()
        except (serial.SerialException, OSError) as e:
            # The Cube's USB link died (e.g. it power-cycled/re-enumerated).
            # This thread dying silently would leave the process running but
            # unable to send anything -- kill the whole process instead so
            # systemd's Restart=on-failure brings it back with a fresh
            # connection, rather than a zombie that looks alive but isn't.
            print(f"Fatal: lost connection to Cube ({e}). Exiting for restart.")
            os._exit(1)

    def _worker_loop(self):
        last_send = 0
        while not self._stop_event.is_set():
            now = time.time()

            with self._lock:
                steer_pwm = self._target_steer_pwm
                throttle_pwm = self._target_throttle_pwm
                last_cmd = self._last_command_time
                reverse_brake_until = self._reverse_brake_until
                arm_req = self._arm_request
                self._arm_request = None
                nav_target = self._nav_target
                pending_path = self._pending_path
                self._pending_path = None

            if pending_path:
                self._start_mission_upload(pending_path)
            if self._mission_upload_state == "sending" and now > self._mission_upload_deadline:
                print("Mission upload timed out, abandoning.")
                self._mission_upload_state = None
                with self._status_lock:
                    self._status["path_status"] = "upload timed out"

            if now - last_cmd > self.command_timeout:
                steer_pwm, throttle_pwm = self.neutral, self.neutral
            elif now < reverse_brake_until:
                # mid brake-before-reverse pulse: hold throttle neutral
                # (steering still passes through normally) until it elapses
                throttle_pwm = self.neutral

            if arm_req == "arm":
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0
                )
            elif arm_req == "disarm":
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0
                )

            if now - last_send > self.send_period:
                self._send_raw(steer_pwm, throttle_pwm)
                last_send = now

            m = self.master.recv_match(blocking=False)
            while m:
                t = m.get_type()
                mission_accepted = False
                with self._status_lock:
                    self._status["connected"] = True
                    if (t == "HEARTBEAT" and m.get_srcSystem() == self.master.target_system
                            and m.get_srcComponent() == self.master.target_component):
                        self._status["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        self._status["mode"] = m.custom_mode
                        self._status["mode_name"] = self._mode_names.get(m.custom_mode)
                    elif t == "SERVO_OUTPUT_RAW":
                        # Steering lives on MAIN OUT2 (SERVO2_FUNCTION=26), not OUT1 --
                        # OUT1's driver failed, so steering was remapped in ArduPilot params.
                        self._status["steer_pwm"] = m.servo2_raw
                        self._status["throttle_pwm"] = m.servo3_raw
                    elif t == "GPS_RAW_INT":
                        self._status["gps_fix"] = m.fix_type
                        self._status["gps_sats"] = m.satellites_visible
                        # cog (course over ground) is where the GPS says it's actually
                        # moving -- meaningless/noisy at a standstill. heading_deg from
                        # ATTITUDE below is the compass/AHRS-derived facing direction,
                        # valid even stationary, so that's what the UI displays.
                        self._status["cog_deg"] = m.cog / 100 if m.cog != 65535 else None
                        self._status["groundspeed_mps"] = m.vel / 100 if m.vel != 65535 else None
                        self._status["lat"] = m.lat / 1e7
                        self._status["lon"] = m.lon / 1e7
                        if nav_target:
                            self._status["nav_target"] = dict(nav_target)
                            self._status["nav_distance_m"] = _haversine_m(
                                m.lat / 1e7, m.lon / 1e7, nav_target["lat"], nav_target["lon"]
                            )
                    elif t == "ATTITUDE":
                        self._status["heading_deg"] = math.degrees(m.yaw) % 360
                    elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and self._mission_upload_state == "sending":
                        if 0 <= m.seq < len(self._mission_items):
                            item_lat, item_lon = self._mission_items[m.seq]
                            self.master.mav.mission_item_int_send(
                                self.master.target_system, self.master.target_component, m.seq,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                0, 1,  # current, autocontinue
                                0, 0, 0, float("nan"),
                                int(item_lat * 1e7), int(item_lon * 1e7), 0,
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                            )
                    elif t == "MISSION_ACK" and self._mission_upload_state == "sending":
                        self._mission_upload_state = None
                        if m.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            # Deferred until after this `with self._status_lock:` block
                            # exits -- _on_mission_accepted() takes self._status_lock
                            # itself, and it's a plain (non-reentrant) Lock, so calling
                            # it while still holding the lock here would deadlock the
                            # worker thread forever with no exception/traceback.
                            mission_accepted = True
                        else:
                            print(f"Mission upload rejected: type={m.type}")
                            self._status["path_status"] = f"upload rejected ({m.type})"
                    elif t == "MISSION_CURRENT" and self._status.get("path_status") == "active":
                        idx = m.seq - 1  # seq 0 is the placeholder item
                        if 0 <= idx < len(self._mission_path_waypoints):
                            target = self._mission_path_waypoints[idx]
                            # Nested lock (status_lock already held, _lock taken here) --
                            # safe: no other code path acquires these two in the
                            # opposite order (goto()/follow_path() always take them
                            # sequentially, never nested), so this can't deadlock.
                            with self._lock:
                                self._nav_target = dict(target)
                            self._status["path_index"] = idx
                            self._status["nav_target"] = dict(target)
                    elif t == "MISSION_ITEM_REACHED" and self._status.get("path_status") == "active":
                        if m.seq == len(self._mission_items) - 1:
                            self._status["path_status"] = "complete"
                if mission_accepted:
                    self._on_mission_accepted()
                m = self.master.recv_match(blocking=False)

            time.sleep(0.01)


if __name__ == "__main__":
    # Small manual demo. Car should be lifted / wheels clear before running.
    rover = RoverController()
    print("Connecting...")
    rover.connect()
    print("Connected:", rover.get_status())

    rover.arm()
    time.sleep(1)
    print("Armed:", rover.get_status())

    print("Steering left...")
    rover.set_steering(-1.0)
    time.sleep(1.5)
    print(rover.get_status())

    print("Steering right...")
    rover.set_steering(1.0)
    time.sleep(1.5)
    print(rover.get_status())

    rover.set_steering(0.0)

    print("Forward 30%...")
    rover.forward(0.3)
    time.sleep(1.5)
    print(rover.get_status())

    print("Brake...")
    rover.brake()
    time.sleep(0.5)

    print("Backward 20%...")
    rover.backward(0.2)
    time.sleep(1.5)
    print(rover.get_status())

    print("Stop, disarm, close.")
    rover.stop()
    rover.disarm()
    time.sleep(0.5)
    rover.close()
