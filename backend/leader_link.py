"""Read-only MAVLink connection to the LEADER vehicle's flight controller
(Cube Orange/Orange+) over USB, on this PC -- for Shadow Mode. This is
purely a GPS/heading/speed *source*; nothing here ever arms, steers, or
otherwise commands that Cube. The leader vehicle is driven by a person; we
just read where it is and how fast/which way it's going.

Auto-detects by USB VID:PID and auto-reconnects on drop, the same pattern
pi/radio_bridge.py uses for the follower's Cube (including trying every
matching port, since these Cubes expose two USB serial interfaces and only
one speaks MAVLink).
"""
import math
import threading
import time

import serial
import serial.tools.list_ports
from pymavlink import mavutil

CUBE_VID = 0x2dae
# Different Cube variants report different PIDs: 0x1016 is plain Cube
# Orange/Blue (confirmed on the follower's Cube Blue -- it identifies as
# "CubeOrange" over USB despite the different paint/branding), 0x1058 is
# Cube Orange+ (confirmed on the leader's Cube via `serial.tools.list_ports`
# -- shows up as "CubeOrange+"). Match either, since the user may run either
# as the leader vehicle's flight controller.
CUBE_PIDS = {0x1016, 0x1058}


class LeaderLink:
    def __init__(self, baud=115200):
        self.baud = baud
        self._status_lock = threading.Lock()
        self._status = {
            "connected": False,
            "gps_fix": None, "gps_sats": None,
            "lat": None, "lon": None,
            "groundspeed_mps": None, "heading_deg": None,
        }
        self._stop_event = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()

    def get_status(self):
        with self._status_lock:
            return dict(self._status)

    def close(self):
        self._stop_event.set()

    # ---------------------------------------------------------------- internals

    def _find_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()
                if p.vid == CUBE_VID and p.pid in CUBE_PIDS]

    def _connect_once(self):
        for port in self._find_ports():
            try:
                candidate = mavutil.mavlink_connection(port, baud=self.baud)
                hb = candidate.recv_match(type="HEARTBEAT", blocking=True, timeout=4)
                if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcSystem() != 0:
                    candidate.target_system = hb.get_srcSystem()
                    candidate.target_component = hb.get_srcComponent()
                    return candidate
                candidate.close()
            except (TimeoutError, OSError, serial.SerialException):
                continue
        return None

    def _worker(self):
        while not self._stop_event.is_set():
            master = self._connect_once()
            if master is None:
                with self._status_lock:
                    self._status["connected"] = False
                time.sleep(2)
                continue

            with self._status_lock:
                self._status["connected"] = True
            last_heartbeat = time.time()

            try:
                while not self._stop_event.is_set():
                    m = master.recv_match(blocking=True, timeout=1.0)
                    if m is None:
                        if time.time() - last_heartbeat > 5.0:
                            raise TimeoutError("no heartbeat from leader Cube in 5s")
                        continue
                    if m.get_srcSystem() != master.target_system:
                        continue
                    t = m.get_type()
                    if t == "HEARTBEAT":
                        last_heartbeat = time.time()
                    elif t == "GPS_RAW_INT":
                        with self._status_lock:
                            self._status["gps_fix"] = m.fix_type
                            self._status["gps_sats"] = m.satellites_visible
                            self._status["lat"] = m.lat / 1e7
                            self._status["lon"] = m.lon / 1e7
                            self._status["groundspeed_mps"] = m.vel / 100 if m.vel != 65535 else None
                    elif t == "ATTITUDE":
                        with self._status_lock:
                            self._status["heading_deg"] = math.degrees(m.yaw) % 360
            except (serial.SerialException, OSError, TimeoutError):
                pass
            finally:
                with self._status_lock:
                    self._status["connected"] = False
                try:
                    master.close()
                except Exception:
                    pass
                time.sleep(1)
