#!/usr/bin/env python3
"""
Runs on the Pi. Listens on the RFD900x radio's serial port for commands from
the PC's GUI, and drives the Cube locally over the existing USB link via
RoverController. Sends status back over the same radio link periodically.

The radio is auto-detected by USB VID:PID -- no fixed device name needed,
survives unplug/replug even if the /dev/ttyUSBx number changes. If the radio
link drops, this reconnects automatically without disturbing the Cube
connection (which stays open the whole time).

Protocol (newline-delimited JSON):
  PC -> Pi:  {"type":"cmd","steer":-1..1,"throttle":-1..1,"arm":bool}
  PC -> Pi:  {"type":"goto","lat":...,"lon":...,"speed":<m/s, optional>}
  PC -> Pi:  {"type":"path","waypoints":[{"lat":...,"lon":...},...],"speed":<m/s, optional>}
  PC -> Pi:  {"type":"stop_nav"}
  Pi -> PC:  {"type":"status", ...RoverController.get_status() fields...}

RoverController already has its own watchdog: if this script stops calling
set_steering/set_throttle (e.g. because the radio link drops), it forces
neutral output on its own after `command_timeout` seconds. So no separate
watchdog is needed here beyond just not calling those methods when nothing
is received.

Run: python3 radio_bridge.py [radio_port] [radio_baud] [cube_port] [cube_baud]
     radio_port is an optional override; leave empty/omit to auto-detect.
"""
import json
import signal
import sys
import time

import serial
import serial.tools.list_ports

from rover_control import RoverController


class _StopSignal(Exception):
    """Raised on SIGTERM so systemd `stop` runs the same clean-shutdown path
    as Ctrl+C (SIGINT/KeyboardInterrupt) instead of killing the process
    before it can disarm/neutral the vehicle."""
    pass


def _handle_sigterm(signum, frame):
    raise _StopSignal()


signal.signal(signal.SIGTERM, _handle_sigterm)

RADIO_PORT_OVERRIDE = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None
RADIO_BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
CUBE_PORT_OVERRIDE = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None
CUBE_BAUD = int(sys.argv[4]) if len(sys.argv) > 4 else 115200

# RFD900x-US shows up as this FTDI chip. Auto-detecting by VID:PID means we
# find it no matter what /dev/ttyUSBx number it gets.
RADIO_VID = 0x0403
RADIO_PID = 0x6001

# Cube Blue/Orange shows up as this VID:PID once booted (its bootloader briefly
# enumerates differently first). Auto-detecting avoids hardcoding /dev/ttyACM0,
# which shifts across reboots/replugs whenever another ACM device is present.
# Different Cube variants report different PIDs -- 0x1016 is plain Cube
# Orange/Blue (confirmed on this vehicle's Cube Blue), 0x1058 is Cube Orange+
# (confirmed on the Shadow Mode leader vehicle's Cube, backend/leader_link.py)
# -- matching both here too in case this Cube ever gets swapped for a
# different variant.
CUBE_VID = 0x2dae
CUBE_PIDS = {0x1016, 0x1058}

STATUS_SEND_HZ = 2


def find_radio_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == RADIO_VID and p.pid == RADIO_PID:
            return p.device
    return RADIO_PORT_OVERRIDE


def find_cube_ports():
    """The Cube exposes two USB serial interfaces under the same VID:PID
    (one is the MAVLink console/shell, not MAVLink) -- try every match, not
    just the first, since which one enumerates first isn't guaranteed."""
    ports = [p.device for p in serial.tools.list_ports.comports()
             if p.vid == CUBE_VID and p.pid in CUBE_PIDS]
    if CUBE_PORT_OVERRIDE:
        ports.insert(0, CUBE_PORT_OVERRIDE)
    return ports


print("Connecting to Cube over USB...")
rover = None
while rover is None:
    cube_ports = find_cube_ports()
    if not cube_ports:
        print(f"No Cube found (VID {CUBE_VID:04x}, PID in {[f'{p:04x}' for p in CUBE_PIDS]}), retrying...")
        time.sleep(2)
        continue
    for cube_port in cube_ports:
        try:
            candidate = RoverController(port=cube_port, baud=CUBE_BAUD)
            candidate.connect(timeout=4)
            rover = candidate
            break
        except (TimeoutError, serial.SerialException) as e:
            print(f"No MAVLink on {cube_port} ({e}), trying next...")
    if rover is None:
        time.sleep(1)
print("Cube connected:", rover.get_status())

print("Bridge running. Waiting for radio + commands from PC...")

try:
    while True:
        port = find_radio_port()
        if port is None:
            print(f"No RFD900x radio found (VID:PID {RADIO_VID:04x}:{RADIO_PID:04x}), retrying...")
            time.sleep(2)
            continue

        try:
            ser = serial.Serial(port, RADIO_BAUD, timeout=0.1)
            print(f"Radio connected on {port}")
        except serial.SerialException as e:
            print(f"Failed to open radio on {port}: {e}")
            time.sleep(2)
            continue

        last_arm_state = None
        last_status_send = 0
        buf = b""

        try:
            while True:
                now = time.time()

                waiting = ser.in_waiting
                if waiting:
                    buf += ser.read(waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            msg = json.loads(line.decode(errors="ignore"))
                        except (json.JSONDecodeError, UnicodeDecodeError):
                            continue
                        msg_type = msg.get("type")

                        if msg_type == "cmd":
                            steer = max(-1.0, min(1.0, float(msg.get("steer", 0.0))))
                            throttle = max(-1.0, min(1.0, float(msg.get("throttle", 0.0))))
                            arm_req = bool(msg.get("arm", False))

                            rover.set_steering(steer)
                            rover.set_throttle(throttle)

                            if arm_req != last_arm_state:
                                if arm_req:
                                    print("ARM requested")
                                    rover.arm()
                                else:
                                    print("DISARM requested")
                                    rover.disarm()
                                last_arm_state = arm_req

                        elif msg_type == "goto":
                            try:
                                lat = float(msg["lat"])
                                lon = float(msg["lon"])
                            except (KeyError, TypeError, ValueError):
                                continue
                            speed = msg.get("speed")
                            print(f"GOTO requested: ({lat}, {lon}) @ {speed if speed else 'default'} m/s")
                            rover.goto(lat, lon, speed)

                        elif msg_type == "path":
                            waypoints = msg.get("waypoints")
                            if not isinstance(waypoints, list) or not waypoints:
                                continue
                            try:
                                clean = [{"lat": float(w["lat"]), "lon": float(w["lon"])} for w in waypoints]
                            except (KeyError, TypeError, ValueError):
                                continue
                            speed = msg.get("speed")
                            print(f"PATH requested: {len(clean)} waypoints @ {speed if speed else 'default'} m/s")
                            rover.follow_path(clean, speed)

                        elif msg_type == "stop_nav":
                            print("STOP NAV requested")
                            rover.stop_navigation()

                if now - last_status_send > 1.0 / STATUS_SEND_HZ:
                    status = rover.get_status()
                    status["type"] = "status"
                    ser.write((json.dumps(status) + "\n").encode())
                    last_status_send = now

                time.sleep(0.02)

        except (serial.SerialException, OSError) as e:
            print(f"Radio link lost ({e}), reconnecting...")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1)
            continue

except (KeyboardInterrupt, _StopSignal):
    pass
finally:
    print("Shutting down: stop, disarm, close.")
    rover.stop()
    rover.disarm()
    time.sleep(0.3)
    rover.close()
