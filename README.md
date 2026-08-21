# FollowBot — manual + autonomous RC control

An RC car (Cube Blue flight controller running ArduPilot Rover) driven
wirelessly from this PC — no WiFi/network needed between the PC and the
car, just a point-to-point RFD900x radio link. Supports manual joystick
control, autonomous single-waypoint navigation (click the map, set a
speed, go), and Shadow Mode — autonomously following a second, person-
driven leader vehicle at a fixed distance (see "Shadow Mode" below).

## Architecture

```
This PC (browser joystick)         --RFD900x radio (USB, JSON)--> Pi --USB, MAVLink--> Cube Blue --> wheels
Leader's Cube (USB, MAVLink, read-only, Shadow Mode) ---^ (same PC)
```

The Cube link is deliberately kept on fast/reliable **wired USB**, never RF —
radio is only used for the PC↔Pi command channel, never for the real-time
actuator control itself. The Pi's `radio_bridge.py` is the only thing that
talks to the follower's Cube. In Shadow Mode, this PC *also* reads a second,
separate Cube directly over USB (the leader's) — see "Shadow Mode" below;
that connection never sends any command, purely reads GPS/heading/speed.

Protocol (newline-delimited JSON over the radio link):
```
PC -> Pi:  {"type":"cmd","steer":-1..1,"throttle":-1..1,"arm":true|false}
PC -> Pi:  {"type":"goto","lat":...,"lon":...,"speed":<m/s, optional>}
PC -> Pi:  {"type":"stop_nav"}
Pi -> PC:  {"type":"status", ...RoverController.get_status() fields...}
```

## Hardware

- **Raspberry Pi** (`followbot.local`, user `maarslab`) — connects to the
  Cube over USB and to an RFD900x-US radio over USB.
- **Cube Blue** (ArduPilot Rover 4.7.0) — the flight controller.
- **HERE4 GPS** — on the Cube's CAN2.
- **ESC** (Mamba XLX2, throttle) — **MAIN OUT3**.
- **Steering servo** — **MAIN OUT2** (`SERVO2_FUNCTION=26`, GroundSteering).
  Originally wired to MAIN OUT1, but that output's driver stopped producing
  any signal (confirmed by pulsing it directly over MAVLink — the FC
  computed the right PWM but nothing came out of the pin); steering was
  remapped to OUT2 in ArduPilot params as a workaround. `SERVO1_FUNCTION=0`
  (disabled) now. If OUT1 is ever repaired/replaced, the servo can be moved
  back and the params reverted (`SERVO1_FUNCTION=26`, `SERVO2_FUNCTION=0`) —
  nothing in the code hardcodes a specific output pin except the
  `SERVO_OUTPUT_RAW` field read for status (`rover_control.py`'s
  `steer_pwm`/`throttle_pwm` status fields, currently reading
  `servo2_raw`/`servo3_raw`).
- **Two RFD900x-US radios**, paired with each other (`NetID=25`,
  `Air Speed=64`, `ECC=off`, `TX Power=30`) — one on the Pi, one on this PC.
  Not connected to the Cube's TELEM2; both radios exist purely for the
  PC↔Pi command channel.
- **Leader vehicle** (Shadow Mode only) — a second vehicle, driven by a
  person, carrying this same PC + its RFD900x radio + a **Cube Orange or
  Orange+** connected over USB + a **HERE4 GPS on that Cube's CAN1**. This
  Cube is never driven/armed by this project — it's purely a position/
  heading/speed source, read directly over USB (see `backend/leader_link.py`).

## Project layout

```
Path_Follow_Bot/
├── frontend/     React + Vite UI — runs on this PC, talks to backend/ over HTTP
├── backend/      Flask API — runs on this PC, talks to the Pi over the radio
├── pi/           Deployed to the Raspberry Pi, talks to the Cube over USB
├── start_backend.sh
├── start_frontend.sh
└── README.md
```

`frontend/` and `backend/` are two separate processes on this PC (the
frontend calls the backend's HTTP API — CORS-enabled — rather than the
backend serving templates directly). `pi/` is a distinct deployment target:
its contents get copied to `/home/maarslab/` on the Pi and never run on
this PC.

| Location | File | What it is |
|---|---|---|
| `frontend/` | React + Vite app | The UI — a full-screen map (robot + leader position, click-to-set waypoint marker), a floating nav bar (speed, GO, STOP), and "🌓 Shadow Mode" / "🎮 Manual Control" buttons that open their respective overlays. Talks to `backend/` via `fetch()`. |
| `backend/` | `server.py` | Flask API. Reads the radio's serial port, exposes `/control`, `/arm`, `/disarm`, `/goto`, `/stop_nav`, `/status`, `/shadow/status`, `/shadow/enable`, `/shadow/disable`, `/shadow/distance`, `/emergency_stop`. Also serves `frontend/dist` statically at `/` once built — the path a future packaged/exe build would use. |
| `backend/` | `leader_link.py` | `LeaderLink` — read-only MAVLink connection to the leader vehicle's Cube over USB (Shadow Mode). Auto-detects/reconnects, never sends any command. |
| `backend/` | `geo_utils.py` | `haversine_m`, `destination_point` — the only "follow" math in this project; ArduPilot's own GUIDED-mode controller does the actual navigating (see "Shadow Mode"). |
| `backend/` | `requirements.txt` | Flask, flask-cors, pyserial, pymavlink (needed here now too, for `leader_link.py`). |
| `backend/` | `venv/` | Python virtualenv. |
| `pi/` | `rover_control.py` | `RoverController` — the control API: `connect()`, `arm()`/`disarm()`, `set_steering(-1..1)`, `set_throttle(-1..1)`, `forward()`/`backward()`, `brake()`, `stop()`, `goto(lat, lon, speed)`, `stop_navigation()`, `get_status()`, `close()`. Background thread continuously re-sends the current command (ArduPilot needs RC override refreshed) and forces neutral if commands stop arriving (`command_timeout`). |
| `pi/` | `radio_bridge.py` | Listens on the radio for `{"type":"cmd",...}` from the PC, drives `RoverController`, sends `{"type":"status",...}` back. Auto-detects both the radio and the Cube by USB VID:PID. Runs as the `followbot-radio-bridge` systemd service — autostarts on boot. |
| `pi/` | `start_radio_bridge.sh` | Manual foreground run (stop the service first). Ctrl+C to stop. |
| `pi/` | `followbot-radio-bridge.service` | The installed systemd unit (`/etc/systemd/system/` on the Pi). `Restart=on-failure` — if the Cube's USB link drops, the process deliberately exits so systemd restarts it fresh (see "Bugs fixed" below). |

## Running it

**Pi side** — runs automatically as a systemd service, nothing to do after
a reboot:
```bash
sudo systemctl status followbot-radio-bridge     # check it's active
journalctl -u followbot-radio-bridge -f          # watch logs
sudo systemctl restart followbot-radio-bridge    # restart if needed
```
For manual/foreground testing instead: `sudo systemctl stop
followbot-radio-bridge && ./start_radio_bridge.sh` (from `pi/`, Ctrl+C to
stop).

**PC side** — two processes, in two terminals:
```bash
./start_backend.sh                           # API on :5050, auto-detects the radio
./start_backend.sh 5050 /dev/ttyUSB0 57600    # or force a specific radio port

./start_frontend.sh                           # Vite dev server on :5173
```
Then open **http://localhost:5173/**. Both run in the foreground — Ctrl+C
stops each cleanly. `start_backend.sh` is safe to re-run any time (stops
any previous instance first).

First time only: `cd frontend && npm install`, `cd backend && python3 -m
venv venv && ./venv/bin/pip install -r requirements.txt`.

Controls: drag the joystick, or click into the page and use arrow keys
(←/→ steer, ↑/↓ throttle). Releasing snaps back to neutral immediately —
there's also a dead-man's-switch (frontend sends neutral on tab blur, the
Pi's `RoverController` forces neutral if commands stop arriving), so a
dropped connection anywhere in the chain forces neutral automatically.

## Map

`frontend/src/components/RobotMap.jsx` shows the robot's live position on a
Google Map, centered on the last GPS fix. It needs a Google Maps
JavaScript API key, served to the frontend by the **backend at runtime**
(`GET`/`POST /api/config` in `backend/server.py`) rather than baked into
the frontend build — this matters because a packaged `.exe` (see
"Packaging as a Windows exe" below) is built once in CI and distributed
afterward, with no build step left at install time to bake a key into.

- **Easiest**: just run the app with no key configured — the map shows a
  field to enter one, which saves it via `POST /api/config` to
  `backend/config.json` (gitignored, next to `server.py` in dev or next to
  the `.exe` when packaged) and it's remembered on every future launch, no
  rebuild needed.
- **Alternative for dev/CI**: set a `GOOGLE_MAPS_API_KEY` environment
  variable before starting `backend/server.py` — this always takes
  precedence over whatever's saved in `config.json`.

No frontend build-time env var is used for this anymore (the old
`VITE_GOOGLE_MAPS_API_KEY` / `frontend/.env` approach is gone).

No other backend changes were needed for the map itself beyond the key
endpoint above — `pi/rover_control.py` already
tracks `lat`/`lon` from `GPS_RAW_INT` in its status dict, which
`radio_bridge.py` already relays over the radio and `backend/server.py`
already exposes via `/status`. The frontend just polls that (already
happening every 500ms for the rest of the status panel) and feeds it to the
map. Until a fix is available (`lat`/`lon` both `0` or missing), the map
shows a "Waiting for GPS fix…" placeholder instead of a marker at `(0, 0)`.

**Viewport is sticky, not glued to the robot:** the map only centers itself
once, on first fix (`RobotMap.jsx`'s `initialCenterRef`) — passing a fresh
`{lat, lng}` object as the `center` prop on every 500ms status poll made
the map snap back to the robot's position every half-second, fighting any
manual pan/zoom. The 🎯 button (top-right of the map) recenters on demand;
the robot/waypoint markers themselves still update live regardless.

**Markers are plain default pins, managed imperatively, StrictMode
removed — the actual story behind several rounds of "marker won't show":**

1. First attempt used `<Marker icon={...}>` (react-google-maps's
   declarative wrapper) pointed at Google's hosted
   `mapfiles/ms/icons/*.png` images — ad blockers silently block that
   request (`ERR_BLOCKED_BY_CLIENT`), and the marker just doesn't render,
   no visible error anywhere in the app.
2. Switched to an inline `google.maps.Symbol` (no external request) — still
   used the declarative `<Marker>` wrapper, which turned out to be
   unreliable across this app's frequent 500ms poll-driven re-renders:
   worked once, then silently stopped rendering on a later update.
3. Rewrote marker management **imperatively** with the raw Maps JS API
   (`new google.maps.Marker(...)`, updated via `.setPosition()`,
   `RobotMap.jsx`) instead of the JSX wrapper — more reliable, but still
   failed specifically **on a real page refresh** (worked fine right after
   an HMR-applied code change, broke on an actual reload).
4. Root cause, finally confirmed: **React 18 StrictMode's dev-only
   intentional double-mount** (mount → cleanup → mount, meant to help
   surface exactly this class of bug). The marker-creation effect's
   cleanup removed the marker from the map but didn't clear the ref, so on
   the simulated remount the "does it already exist" check saw a non-null
   (but now map-less) ref and just repositioned an already-detached
   marker forever. Fixed two ways: (a) creation and position-updates are
   now in separate effects, with creation's cleanup properly nulling the
   ref (`RobotMap.jsx`), and (b) **`<StrictMode>` was removed entirely**
   from `frontend/src/main.jsx` — it's dev-only tooling with zero effect
   on production behavior, and removing it eliminates this whole class of
   double-mount bugs outright rather than requiring every future effect to
   be perfectly StrictMode-safe.

The waypoint marker uses the plain default pin (labeled "T"). The robot
marker is a rotating arrow (`google.maps.SymbolPath.FORWARD_CLOSED_ARROW`)
pointing in its actual heading — see "Heading indicator" below — falling
back to a plain dot if heading isn't available yet. Custom icons were never
actually the problem (see the debugging story above), so this is safe now
that the real StrictMode bug is fixed.

### Heading indicator

The robot marker rotates to show which way it's actually facing — pulled
from `ATTITUDE.yaw` (`rover_control.py`'s `heading_deg` status field), not
GPS course-over-ground (`cog_deg`, also still tracked/shown in the status
panel). `yaw` is ArduPilot's EKF-fused heading estimate, using the compass
as its absolute reference (fused with gyro for smoothness) — reliable even
when stationary, unlike course-over-ground which is only meaningful while
actually moving and would show a stale/arbitrary direction at a standstill.

### Approximate path line

While a waypoint is set (Autonomy mode), a dashed line is drawn from the
robot's live position to the target (`google.maps.Polyline` with a
repeated short-segment icon for the dashed effect — no separate library).
It's a **straight line, not the vehicle's actual planned trajectory** —
ArduPilot computes the real path internally and doesn't expose it over
MAVLink in a form worth re-plotting here; this is just enough to see
roughly where the robot is headed. Recreated whenever the waypoint itself
changes (a new map click cleanly replaces the old line), with a separate
effect keeping the robot-side endpoint live as it moves — same split
create/update pattern used for the markers, for the same StrictMode-safety
reasons (see the marker debugging story above).

**A single missed status poll used to blank out the whole map:** `App.jsx`
polls `/status` every 500ms; on a failed fetch it used to *replace* the
entire `status` state with `{connected:false, error:...}`, wiping
`lat`/`lon` along with everything else. Since the map only renders once
`lat`/`lon` are present, one harmless local network blip (unrelated to the
actual radio/Pi link — the backend's own `/status` already preserves last-
known position through real radio hiccups, see `backend/server.py`'s
`radio_worker` exception handler) was enough to suddenly blank the whole
map to "Waiting for GPS fix…". Fixed by merging (`setStatus(prev => ({...
prev, connected:false, ...}))`) instead of replacing, so a failed poll only
marks "disconnected" without discarding the last known position.

### Packaging as a Windows exe

`npm run build` in `frontend/` produces `frontend/dist/`; once that exists,
`backend/server.py`'s `/` route serves it directly — so `python
backend/server.py` alone is a single process serving both the UI and the
API. `.github/workflows/build-windows-exe.yml` wraps exactly that with
PyInstaller into a standalone `GhostPlanner.exe`, on every push to `main`
(also on-demand via the Actions tab's "Run workflow" button, and attached
to a GitHub Release on any `v*` tag push):

1. Builds the frontend (`npm ci && npm run build`).
2. Runs PyInstaller (`--onefile`) against `backend/server.py`, with
   `--collect-all pymavlink` (it picks its MAVLink dialect module via a
   dynamic import PyInstaller's static analysis can't see on its own —
   without this the exe builds fine but fails to talk to any Cube) and
   `--add-data "../frontend/dist;frontend_dist"` to bundle the built UI
   into the exe. `server.py` resolves that bundled path itself at runtime
   via `sys._MEIPASS` when `sys.frozen` is set (see `FRONTEND_DIST` near
   the top of the file) — no separate frozen-vs-dev config needed.
3. **Smoke-tests the built exe on the runner itself**: launches it, polls
   `/status` and `/` until both respond (or 20s times out the job), then
   kills it. This catches "the exe doesn't actually launch/serve" before
   anyone downloads it — it can't test radio/Cube hardware (none is
   attached to a GitHub-hosted runner), but it does catch packaging
   regressions in the frozen-path or pymavlink-bundling logic above.
4. Uploads `GhostPlanner.exe` as a workflow run artifact (Actions tab →
   the run → Artifacts), or as a Release asset for tag pushes.

The exe ships with **no Google Maps API key baked in** — see "Map" above
for how that's supplied at runtime instead. It's also unsigned, so
Windows SmartScreen/antivirus may flag it on first run; that's expected
for an unsigned PyInstaller build, not a sign of a bad one.

## Autonomous waypoint navigation ("Autonomy" — the default mode)

This is the default/home mode — the top bar's mode badge reads
**AUTONOMY** unless Shadow Mode is on or the vehicle is actually in
MANUAL. Click anywhere on the map to drop a waypoint (pin labeled "T", vs.
the robot's own pin labeled "R"; a dashed line shows the straight-line path
between them, redrawn whenever the waypoint changes), set a speed **in
mph** in the floating "Autonomy" nav bar, and hit **GO** (needs the
vehicle armed — the ARM/DISARM button is right in the top bar, no need to
open Manual Control just for that). **STOP** cancels it. Distance-
remaining and current mode are shown live while navigating. Speed is mph
only in the UI — converted to m/s at the point of calling the API, since
that's what the wire protocol and ArduPilot's `WP_SPEED` actually expect
(see below).

**How it works:** steering and speed control are handed off entirely to
ArduPilot Rover's own **GUIDED mode**, via a single `MAV_CMD_DO_REPOSITION`
command (`RoverController.goto()` in `pi/rover_control.py`) — the same
MAVLink command Mission Planner/QGroundControl use for "click to drive
here". This project does **not** implement its own bearing/distance
steering-control loop; the firmware's tested navigation controller does
that work. Re-sending `goto()` with new coordinates mid-transit retargets
immediately. **STOP** (`stop_navigation()`) switches to **HOLD** mode,
which brakes and holds position — it does not disarm.

**Firmware quirk — speed goes through `WP_SPEED`, not `DO_REPOSITION`'s own
speed field:** verified empirically (`COMMAND_ACK` result codes) that this
Rover firmware's `DO_REPOSITION` handler returns `MAV_RESULT_FAILED` for
*any* explicit speed value in param1 (tried 0.01 through 2.5 m/s — all
rejected); only `-1` (unchanged) or exactly `0.0` are accepted. Distance to
target was not the factor (tested 3m–60m, all failed at the same rate once
a nonzero speed was set). Workaround, and what `goto()` actually does:
set the `WP_SPEED` parameter directly first (`PARAM_SET`), then send
`DO_REPOSITION` with speed left at `-1` so it picks up the new `WP_SPEED`.
Functionally identical result, just routed around the broken field. If
this ever gets fixed upstream (or was a config issue misdiagnosed as a
firmware bug), it'd be worth retrying the direct param1 approach.

**Arrival radius (`WP_RADIUS`):** `goto(lat, lon, speed, radius=4.0)`
defaults to a 4m "close enough, stop here" bubble around the target,
set the same way as speed (`PARAM_SET` before the reposition command).
The stock default was 2m, which was too tight in practice — the vehicle
would drive past/beside the target instead of recognizing arrival and
stopping (GPS noise and turning geometry at speed can both make a small
radius easy to miss geometrically). 4m fixed it; adjust the default in
`rover_control.py` if it needs retuning for a given speed/vehicle.

**Pivot turning (`WP_PIVOT_ANGLE`):** was `0` on this vehicle, which
**disables pivot turns entirely** in ArduPilot Rover — any waypoint
requiring a real heading change forced a wide curved approach using the
vehicle's normal turn radius instead of turning to face the target first.
Symptom matched exactly: fine for waypoints roughly straight ahead, but
turns were bad and it would just drive past/through waypoints requiring an
actual turn instead of reaching them. Set to `45` (degrees) directly on
the vehicle — any waypoint requiring more than a 45° heading change now
triggers a pivot maneuver first. This is a vehicle-level tuning parameter
(not something `goto()` sets per-call, unlike speed/radius), changed once
via `PARAM_SET` and persisted on the Cube. If turning still isn't great
after this, `TURN_RADIUS` (currently 0.9m) is the next thing to check —
it should match the vehicle's actual achievable minimum turn radius, or
the path planner computes geometry it can't track well.

**Reverse driving for waypoints behind the vehicle:** ArduPilot Rover's
GUIDED navigation does not automatically decide to back up toward a target
that's behind it — left alone it always drives forward (pivoting first, if
`WP_PIVOT_ANGLE` allows), which for a target close to directly behind
means driving forward before it's turned enough — confirmed as **a real
crash** during testing (target behind the robot → robot drove straight
into what was in front of it instead of turning around). Fixed in
`RoverController.goto()`: before every reposition, it computes the bearing
from the vehicle's own last-known position to the target, compares that
against its own last-known heading (`heading_deg`, from `ATTITUDE.yaw` —
see "Heading indicator"), and if the required turn exceeds
`REVERSE_THRESHOLD_DEG` (120°, a class constant on `RoverController`), it
explicitly sends `MAV_CMD_DO_SET_REVERSE` first so the vehicle drives
straight backward to the point instead of attempting to turn around. Below
that threshold, normal forward driving (+ pivot turn if needed) applies as
before. This needs the vehicle's own position/heading to already be known
(from live telemetry) — if `goto()` is called before any fix has ever been
received, it can't compute a bearing and defaults to forward.

**Manual override always wins:** touching the joystick or arrow keys (a
genuinely non-zero steer/throttle value, not the idle heartbeat) forces the
vehicle back to MANUAL mode immediately and clears the active waypoint —
see `RoverController._resume_manual_if_needed()`. No separate "cancel
autonomy" action is needed; grabbing the stick is enough. This mattered
during implementation: the PC backend's continuous ~10Hz `cmd` heartbeat
(kept alive even when idle, steer=throttle=0, as the existing dead-man's-
switch/RC-override-refresh mechanism) would otherwise have yanked control
back out of GUIDED within ~100ms of every `goto()` — the fix gates the
mode-resume on the command actually being non-zero, not just present.

**Safety notes:**
- `goto()` does not arm the vehicle — arm explicitly first (Manual Control
  panel), same as regular driving.
- Once a `goto()` is sent, ArduPilot drives to that point autonomously on
  its own — it does not need continuous PC/radio contact to keep going
  (unlike RC-override manual driving, which needs a fresh command every
  `command_timeout` or it goes neutral). This means **STOP requires a live
  radio link**, same as every other control in this system — there's no
  new gap here, but it's worth remembering before sending a `goto()` at the
  edge of radio range.
- No obstacle avoidance is implemented here by design (LiDAR, if added, is
  intended to live outside this control path) — only drive to waypoints in
  a space known to be clear.

## Shadow Mode

Autonomously follows a second, person-driven **leader vehicle** at a fixed
distance behind it, matching its speed — start/stop when it starts/stops,
slow down when it slows down. Built entirely on top of the single-waypoint
`goto()` mechanism above: **not** a separate navigation system.

**Setup:** the leader vehicle carries this same PC, its RFD900x radio, and
a second flight controller (Cube Orange/Orange+) with its own HERE4 GPS on
CAN1. That Cube is read-only — `backend/leader_link.py` connects to it
directly over USB (auto-detected/reconnected the same way the Pi detects
its own Cube) purely to read position, heading, and groundspeed. It is
never armed or commanded by this project.

**The "🌓 Shadow Mode" panel** shows a three-item checklist that must all
be green before it can be enabled:
1. **Radio connected** — the RFD900x on this PC is open.
2. **Leader Cube connected** — `leader_link.py` has a live MAVLink heartbeat.
3. **Leader GPS fix** — that Cube's GPS has at least a 3D fix.

Follow distance is a **live-adjustable number input** in the panel (1–50m,
default 7m; `POST /shadow/distance`) — not a fixed constant — takes effect
on the next follow-point computation whether Shadow Mode is on or not.

**Enabling** disarms the follower robot first (same explicit-arm-before-
driving requirement as every other mode here) and starts a background loop
(`shadow_worker` in `server.py`) that, every `FOLLOW_INTERVAL_S` (3s),
computes a point at the current follow distance directly behind the
leader's heading (`geo_utils.destination_point`, offset by
`leader_heading + 180°`) and sends it as an ordinary `goto()`, with the
leader's current groundspeed as the target speed — **including exactly 0**
(leader stopped → follower's target speed is explicitly set to 0 too, not
just left at whatever it was), so "stop when the leader stops" is a real
commanded state, not a side effect of the follow point happening to stop
moving. That's the entire "follow" implementation — a loop recomputing
where the target point is, using ArduPilot's own GUIDED-mode controller
(see "Autonomous waypoint navigation" above) to actually drive there each
time, the same as a manual waypoint click. There's no separate convoy/
pursuit control loop.

**Arming and stopping — available both on the home screen and inside the
Shadow Mode panel** (not just buried in Manual Control):
- **ARM/DISARM** toggle button — top bar, and again inside the Shadow Mode
  panel for convenience while it's open.
- **🛑 EMERGENCY STOP** — top bar, and again inside the Shadow Mode panel.
  One action (`POST /emergency_stop`, bundled server-side so it can't be
  left half-done by a dropped request): disarms the follower, cancels any
  in-progress navigation, **and** disables Shadow Mode so nothing
  re-engages it a few seconds later. This is the "make it stop, full stop"
  button — prefer it over DISARM alone if Shadow Mode is active.

**Other safety behavior:**
- If the leader's telemetry goes stale for more than `LEADER_STALE_TIMEOUT_S`
  (10s) while Shadow Mode is active, the follower is stopped (`stop_nav`,
  same as HOLD mode) and Shadow Mode disables itself automatically — the
  panel shows why (`stopped_reason`). Better to stop than keep driving
  toward an increasingly outdated point.
- **Manual override still always wins** — touching the joystick immediately
  regains control exactly as with a manual `goto()` (see above). Note this
  does *not* auto-disable Shadow Mode on the PC side: the next periodic
  follow-point send (up to 3s later) will pull the vehicle back into
  GUIDED mode unless Shadow Mode is explicitly disabled too (or use
  🛑 EMERGENCY STOP, which does both at once).
- Disabling Shadow Mode (or 🛑 EMERGENCY STOP) sends `stop_nav` immediately.

**A real bug hit while building this:** the leader's Cube Orange+ reports a
**different USB PID (`0x1058`)** than the follower's Cube Blue/Orange
(`0x1016`) — different hardware variants, different IDs. `leader_link.py`
(and `pi/radio_bridge.py`, broadened for consistency/future-proofing) now
match either. If a leader/follower Cube is ever swapped for some other
Cube variant, check its actual PID via `serial.tools.list_ports` rather
than assuming it matches — don't extrapolate from one already-confirmed
device.

**Also hit:** the leader Cube's serial ports weren't accessible at all
(`PermissionError`) — Linux restricts raw serial devices to `root`/`dialout`
by default, and (unlike the radio) there was no udev rule granting broader
access for the Cube's VID:PID on this PC. Fixed with a new
`/etc/udev/rules.d/99-cube.rules` (matches `idVendor=2dae`, both known
`idProduct` values, same `MODE="0666"` pattern as the existing
`99-rfd900x.rules`). If Shadow Mode's checklist shows the leader Cube as
disconnected on a **new** PC this hasn't been set up on yet, check this
first — `ls -la /dev/ttyACM*` should show `crw-rw-rw-`, not `crw-rw----`.

## Device auto-detection

Both the radio and any Cube (follower's, on the Pi; leader's, on this PC
for Shadow Mode) are found by USB VID:PID scan (`serial.tools.list_ports`)
instead of a fixed device path — numbers like `/dev/ttyUSB0` or
`/dev/ttyACM0` aren't stable across unplug/replug or reboots. Radio:
`0403:6001` (FTDI). Cube: vendor `2dae`, product `1016` **or** `1058`
(different Cube variants report different PIDs — confirmed both on real
hardware, see "Shadow Mode" above) — a Cube actually exposes *two* USB
serial interfaces under its VID:PID (one is MAVLink, the other a console/
shell that never sends a heartbeat), so both `radio_bridge.py` and
`leader_link.py` try every match and keep the first one that answers with
a valid ArduPilot heartbeat.

Two udev rules grant `0666` permissions by VID:PID so no manual `chmod` is
ever needed: `/etc/udev/rules.d/99-rfd900x.rules` (radio, both machines)
and `/etc/udev/rules.d/99-cube.rules` (Cube, PC only — matches both known
PIDs; the Pi doesn't need this one since `radio_bridge.py` there already
runs as a service under a user in the `dialout` group). The radio rule
also creates a `/dev/rfd900x` symlink that nothing actually depends on
anymore.

## Known vehicle-side bypasses (not yet "production safe")

- No physical RC receiver is wired in — relies on `FS_THR_ENABLE=0` and a
  force-arm magic value (`21196`) to bypass RC failsafe/prearm checks.
  `ARMING_SKIPCHK` is this firmware's renamed `ARMING_CHECK`.
- `BRD_SAFETY_DEFLT=0` and `BRD_SAFETYOPTION=3` disable the physical
  safety-switch requirement entirely — no switch is wired to this Cube.
- Compass health warnings ("Compasses inconsistent"/"not healthy") were
  seen during setup; a calibration was attempted but may be incomplete —
  worth redoing before trusting heading for anything beyond manual driving.

These exist because this is a companion-computer-only bench setup with no
physical safety pilot. Revisit before relying on this for unattended
autonomous driving.

## Bugs fixed (useful if similar symptoms recur)

1. **Servo output frozen despite ARMED.** `BRD_SAFETY_DEFLT=1` (ArduPilot
   default) was waiting forever for a physical safety switch that was never
   wired. Fixed with `BRD_SAFETY_DEFLT=0` + `BRD_SAFETYOPTION=3`.
2. **Reported `armed` state flickering true/false every ~1s.** The Cube
   emits a second `HEARTBEAT` from a different MAVLink *component* (a
   phantom ADS-B placeholder, always `armed=False`) on the same system ID.
   `RoverController` was filtering by source system only — fixed by also
   checking source component.
3. **Steering completely dead, throttle fine.** Confirmed via direct
   MAVLink testing (bypassing all Python/radio code) that MAIN OUT1's
   driver produced no signal even though the FC computed the correct PWM
   value. Root cause was a disconnected wire; steering was also remapped to
   MAIN OUT2 as a fallback path while debugging and left there (see
   "Hardware" above).
4. **Bridge process looked alive but was fully unresponsive after any Cube
   USB drop** (e.g. a battery power-cycle re-enumerating the Cube).
   `RoverController`'s background send/telemetry thread would crash
   silently on the resulting `SerialException`, but the main
   `radio_bridge.py` loop kept running — still printing "ARM
   requested"/"DISARM requested" from incoming radio traffic, but never
   actually sending anything to the Cube since the thread that does that
   was dead. Fixed by having that thread kill the whole process
   (`os._exit(1)`) on a fatal serial error, so systemd's
   `Restart=on-failure` brings the bridge back with a fresh connection
   instead of leaving a zombie.

## Troubleshooting

- **UI shows "not connected" / permission denied**: check the udev rule
  applied — `ls -la /dev/ttyUSB*` should show `crw-rw-rw-`, not `crw-rw----`.
- **UI shows "No RFD900x radio found"**: radio isn't plugged in yet, or
  Linux hasn't enumerated it — check the physical connection. This is
  `backend/server.py`'s error, visible in its own terminal output too.
- **UI shows connected, but nothing moves**: check the bridge is actually
  running and connected to the Cube on the Pi:
  `journalctl -u followbot-radio-bridge -n 30 --no-pager`.
- **Frontend loads but shows "lost connection to local server"**: the
  backend isn't running, or CORS/port mismatch — confirm
  `./start_backend.sh` is up and `frontend/src/api.js`'s `VITE_BACKEND_URL`
  (default `http://localhost:5050`) matches.
- **Radio pairing**: both radios need matching `NetID`/`Air Speed` — check
  via Mission Planner's **Setup → Optional Hardware → SiK Radio** if the UI
  shows "not connected" with no error at all.
