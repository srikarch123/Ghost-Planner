import { useCallback, useEffect, useState } from "react";
import { api } from "./api";
import RobotMap, { ApiKeyForm } from "./components/RobotMap";
import ControlPanel from "./components/ControlPanel";
import NavControls from "./components/NavControls";
import ShadowPanel from "./components/ShadowPanel";
import "./App.css";

const ARROW_KEYS = ["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"];

export default function App() {
  const [status, setStatus] = useState({ connected: false });
  const [controlOpen, setControlOpen] = useState(false);
  const [shadowOpen, setShadowOpen] = useState(false);
  const [shadow, setShadow] = useState({
    enabled: false, radio_ok: false, leader_ok: false, gps_ok: false, leader: {},
  });
  const [path, setPath] = useState([]);
  const [loop, setLoop] = useState(false);
  // null = still asking the backend; "" = confirmed no key set yet; a
  // string = the actual key. Served from the backend rather than baked in
  // at build time (Vite's old VITE_GOOGLE_MAPS_API_KEY approach) because a
  // packaged .exe is built once in CI and distributed afterward -- there's
  // no build step left at install time to bake a key into, so whoever runs
  // it needs to be able to enter their own without a rebuild.
  const [mapsApiKey, setMapsApiKey] = useState(null);
  useEffect(() => {
    api
      .getConfig()
      .then((cfg) => setMapsApiKey(cfg.google_maps_api_key || ""))
      .catch(() => setMapsApiKey(""));
  }, []);
  const saveMapsApiKey = (key) => api.setGoogleMapsApiKey(key).then(() => setMapsApiKey(key));
  const [maxForward, setMaxForward] = useState(
    () => Number(localStorage.getItem("maxForward")) || 0.6
  );
  const [maxReverse, setMaxReverse] = useState(
    () => Number(localStorage.getItem("maxReverse")) || 0.4
  );
  // Speed shown/edited in the UI as mph (more intuitive than m/s) --
  // converted to m/s only at the point of calling api.goto(), since that's
  // the unit ArduPilot/WP_SPEED actually expects.
  const [navSpeedMph, setNavSpeedMph] = useState(
    () => Number(localStorage.getItem("navSpeedMph")) || 3.0
  );
  const [followDistanceM, setFollowDistanceM] = useState(
    () => Number(localStorage.getItem("followDistanceM")) || 7.0
  );
  // Shadow Mode's pace-matching speed ceiling, shown/edited in mph like
  // navSpeedMph -- backend default is 3.0 m/s (~6.7mph), converted at the
  // point of calling api.shadowSetMaxSpeed() since that's what the backend
  // expects.
  const [shadowMaxSpeedMph, setShadowMaxSpeedMph] = useState(
    () => Number(localStorage.getItem("shadowMaxSpeedMph")) || 6.7
  );

  useEffect(() => {
    localStorage.setItem("maxForward", maxForward);
  }, [maxForward]);
  useEffect(() => {
    localStorage.setItem("maxReverse", maxReverse);
  }, [maxReverse]);
  useEffect(() => {
    localStorage.setItem("navSpeedMph", navSpeedMph);
  }, [navSpeedMph]);
  useEffect(() => {
    localStorage.setItem("followDistanceM", followDistanceM);
  }, [followDistanceM]);
  useEffect(() => {
    localStorage.setItem("shadowMaxSpeedMph", shadowMaxSpeedMph);
  }, [shadowMaxSpeedMph]);

  // Poll status every 500ms. On a failed poll, merge in "disconnected"
  // rather than replacing the whole object -- otherwise a single missed
  // HTTP request (a harmless local network blip, unrelated to the actual
  // radio/Pi connection) wipes out the last known lat/lon along with
  // everything else, and the map suddenly blanks out to "Waiting for GPS
  // fix..." even though the vehicle's position hasn't actually changed.
  useEffect(() => {
    const id = setInterval(() => {
      api
        .status()
        .then(setStatus)
        .catch(() =>
          setStatus((prev) => ({ ...prev, connected: false, error: "lost connection to local server" }))
        );
    }, 500);
    return () => clearInterval(id);
  }, []);

  // Shadow Mode status/leader-telemetry poll -- same cadence as the main
  // status poll, same "merge on failure" reasoning (a missed request
  // shouldn't erase the leader marker from the map).
  useEffect(() => {
    const id = setInterval(() => {
      api
        .shadowStatus()
        .then(setShadow)
        .catch(() => setShadow((prev) => ({ ...prev, radio_ok: false, leader_ok: false, gps_ok: false })));
    }, 500);
    return () => clearInterval(id);
  }, []);

  const sendControl = useCallback(
    (steer, throttle) => {
      const scaled = throttle >= 0 ? throttle * maxForward : throttle * maxReverse;
      api.control(steer, scaled).catch(() => {});
    },
    [maxForward, maxReverse]
  );

  // Arrow-key control: held keys directly set steer/throttle, released keys
  // zero their axis immediately -- same "snap to neutral on release" as the
  // joystick. Only active while the control panel is open, so arrow keys
  // don't drive the car while just looking at the map. `manualAxes` mirrors
  // the raw -1..1 steer/throttle (before max-speed scaling) down to the
  // Joystick component so its handle visually moves with the keys too, not
  // just the actual motor output.
  const [manualAxes, setManualAxes] = useState({ steer: 0, throttle: 0 });
  useEffect(() => {
    if (!controlOpen) return;
    const held = new Set();
    const compute = () => {
      const steer = (held.has("ArrowRight") ? 1 : 0) - (held.has("ArrowLeft") ? 1 : 0);
      const throttle = (held.has("ArrowUp") ? 1 : 0) - (held.has("ArrowDown") ? 1 : 0);
      setManualAxes({ steer, throttle });
      sendControl(steer, throttle);
    };
    const onKeyDown = (e) => {
      if (!ARROW_KEYS.includes(e.key)) return;
      e.preventDefault();
      if (!held.has(e.key)) {
        held.add(e.key);
        compute();
      }
    };
    const onKeyUp = (e) => {
      if (!ARROW_KEYS.includes(e.key)) return;
      held.delete(e.key);
      compute();
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      setManualAxes({ steer: 0, throttle: 0 });
      sendControl(0, 0);
    };
  }, [controlOpen, sendControl]);

  // Dead-man's switch: if the tab loses focus mid-drive, or the control
  // panel closes, force neutral immediately.
  useEffect(() => {
    const onBlur = () => api.control(0, 0).catch(() => {});
    window.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) onBlur();
    });
    return () => window.removeEventListener("blur", onBlur);
  }, []);

  const armed = status.armed ?? false;
  const toggleArm = () => (armed ? api.disarm() : api.arm()).catch(() => {});

  const addWaypoint = (pt) => setPath((prev) => [...prev, pt]);
  const undoWaypoint = () => setPath((prev) => prev.slice(0, -1));
  const clearPath = () => setPath([]);

  const goPath = () => {
    if (path.length === 0) return;
    const speedMps = navSpeedMph / 2.23694;
    const waypoints = loop ? [...path, path[0]] : path;
    api.path(waypoints, speedMps).catch(() => {});
  };
  const stopNav = () => api.stopNav().catch(() => {});
  const goHome = () => {
    const speedMps = navSpeedMph / 2.23694;
    api.goHome(speedMps).catch(() => {});
  };

  // Enable/disable responses are just {ok, error?} -- not the full status
  // shape -- so don't setShadow() from them directly; the 500ms poll picks
  // up the new state right after.
  const enableShadow = () => api.shadowEnable().catch(() => {});
  const disableShadow = () => api.shadowDisable().catch(() => {});
  const setFollowDistance = (d) => {
    setFollowDistanceM(d);
    api.shadowSetDistance(d).catch(() => {});
  };
  const setShadowMaxSpeed = (mph) => {
    setShadowMaxSpeedMph(mph);
    api.shadowSetMaxSpeed(mph / 2.23694).catch(() => {});
  };

  // The one "stop everything now" action -- disarms, cancels navigation,
  // and disables Shadow Mode together, from wherever it's clicked.
  const emergencyStop = () => api.emergencyStop().catch(() => {});

  const hasFix =
    typeof status.lat === "number" &&
    typeof status.lon === "number" &&
    !(status.lat === 0 && status.lon === 0);

  // Autonomy (click-to-add-waypoints path following) is the default/home
  // mode -- Shadow
  // Mode and Manual driving are the two things that change it away from
  // that default, reflected here from the vehicle's *actual* reported
  // mode/Shadow state rather than just "which panel is open".
  const modeLabel = shadow.enabled ? "SHADOW" : status.mode_name === "MANUAL" ? "MANUAL" : "AUTONOMY";

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand">
          <h1>Ghost Planner</h1>
          <span className="brand-sub">MAARS LAB™ · developed by Srikar Chanamolu</span>
        </div>
        <div className="topbar-status">
          <span className={`dot ${status.connected ? "ok" : "bad"}`} />
          {status.connected ? "connected" : "not connected"}
          <span className={`mode-badge mode-${modeLabel.toLowerCase()}`}>{modeLabel}</span>
        </div>
        <button className={`arm-toggle-btn ${armed ? "armed" : ""}`} onClick={toggleArm}>
          {armed ? "DISARM" : "ARM"}
        </button>
        <button className="emergency-stop-btn" onClick={emergencyStop} title="Disarm + cancel navigation + disable Shadow Mode">
          🛑 STOP
        </button>
        <button className="manual-control-btn" onClick={() => setShadowOpen(true)}>
          🌓 Shadow Mode
        </button>
        <button className="manual-control-btn" onClick={() => setControlOpen(true)}>
          🎮 Manual Control
        </button>
      </header>

      <main className="map-area">
        {mapsApiKey ? (
          <RobotMap
            lat={status.lat}
            lon={status.lon}
            hasFix={hasFix}
            heading={status.heading_deg}
            path={path}
            loop={loop}
            leader={shadow.leader}
            onMapClick={addWaypoint}
            apiKey={mapsApiKey}
          />
        ) : mapsApiKey === null ? (
          <div className="map-fill map-placeholder">Loading map…</div>
        ) : (
          <ApiKeyForm onSave={saveMapsApiKey} />
        )}
        <NavControls
          path={path}
          loop={loop}
          setLoop={setLoop}
          speed={navSpeedMph}
          setSpeed={setNavSpeedMph}
          armed={armed}
          status={status}
          onGo={goPath}
          onStop={stopNav}
          onUndo={undoWaypoint}
          onClear={clearPath}
          onHome={goHome}
          leaderAvailable={shadow.leader_ok && shadow.gps_ok}
        />
      </main>

      {controlOpen && (
        <ControlPanel
          status={status}
          armed={armed}
          onToggleArm={toggleArm}
          onControlChange={sendControl}
          manualAxes={manualAxes}
          maxForward={maxForward}
          setMaxForward={setMaxForward}
          maxReverse={maxReverse}
          setMaxReverse={setMaxReverse}
          onClose={() => setControlOpen(false)}
        />
      )}

      {shadowOpen && (
        <ShadowPanel
          shadow={shadow}
          armed={armed}
          onToggleArm={toggleArm}
          onEnable={enableShadow}
          onDisable={disableShadow}
          distance={followDistanceM}
          setDistance={setFollowDistance}
          maxSpeed={shadowMaxSpeedMph}
          setMaxSpeed={setShadowMaxSpeed}
          onEmergencyStop={emergencyStop}
          onClose={() => setShadowOpen(false)}
        />
      )}
    </div>
  );
}
