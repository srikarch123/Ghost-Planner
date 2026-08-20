function CheckRow({ ok, label }) {
  return (
    <div className="shadow-check">
      <span className={`dot ${ok ? "ok" : "bad"}`} />
      {label}
    </div>
  );
}

export default function ShadowPanel({
  shadow,
  armed,
  onToggleArm,
  onEnable,
  onDisable,
  distance,
  setDistance,
  maxSpeed,
  setMaxSpeed,
  onEmergencyStop,
  onClose,
}) {
  const allOk = shadow.radio_ok && shadow.leader_ok && shadow.gps_ok;
  const leader = shadow.leader || {};

  return (
    <div className="overlay-backdrop" onClick={onClose}>
      <div className="control-panel" onClick={(e) => e.stopPropagation()}>
        <div className="control-panel-header">
          <h2>Shadow Mode</h2>
          <button className="close-btn" onClick={onClose} aria-label="Close">
            ×
          </button>
        </div>

        <p className="hint">
          Follows the leader vehicle (this laptop + radio + Cube + GPS, driven by a
          person), matching its speed and stopping when it stops.
        </p>

        <div className="shadow-checklist">
          <CheckRow ok={shadow.radio_ok} label="Radio connected" />
          <CheckRow ok={shadow.leader_ok} label="Leader Cube connected" />
          <CheckRow ok={shadow.gps_ok} label="Leader GPS fix" />
        </div>

        <div className="nav-row">
          <label className="nav-speed">
            Follow distance
            <input
              type="number"
              min="1"
              max="50"
              step="0.5"
              value={distance}
              onChange={(e) => setDistance(Number(e.target.value))}
            />
            m
          </label>
          <button className={`arm-toggle-btn ${armed ? "armed" : ""}`} onClick={onToggleArm}>
            {armed ? "DISARM" : "ARM"}
          </button>
        </div>

        <div className="nav-row">
          <label className="nav-speed">
            Max speed
            <input
              type="number"
              min="1"
              max="13.4"
              step="0.5"
              value={maxSpeed}
              onChange={(e) => setMaxSpeed(Number(e.target.value))}
            />
            mph
          </label>
        </div>

        {shadow.enabled ? (
          <>
            <div className="status-panel">
              <div>Leader: {leader.lat?.toFixed(6) ?? "--"}, {leader.lon?.toFixed(6) ?? "--"}</div>
              <div>
                Speed: {leader.groundspeed_mps != null ? `${(leader.groundspeed_mps * 2.23694).toFixed(1)} mph` : "--"}
                {" | heading: "}
                {leader.heading_deg != null ? `${leader.heading_deg.toFixed(0)}°` : "--"}
              </div>
            </div>
            <div className="controls-row">
              <button className="stop-nav-btn" style={{ borderWidth: 2 }} onClick={onDisable}>
                DISABLE SHADOW MODE
              </button>
            </div>
          </>
        ) : (
          <div className="controls-row">
            <button className="go-btn" style={{ borderWidth: 2 }} onClick={onEnable} disabled={!allOk}>
              ENABLE SHADOW MODE
            </button>
          </div>
        )}

        {shadow.stopped_reason && (
          <div className="nav-hint">Stopped automatically: {shadow.stopped_reason}</div>
        )}

        <div className="controls-row">
          <button className="emergency-stop-btn" style={{ width: "100%" }} onClick={onEmergencyStop}>
            🛑 EMERGENCY STOP
          </button>
        </div>

        <p className="hint">
          Arm the vehicle (above, or via Manual Control) — enabling Shadow
          Mode by itself won&apos;t move it while disarmed. Touching the
          joystick always immediately regains manual control, but doesn&apos;t
          disable Shadow Mode itself — the next follow update (within a
          second) pulls it back into following unless you disable it here.
        </p>
      </div>
    </div>
  );
}
