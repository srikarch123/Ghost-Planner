export default function NavControls({
  path, loop, setLoop, speed, setSpeed, armed, status, onGo, onStop, onUndo, onClear,
  onHome, leaderAvailable,
}) {
  // Covers both kinds of one-shot/queued nav this panel can kick off: a
  // multi-waypoint path upload (path_status) and a plain single-target
  // goto() -- either a manual map click or "Home" -- which just sets
  // nav_target under GUIDED with no path_status at all.
  const navigating =
    status.path_status === "uploading" ||
    status.path_status === "active" ||
    (status.mode_name === "GUIDED" && Boolean(status.nav_target) && !status.path_status);
  const canGo = path.length > 0 && armed && !navigating;
  const canHome = armed && leaderAvailable && !navigating;

  return (
    <div className="nav-controls">
      <div className="nav-controls-title">Autonomy</div>
      <div className="nav-row">
        <label className="nav-speed">
          Speed
          <input
            type="number"
            min="0.5"
            max="11"
            step="0.5"
            value={speed}
            onChange={(e) => setSpeed(Number(e.target.value))}
          />
          mph
        </label>
        <button className="go-btn" onClick={onGo} disabled={!canGo}>
          GO
        </button>
        <button className="stop-nav-btn" onClick={onStop} disabled={!navigating}>
          STOP
        </button>
      </div>

      <div className="nav-row">
        <button
          className="home-btn"
          onClick={onHome}
          disabled={!canHome}
          title={leaderAvailable ? "Drive to the leader's current position" : "Leader Cube not connected / no GPS fix"}
        >
          🏠 Home
        </button>
      </div>

      <div className="nav-row">
        <label className="nav-loop">
          <input
            type="checkbox"
            checked={loop}
            onChange={(e) => setLoop(e.target.checked)}
            disabled={navigating}
          />
          Loop back to start
        </label>
        <button className="path-edit-btn" onClick={onUndo} disabled={path.length === 0 || navigating}>
          ↩ Undo
        </button>
        <button className="path-edit-btn" onClick={onClear} disabled={path.length === 0 || navigating}>
          ✕ Clear
        </button>
      </div>

      <div className="nav-status-row">
        {path.length > 0 ? (
          <span>
            {path.length} waypoint{path.length === 1 ? "" : "s"} queued{loop ? " (loop)" : ""}
          </span>
        ) : (
          <span>Click the map to add waypoints — add as many as you like</span>
        )}
        {!armed && <span className="nav-hint"> — arm the vehicle to drive</span>}
      </div>

      {navigating && (
        <div className="nav-status-row">
          {status.mode_name}
          {status.path_total ? ` · leg ${(status.path_index ?? 0) + 1}/${status.path_total}` : ""}
          {" · "}
          {status.nav_distance_m?.toFixed(1) ?? "--"} m remaining
        </div>
      )}
      {status.path_status && !navigating && status.path_status !== "complete" && (
        <div className="nav-status-row nav-hint">{status.path_status}</div>
      )}
      {status.path_status === "complete" && (
        <div className="nav-status-row">Path complete ✓</div>
      )}
    </div>
  );
}
