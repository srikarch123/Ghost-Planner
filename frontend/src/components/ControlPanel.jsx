import Joystick from "./Joystick";
import StatusPanel from "./StatusPanel";

export default function ControlPanel({
  status,
  armed,
  onToggleArm,
  onControlChange,
  manualAxes,
  maxForward,
  setMaxForward,
  maxReverse,
  setMaxReverse,
  onClose,
}) {
  return (
    <div className="overlay-backdrop" onClick={onClose}>
      <div className="control-panel" onClick={(e) => e.stopPropagation()}>
        <div className="control-panel-header">
          <h2>Manual Control</h2>
          <button className="close-btn" onClick={onClose} aria-label="Close">
            ×
          </button>
        </div>

        <StatusPanel status={status} />

        <Joystick onChange={onControlChange} externalAxes={manualAxes} />

        <div className="controls-row">
          <button className={`arm-btn ${armed ? "armed" : ""}`} onClick={onToggleArm}>
            {armed ? "DISARM" : "ARM"}
          </button>
        </div>

        <div className="sliders">
          <label>
            Max forward: {maxForward.toFixed(2)}
            <input
              type="range"
              min="0"
              max="1"
              step="0.05"
              value={maxForward}
              onChange={(e) => setMaxForward(Number(e.target.value))}
            />
          </label>
          <label>
            Max reverse: {maxReverse.toFixed(2)}
            <input
              type="range"
              min="0"
              max="1"
              step="0.05"
              value={maxReverse}
              onChange={(e) => setMaxReverse(Number(e.target.value))}
            />
          </label>
        </div>

        <p className="hint">Drag the joystick, or click the page and use arrow keys (←/→ steer, ↑/↓ throttle).</p>
      </div>
    </div>
  );
}
