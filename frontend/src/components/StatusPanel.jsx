export default function StatusPanel({ status }) {
  const connected = status.connected;
  return (
    <div className="status-panel">
      <div>
        <span className={`dot ${connected ? "ok" : "bad"}`} />
        {connected ? "connected (RF)" : "not connected"}
        {" | armed: "}
        {String(status.armed ?? false)}
        {" | mode: "}
        {status.mode_name ?? status.mode ?? "--"}
      </div>
      <div>
        steer: {status.steer_pwm ?? "--"} | throttle: {status.throttle_pwm ?? "--"}
      </div>
      <div>
        GPS fix: {status.gps_fix ?? "--"} sats: {status.gps_sats ?? "--"}
        {" | speed: "}
        {status.groundspeed_mps != null ? `${(status.groundspeed_mps * 2.23694).toFixed(1)} mph` : "--"}
        {" | heading: "}
        {status.heading_deg != null ? `${status.heading_deg.toFixed(0)}°` : "--"}
      </div>
      {status.error && <div className="error">{status.error}</div>}
    </div>
  );
}
