const BASE = import.meta.env.VITE_BACKEND_URL || "http://localhost:5050";

async function post(path, body) {
  const res = await fetch(`${BASE}${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body !== undefined ? JSON.stringify(body) : undefined,
  });
  return res.json();
}

export const api = {
  control: (steer, throttle) => post("/control", { steer, throttle }),
  arm: () => post("/arm"),
  disarm: () => post("/disarm"),
  status: () => fetch(`${BASE}/status`).then((r) => r.json()),
  goto: (lat, lon, speed) => post("/goto", speed != null ? { lat, lon, speed } : { lat, lon }),
  path: (waypoints, speed) => post("/path", speed != null ? { waypoints, speed } : { waypoints }),
  goHome: (speed) => post("/go_home", speed != null ? { speed } : {}),
  stopNav: () => post("/stop_nav"),
  shadowStatus: () => fetch(`${BASE}/shadow/status`).then((r) => r.json()),
  shadowEnable: () => post("/shadow/enable"),
  shadowDisable: () => post("/shadow/disable"),
  shadowSetDistance: (distanceM) => post("/shadow/distance", { distance_m: distanceM }),
  shadowSetMaxSpeed: (speedMps) => post("/shadow/max_speed", { max_speed_mps: speedMps }),
  emergencyStop: () => post("/emergency_stop"),
  getConfig: () => fetch(`${BASE}/api/config`).then((r) => r.json()),
  setGoogleMapsApiKey: (key) => post("/api/config", { google_maps_api_key: key }),
};
