import { useCallback, useEffect, useRef, useState } from "react";
import { useJsApiLoader, GoogleMap } from "@react-google-maps/api";

const CONTAINER_STYLE = { width: "100%", height: "100%" };
const MAP_OPTIONS = {
  streetViewControl: false,
  mapTypeControl: false,
  fullscreenControl: false,
  mapTypeId: "hybrid", // satellite imagery + road/place labels
};
const DEFAULT_ZOOM = 19;

// Leader: a plain directional arrow -- it's a person-driven vehicle just
// passing through the map, the arrow alone reads fine.
function arrowIcon(headingDeg, color) {
  return {
    path: window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
    scale: 5,
    rotation: headingDeg, // degrees clockwise from north, matches compass heading directly
    fillColor: color,
    fillOpacity: 1,
    strokeColor: "#ffffff",
    strokeWeight: 1,
  };
}
// Robot (follower): a distinct, sportier top-down car silhouette (body +
// windshield + a couple of detail lines), not just another arrow -- the
// robot and leader markers were too easy to confuse at a glance when both
// were the same plain triangle, only color different. This is a real SVG
// image (data URI), not a Maps `Symbol` path -- Symbols only support one
// flat fill, no windshield/detail layering -- so rotation has to be baked
// into the SVG itself (a `rotate()` transform around its center) rather
// than using Symbol's `rotation` property, which only works for Symbols.
function carIcon(headingDeg, color) {
  const heading = typeof headingDeg === "number" ? headingDeg : 0;
  const svg = `
    <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">
      <g transform="rotate(${heading} 24 24)">
        <path d="M24 3 C27 3 29 8 30.5 12 C33.5 17 35 23 35 28 L35 37 C35 41 31.5 43 27 43
                 L21 43 C16.5 43 13 41 13 37 L13 28 C13 23 14.5 17 17.5 12 C19 8 21 3 24 3 Z"
              fill="${color}" stroke="#ffffff" stroke-width="1.6" stroke-linejoin="round"/>
        <path d="M18.5 14 L29.5 14 C30.5 17 31.5 20.5 31.5 23.5 L16.5 23.5 C16.5 20.5 17.5 17 18.5 14 Z"
              fill="#0d1117" opacity="0.65"/>
        <rect x="14.5" y="27" width="19" height="3" rx="1.5" fill="#0d1117" opacity="0.35"/>
        <rect x="14.5" y="33" width="19" height="3" rx="1.5" fill="#0d1117" opacity="0.35"/>
      </g>
    </svg>`;
  return {
    url: `data:image/svg+xml;charset=UTF-8,${encodeURIComponent(svg)}`,
    scaledSize: new window.google.maps.Size(48, 48),
    anchor: new window.google.maps.Point(24, 24),
  };
}
function dotIcon(color) {
  return { path: window.google.maps.SymbolPath.CIRCLE, scale: 6, fillColor: color, fillOpacity: 1, strokeColor: "#ffffff", strokeWeight: 2 };
}

const ROBOT_COLOR = "#58a6ff";
const LEADER_COLOR = "#3fb950";

// Shown by App.jsx INSTEAD of <RobotMap> (not from within it) when the
// backend has no Google Maps API key configured yet -- lets whoever's
// running this (e.g. a packaged .exe with no build-time key baked in)
// supply their own without a rebuild. Saves via POST /api/config,
// persisted to a local file next to the exe/server.py so it's remembered
// on the next launch too.
//
// Deliberately NOT rendered from inside RobotMap based on a prop: the
// Google Maps loader (useJsApiLoader below, wrapping @googlemaps/js-api-
// loader's Loader) is a page-wide singleton that throws ("Loader must not
// be called again with different options") if it's ever invoked twice
// with a different API key in the same page session. Since apiKey starts
// out unknown while GET /api/config is in flight, calling the hook
// unconditionally inside RobotMap with a placeholder value first and the
// real key moments later hit exactly that crash -- every time, not just
// when a user actively re-enters a key. Fix: App.jsx doesn't mount
// <RobotMap> (and so never calls this hook) until it already has the
// confirmed real key in hand, so the hook only ever runs once, already
// correct.
export function ApiKeyForm({ onSave }) {
  const [value, setValue] = useState("");
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState(null);

  const submit = (e) => {
    e.preventDefault();
    if (!value.trim() || !onSave) return;
    setSaving(true);
    setError(null);
    onSave(value.trim())
      .catch(() => setError("Couldn't save the key -- check the backend is running."))
      .finally(() => setSaving(false));
  };

  return (
    <div className="map-fill map-placeholder">
      <form className="api-key-form" onSubmit={submit}>
        <p>Enter a Google Maps API key to enable the map.</p>
        <div className="api-key-row">
          <input
            type="text"
            value={value}
            onChange={(e) => setValue(e.target.value)}
            placeholder="AIza..."
            autoFocus
          />
          <button type="submit" disabled={saving || !value.trim()}>
            {saving ? "Saving…" : "Save"}
          </button>
        </div>
        {error && <p className="error">{error}</p>}
      </form>
    </div>
  );
}

export default function RobotMap({
  lat, lon, hasFix, heading, path, loop, leader, onMapClick, apiKey,
}) {
  // `apiKey` is guaranteed non-empty here -- App.jsx only mounts this
  // component once it has a confirmed real key (see the ApiKeyForm comment
  // above for why that matters).
  const { isLoaded, loadError } = useJsApiLoader({
    id: "followbot-google-map",
    googleMapsApiKey: apiKey,
  });

  // Center is only ever set ONCE, on the first fix -- not recomputed as a
  // fresh object every render (every 500ms status poll). Passing a new
  // object as `center` on every render made the map snap back to the
  // robot's position every half-second, fighting any manual pan/zoom.
  const initialCenterRef = useRef(null);
  if (hasFix && !initialCenterRef.current) {
    initialCenterRef.current = { lat, lng: lon };
  }

  const mapRef = useRef(null);
  const [mapReady, setMapReady] = useState(false);
  const handleLoad = useCallback((map) => {
    mapRef.current = map;
    setMapReady(true);
  }, []);

  const recenter = useCallback(() => {
    if (mapRef.current && hasFix) {
      mapRef.current.panTo({ lat, lng: lon });
      mapRef.current.setZoom(DEFAULT_ZOOM);
    }
  }, [lat, lon, hasFix]);

  // Robot and waypoint markers are managed IMPERATIVELY with the raw Maps
  // JS API (plain `new google.maps.Marker(...)`, updated via `.setPosition`)
  // instead of react-google-maps's <Marker> JSX component, which was
  // unreliable across this app's frequent (500ms poll-driven) re-renders.
  //
  // Creation and position-updates are deliberately in SEPARATE effects.
  // React 18 StrictMode runs every effect's mount->cleanup->mount cycle
  // once in development to catch exactly this class of bug: an earlier
  // version created the marker in the same effect that also updated its
  // position on every lat/lon change, with one shared cleanup effect at
  // the bottom of the component. StrictMode's simulated unmount ran that
  // cleanup (removing the marker from the map) but nothing cleared the
  // ref, so on remount the "does it already exist" check saw a non-null
  // (but now map-less) ref and just called .setPosition() on an already-
  // detached marker forever -- it would never get re-attached, so it
  // silently stayed invisible from then on. Splitting into a create/
  // destroy effect (deps: [mapReady] only, ref properly nulled in its own
  // cleanup) and a separate position-sync effect (deps: [hasFix, lat, lon],
  // no creation/cleanup of its own) means the ref is always correctly
  // null right when a fresh marker actually needs creating.
  const robotMarkerRef = useRef(null);
  useEffect(() => {
    if (!mapReady) return;
    const marker = new window.google.maps.Marker({
      map: mapRef.current,
      title: "Robot",
    });
    robotMarkerRef.current = marker;
    return () => {
      marker.setMap(null);
      robotMarkerRef.current = null;
    };
  }, [mapReady]);
  useEffect(() => {
    if (!hasFix || !robotMarkerRef.current) return;
    robotMarkerRef.current.setPosition({ lat, lng: lon });
    // Heading unavailable yet (null) -> plain dot rather than an arrow
    // pointing an arbitrary/stale direction.
    robotMarkerRef.current.setIcon(
      typeof heading === "number" ? carIcon(heading, ROBOT_COLOR) : dotIcon(ROBOT_COLOR)
    );
  }, [hasFix, lat, lon, heading]);

  // Leader marker (Shadow Mode): same split create/position-sync pattern
  // as the robot marker, since it updates on the same polling cadence.
  // Only exists while `leader` has a valid position.
  const leaderMarkerRef = useRef(null);
  const leaderHasFix = leader && typeof leader.lat === "number" && typeof leader.lon === "number";
  useEffect(() => {
    if (!mapReady) return;
    const marker = new window.google.maps.Marker({ title: "Leader" });
    leaderMarkerRef.current = marker;
    return () => {
      marker.setMap(null);
      leaderMarkerRef.current = null;
    };
  }, [mapReady]);
  useEffect(() => {
    if (!leaderMarkerRef.current) return;
    if (!leaderHasFix) {
      leaderMarkerRef.current.setMap(null);
      return;
    }
    leaderMarkerRef.current.setMap(mapRef.current);
    leaderMarkerRef.current.setPosition({ lat: leader.lat, lng: leader.lon });
    leaderMarkerRef.current.setIcon(
      typeof leader.heading_deg === "number" ? arrowIcon(leader.heading_deg, LEADER_COLOR) : dotIcon(LEADER_COLOR)
    );
  }, [leaderHasFix, leader?.lat, leader?.lon, leader?.heading_deg]);

  // Waypoint markers: the whole set is recreated whenever `path` itself
  // changes (a click added a point, or Undo/Clear ran) -- infrequent/
  // user-driven (unlike the robot marker's 500ms poll churn), and path
  // arrays here are always small, so just tearing down and rebuilding all
  // of them per edit is simpler than diffing. Numbered labels show drive
  // order.
  const waypointMarkersRef = useRef([]);
  useEffect(() => {
    if (!mapReady) return;
    waypointMarkersRef.current = (path || []).map(
      (wp, i) =>
        new window.google.maps.Marker({
          map: mapRef.current,
          position: { lat: wp.lat, lng: wp.lon },
          label: String(i + 1),
          title: `Waypoint ${i + 1}`,
        })
    );
    return () => {
      waypointMarkersRef.current.forEach((m) => m.setMap(null));
      waypointMarkersRef.current = [];
    };
  }, [mapReady, path]);

  // Approximate path: a dashed line from the robot's live position through
  // every waypoint in order (and back to the first if looping) -- just
  // straight segments, not the vehicle's actual planned trajectory (that's
  // ArduPilot's internally, not something exposed to recompute here), but
  // enough to show roughly where it's headed. Same "recreate whenever path
  // changes" pattern as the markers, so editing the path cleanly clears
  // the old line; a separate position-sync effect keeps the robot-side
  // endpoint live as it moves.
  const pathLineRef = useRef(null);
  useEffect(() => {
    if (!mapReady || !path || path.length === 0) return;
    const line = new window.google.maps.Polyline({
      map: mapRef.current,
      strokeOpacity: 0,
      icons: [
        {
          icon: { path: "M 0,-1 0,1", strokeOpacity: 1, scale: 3, strokeColor: ROBOT_COLOR },
          offset: "0",
          repeat: "14px",
        },
      ],
    });
    pathLineRef.current = line;
    return () => {
      line.setMap(null);
      pathLineRef.current = null;
    };
  }, [mapReady, path]);
  useEffect(() => {
    if (!pathLineRef.current || !hasFix || !path || path.length === 0) return;
    const points = [{ lat, lng: lon }, ...path.map((wp) => ({ lat: wp.lat, lng: wp.lon }))];
    if (loop) points.push({ lat: path[0].lat, lng: path[0].lon });
    pathLineRef.current.setPath(points);
  }, [hasFix, lat, lon, path, loop]);

  if (loadError) {
    return <div className="map-fill map-placeholder error">Failed to load Google Maps.</div>;
  }
  if (!isLoaded) {
    return <div className="map-fill map-placeholder">Loading map…</div>;
  }
  // Gated on "have we EVER had a fix" (initialCenterRef, set once and never
  // cleared), NOT the instantaneous `hasFix` -- that used to unmount
  // <GoogleMap> (destroying the real google.maps.Map instance) on every
  // transient blip where a status poll came back without lat/lon (e.g. a
  // backend restart's first response, before it's re-populated from the
  // Pi). Remounting later brought back a *new* map instance, but
  // `mapReady` state doesn't reset on unmount, so it stayed `true` and the
  // marker-creation effects above (keyed on `[mapReady]`) never re-fired
  // for the new map -- leaving the robot marker permanently pointing at a
  // destroyed map object, invisible from then on. Keeping the map mounted
  // through blips avoids the whole class of bug; markers just stop
  // updating (not disappearing) while `hasFix` is momentarily false.
  if (!initialCenterRef.current) {
    return <div className="map-fill map-placeholder">Waiting for GPS fix…</div>;
  }

  const handleClick = (e) => {
    onMapClick?.({ lat: e.latLng.lat(), lon: e.latLng.lng() });
  };

  return (
    <div className="map-fill">
      <GoogleMap
        mapContainerStyle={CONTAINER_STYLE}
        center={initialCenterRef.current}
        zoom={DEFAULT_ZOOM}
        options={MAP_OPTIONS}
        onClick={handleClick}
        onLoad={handleLoad}
      />
      <button className="recenter-btn" onClick={recenter} title="Recenter on robot">
        🎯
      </button>
    </div>
  );
}
