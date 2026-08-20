import { useRef, useState, useCallback, useEffect } from "react";

const SIZE = 220;
const HANDLE = 56;
const RADIUS = (SIZE - HANDLE) / 2;

// Independent per-axis clamping, not vector normalization -- dragging to a
// corner gives full steer AND full throttle at once, matching how the ESC
// and steering servo are actually driven (two separate channels).
//
// `externalAxes` ({steer, throttle}, each -1..1) lets a non-pointer input
// source (arrow keys, handled up in App.jsx) drive the handle's visual
// position too -- without this, arrow-key driving worked (the actual
// steer/throttle commands went out fine) but the handle never moved,
// since it only ever updated `pos` from its own pointer event handlers.
// Only applied while NOT actively dragging, so it can't fight a live touch/
// mouse drag.
export default function Joystick({ onChange, externalAxes }) {
  const baseRef = useRef(null);
  const [pos, setPos] = useState({ x: 0, y: 0 }); // -1..1 each axis
  const draggingRef = useRef(false);

  useEffect(() => {
    if (draggingRef.current || !externalAxes) return;
    setPos({ x: externalAxes.steer, y: -externalAxes.throttle });
  }, [externalAxes]);

  const updateFromEvent = useCallback(
    (clientX, clientY) => {
      const base = baseRef.current;
      if (!base) return;
      const rect = base.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const dx = Math.max(-1, Math.min(1, (clientX - cx) / RADIUS));
      const dy = Math.max(-1, Math.min(1, (clientY - cy) / RADIUS));
      setPos({ x: dx, y: dy });
      onChange(dx, -dy); // screen-down is negative throttle
    },
    [onChange]
  );

  const release = useCallback(() => {
    draggingRef.current = false;
    setPos({ x: 0, y: 0 });
    onChange(0, 0);
  }, [onChange]);

  const onPointerDown = (e) => {
    draggingRef.current = true;
    e.target.setPointerCapture(e.pointerId);
    updateFromEvent(e.clientX, e.clientY);
  };
  const onPointerMove = (e) => {
    if (!draggingRef.current) return;
    updateFromEvent(e.clientX, e.clientY);
  };
  const onPointerUp = () => release();

  return (
    <div
      ref={baseRef}
      className="joystick-base"
      style={{ width: SIZE, height: SIZE }}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerUp}
    >
      <div
        className="joystick-handle"
        style={{
          width: HANDLE,
          height: HANDLE,
          transform: `translate(${pos.x * RADIUS}px, ${pos.y * RADIUS}px)`,
        }}
      />
    </div>
  );
}
