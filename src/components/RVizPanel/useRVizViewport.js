/**
 * RVizTDTU — Viewport Hook (Pan + Zoom)
 * 
 * Quản lý camera 2D cho canvas:
 *   - Mouse wheel → zoom (centered on cursor)
 *   - Drag → pan
 *   - worldToScreen / screenToWorld transforms
 * 
 * Coordinate system (giống RViz):
 *   World: X→right, Y→up (mét)
 *   Screen: X→right, Y→down (pixels)
 */

import { useState, useCallback, useRef } from 'react';

const MIN_SCALE = 10;   // 10 px/m (zoomed out max)
const MAX_SCALE = 400;  // 400 px/m (zoomed in max)
const DEFAULT_SCALE = 60; // 60 px/m (default)

function normalizeAngle(a) {
  let out = a;
  while (out > Math.PI) out -= Math.PI * 2;
  while (out < -Math.PI) out += Math.PI * 2;
  return out;
}

export function useRVizViewport(canvasWidth, canvasHeight, rotateCenterWorld = null) {
  // offset = center of viewport in world coords (meters)
  const [offset, setOffset] = useState({ x: 5, y: 5 }); // center of warehouse
  const [scale, setScale] = useState(DEFAULT_SCALE);     // pixels per meter
  const [rotation, setRotation] = useState(0);            // radians, screen-space rotation

  const isDragging = useRef(false);
  const dragMode = useRef(null); // 'pan' | 'rotate'
  const dragStart = useRef({ x: 0, y: 0 });
  const offsetStart = useRef({ x: 0, y: 0 });
  const rotationStart = useRef(0);
  const rotateAngleStart = useRef(0);
  const rotateCenterStart = useRef({ world: null, screen: null });

  // ── Coordinate Transforms ────────────────────────────────

  /**
   * World coords (meters) → Canvas pixels
   */
  const worldToScreen = useCallback((wx, wy) => {
    const ux = (wx - offset.x) * scale;
    const uy = -(wy - offset.y) * scale; // Y flipped
    const c = Math.cos(rotation);
    const s = Math.sin(rotation);
    const sx = ux * c - uy * s + canvasWidth / 2;
    const sy = ux * s + uy * c + canvasHeight / 2;
    return { x: sx, y: sy };
  }, [offset.x, offset.y, scale, rotation, canvasWidth, canvasHeight]);

  /**
   * Canvas pixels → World coords (meters)
   */
  const screenToWorld = useCallback((sx, sy) => {
    const dx = sx - canvasWidth / 2;
    const dy = sy - canvasHeight / 2;
    const c = Math.cos(rotation);
    const s = Math.sin(rotation);
    const ux = dx * c + dy * s;
    const uy = -dx * s + dy * c;
    const wx = ux / scale + offset.x;
    const wy = -uy / scale + offset.y; // Y flipped
    return { x: wx, y: wy };
  }, [offset.x, offset.y, scale, rotation, canvasWidth, canvasHeight]);

  // ── Mouse Handlers ───────────────────────────────────────

  const onWheel = useCallback((e) => {
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    const dx = mx - canvasWidth / 2;
    const dy = my - canvasHeight / 2;
    const c = Math.cos(rotation);
    const s = Math.sin(rotation);
    const ux = dx * c + dy * s;
    const uy = -dx * s + dy * c;

    // World pos under cursor before zoom
    const worldBefore = {
      x: ux / scale + offset.x,
      y: -uy / scale + offset.y,
    };

    // Zoom
    const factor = e.deltaY < 0 ? 1.12 : 0.89;
    const newScale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, scale * factor));

    // World pos under cursor after zoom (should stay same)
    const newOffsetX = worldBefore.x - ux / newScale;
    const newOffsetY = worldBefore.y + uy / newScale;

    setScale(newScale);
    setOffset({ x: newOffsetX, y: newOffsetY });
  }, [scale, offset, rotation, canvasWidth, canvasHeight]);

  const onMouseDown = useCallback((e) => {
    if (e.button !== 0 && e.button !== 1 && e.button !== 2) return;
    e.preventDefault();
    isDragging.current = true;
    dragMode.current = e.button === 2 ? 'rotate' : 'pan';
    dragStart.current = { x: e.clientX, y: e.clientY };
    offsetStart.current = { ...offset };
    rotationStart.current = rotation;

    if (dragMode.current === 'rotate') {
      const rect = e.currentTarget.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const centerWorld = (
        rotateCenterWorld &&
        Number.isFinite(rotateCenterWorld.x) &&
        Number.isFinite(rotateCenterWorld.y)
      ) ? rotateCenterWorld : offset;
      const centerScreen = worldToScreen(centerWorld.x, centerWorld.y);

      rotateCenterStart.current = {
        world: { x: centerWorld.x, y: centerWorld.y },
        screen: centerScreen,
      };
      rotateAngleStart.current = Math.atan2(my - centerScreen.y, mx - centerScreen.x);
    }

    e.currentTarget.style.cursor = dragMode.current === 'rotate' ? 'ew-resize' : 'grabbing';
  }, [offset, rotation, rotateCenterWorld, worldToScreen]);

  const onMouseMove = useCallback((e) => {
    if (!isDragging.current) return;
    const dx = e.clientX - dragStart.current.x;
    const dy = e.clientY - dragStart.current.y;

    if (dragMode.current === 'rotate') {
      const center = rotateCenterStart.current;
      if (!center.world || !center.screen) return;

      const rect = e.currentTarget.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const angle = Math.atan2(my - center.screen.y, mx - center.screen.x);
      const newRotation = rotationStart.current + normalizeAngle(angle - rotateAngleStart.current);
      const c = Math.cos(newRotation);
      const s = Math.sin(newRotation);
      const screenDx = center.screen.x - canvasWidth / 2;
      const screenDy = center.screen.y - canvasHeight / 2;
      const ux = screenDx * c + screenDy * s;
      const uy = -screenDx * s + screenDy * c;

      setRotation(newRotation);
      setOffset({
        x: center.world.x - ux / scale,
        y: center.world.y + uy / scale,
      });
      return;
    }

    const c = Math.cos(rotation);
    const s = Math.sin(rotation);
    const ux = dx * c + dy * s;
    const uy = -dx * s + dy * c;
    setOffset({
      x: offsetStart.current.x - ux / scale,
      y: offsetStart.current.y + uy / scale, // Y flipped
    });
  }, [scale, rotation, canvasWidth, canvasHeight]);

  const onMouseUp = useCallback((e) => {
    isDragging.current = false;
    dragMode.current = null;
    e.currentTarget.style.cursor = 'crosshair';
  }, []);

  const onMouseLeave = useCallback((e) => {
    isDragging.current = false;
    dragMode.current = null;
    e.currentTarget.style.cursor = 'crosshair';
  }, []);

  // ── API ──────────────────────────────────────────────────

  /** Center viewport on world coords */
  const centerOn = useCallback((wx, wy) => {
    setOffset({ x: wx, y: wy });
  }, []);

  /** Reset zoom to default */
  const resetZoom = useCallback(() => {
    setScale(DEFAULT_SCALE);
    setOffset({ x: 5, y: 5 });
    setRotation(0);
  }, []);

  /** Follow robot (smooth center) */
  const followRobot = useCallback((rx, ry) => {
    setOffset(prev => ({
      x: prev.x + (rx - prev.x) * 0.1,
      y: prev.y + (ry - prev.y) * 0.1,
    }));
  }, []);

  return {
    // State
    offset,
    scale,
    rotation,

    // Transforms
    worldToScreen,
    screenToWorld,

    // Mouse handlers (attach to canvas)
    handlers: {
      onWheel,
      onMouseDown,
      onMouseMove,
      onMouseUp,
      onMouseLeave,
    },

    // Actions
    centerOn,
    resetZoom,
    followRobot,
    setScale,
    setRotation,
  };
}

export default useRVizViewport;
