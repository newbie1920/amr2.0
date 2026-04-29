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

export function useRVizViewport(canvasWidth, canvasHeight) {
  // offset = center of viewport in world coords (meters)
  const [offset, setOffset] = useState({ x: 5, y: 5 }); // center of warehouse
  const [scale, setScale] = useState(DEFAULT_SCALE);     // pixels per meter

  const isDragging = useRef(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const offsetStart = useRef({ x: 0, y: 0 });

  // ── Coordinate Transforms ────────────────────────────────

  /**
   * World coords (meters) → Canvas pixels
   */
  const worldToScreen = useCallback((wx, wy) => {
    const sx = (wx - offset.x) * scale + canvasWidth / 2;
    const sy = -(wy - offset.y) * scale + canvasHeight / 2; // Y flipped
    return { x: sx, y: sy };
  }, [offset.x, offset.y, scale, canvasWidth, canvasHeight]);

  /**
   * Canvas pixels → World coords (meters)
   */
  const screenToWorld = useCallback((sx, sy) => {
    const wx = (sx - canvasWidth / 2) / scale + offset.x;
    const wy = -(sy - canvasHeight / 2) / scale + offset.y; // Y flipped
    return { x: wx, y: wy };
  }, [offset.x, offset.y, scale, canvasWidth, canvasHeight]);

  // ── Mouse Handlers ───────────────────────────────────────

  const onWheel = useCallback((e) => {
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    // World pos under cursor before zoom
    const worldBefore = {
      x: (mx - canvasWidth / 2) / scale + offset.x,
      y: -(my - canvasHeight / 2) / scale + offset.y,
    };

    // Zoom
    const factor = e.deltaY < 0 ? 1.12 : 0.89;
    const newScale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, scale * factor));

    // World pos under cursor after zoom (should stay same)
    const newOffsetX = worldBefore.x - (mx - canvasWidth / 2) / newScale;
    const newOffsetY = worldBefore.y + (my - canvasHeight / 2) / newScale;

    setScale(newScale);
    setOffset({ x: newOffsetX, y: newOffsetY });
  }, [scale, offset, canvasWidth, canvasHeight]);

  const onMouseDown = useCallback((e) => {
    if (e.button !== 0 && e.button !== 1) return; // Left or middle click
    isDragging.current = true;
    dragStart.current = { x: e.clientX, y: e.clientY };
    offsetStart.current = { ...offset };
    e.currentTarget.style.cursor = 'grabbing';
  }, [offset]);

  const onMouseMove = useCallback((e) => {
    if (!isDragging.current) return;
    const dx = e.clientX - dragStart.current.x;
    const dy = e.clientY - dragStart.current.y;
    setOffset({
      x: offsetStart.current.x - dx / scale,
      y: offsetStart.current.y + dy / scale, // Y flipped
    });
  }, [scale]);

  const onMouseUp = useCallback((e) => {
    isDragging.current = false;
    e.currentTarget.style.cursor = 'crosshair';
  }, []);

  const onMouseLeave = useCallback((e) => {
    isDragging.current = false;
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
  };
}

export default useRVizViewport;
