/**
 * RVizTDTU — Main Debug Visualization Panel
 * 
 * Canvas 2D top-down view giống RViz trong ROS2.
 * Hiển thị: Grid, Occupancy Map, Laser Scan, Robot Pose,
 *           Path, Costmap, TF Frames, Frontiers, World Segments
 * 
 * Features:
 *   - Pan/Zoom (mouse wheel + drag)
 *   - Layer toggle
 *   - Tool modes: Move, Pose Estimate, Nav Goal, Measure
 *   - Follow Robot mode
 *   - Topic Inspector
 */

import React, { useRef, useEffect, useState, useCallback } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import { useRVizViewport } from './useRVizViewport.js';
import RVizToolbar from './RVizToolbar.jsx';
import TopicInspector from './TopicInspector.jsx';
import { navWorkerApi } from '../../core/navWorkerSetup.js';
import {
  drawGrid,
  drawOccupancyMap,
  drawCostmap,
  drawCostmapNav2,
  drawWorldSegments,
  drawPath,
  drawLaserScan,
  drawRobotPose,
  drawTFFrames,
  drawFrontiers,
  drawMeasureLine,
  drawGoalMarker,
  drawNavStatus,
} from './rvizLayers.js';

// ============================================================
//   STYLES
// ============================================================

const panelStyle = {
  display: 'flex',
  flexDirection: 'column',
  height: '100%',
  width: '100%',
  flex: 1,
  background: '#0c1219',
  overflow: 'hidden',
  position: 'relative',
};

const headerStyle = {
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  padding: '4px 10px',
  background: 'rgba(15, 25, 35, 0.95)',
  borderBottom: '1px solid rgba(16, 185, 129, 0.2)',
  minHeight: '28px',
  flexShrink: 0,
};

const titleStyle = {
  fontSize: '11px',
  fontWeight: 700,
  color: '#10b981',
  letterSpacing: '0.5px',
};

const headerBtnStyle = (active) => ({
  padding: '2px 8px',
  fontSize: '10px',
  background: active ? 'rgba(16, 185, 129, 0.2)' : 'transparent',
  border: active ? '1px solid rgba(16, 185, 129, 0.4)' : '1px solid rgba(255,255,255,0.08)',
  borderRadius: '3px',
  color: active ? '#4ade80' : '#64748b',
  cursor: 'pointer',
  marginLeft: '4px',
  transition: 'all 0.12s ease',
});

const canvasContainerStyle = {
  flex: 1,
  display: 'flex',
  position: 'relative',
  overflow: 'hidden',
};

const canvasStyle = {
  flex: 1,
  cursor: 'crosshair',
};

const statusBarStyle = {
  display: 'flex',
  alignItems: 'center',
  gap: '12px',
  padding: '2px 10px',
  background: 'rgba(15, 25, 35, 0.95)',
  borderTop: '1px solid rgba(16, 185, 129, 0.1)',
  fontSize: '9px',
  color: '#475569',
  fontFamily: "'JetBrains Mono', monospace",
  flexShrink: 0,
};

// ============================================================
//   COMPONENT
// ============================================================

export default function RVizPanel({ activePath }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const animFrameRef = useRef(null);
  const [canvasSize, setCanvasSize] = useState({ w: 400, h: 300 });
  const [activeTool, setActiveTool] = useState('move');
  const [followRobot, setFollowRobot] = useState(false);
  const [cursorWorld, setCursorWorld] = useState({ x: 0, y: 0 });
  const [measurePoints, setMeasurePoints] = useState({ p1: null, p2: null });
  const [goalMarker, setGoalMarker] = useState(null); // {x, y} world coords
  const [navPath, setNavPath] = useState(null); // planned path [{x,y}, ...]

  // Layer visibility
  const [layers, setLayers] = useState({
    grid: true, map: true, costmap: false, walls: true,
    laser: true, path: true, robot: true, tf: false, frontier: true,
  });

  const toggleLayer = useCallback((id) => {
    setLayers(prev => ({ ...prev, [id]: !prev[id] }));
  }, []);

  // Store data
  const robots = useRobotStore((s) => s.robots);
  const occupancyGrid = useRobotStore((s) => s.occupancyGrid);
  const mapperInstances = useRobotStore((s) => s.mapperInstances);
  const mapToOdom = useRobotStore((s) => s.mapToOdom);
  const selectedRobotId = useRobotStore((s) => s.selectedRobotId);
  const simWorldSegments = useRobotStore((s) => s.simWorldSegments);
  const simInfo = useRobotStore((s) => s.simInfo);
  const appNavigationSessions = useRobotStore((s) => s.appNavigationSessions);
  const startAppNavigation = useRobotStore((s) => s.startAppNavigation);
  const stopAppNavigation = useRobotStore((s) => s.stopAppNavigation);

  // Find active robot
  const robotList = Object.values(robots);
  const activeRobot = selectedRobotId
    ? robots[selectedRobotId]
    : robotList.find(r => r.status === 'connected');
  const activeRobotId = activeRobot?.id;
  const telem = activeRobot?.telemetry || {};
  const grid = activeRobotId ? occupancyGrid[activeRobotId] : null;
  const tf = activeRobotId ? (mapToOdom[activeRobotId] || { dx: 0, dy: 0, dTheta: 0 }) : { dx: 0, dy: 0, dTheta: 0 };
  const navSession = activeRobotId ? appNavigationSessions[activeRobotId] : null;
  const firstSimInfo = Object.values(simInfo)[0];

  // Viewport
  const viewport = useRVizViewport(canvasSize.w, canvasSize.h);

  // ── Canvas Resize ─────────────────────────────────────────

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const ro = new ResizeObserver(entries => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width > 0 && height > 0) {
          setCanvasSize({ w: Math.floor(width), h: Math.floor(height) });
        }
      }
    });
    ro.observe(container);
    return () => ro.disconnect();
  }, []);

  // ── Wheel Event (Non-Passive) ─────────────────────────────
  
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleWheel = (e) => {
      viewport.handlers.onWheel(e);
    };

    // Attach non-passive listener to allow preventDefault
    canvas.addEventListener('wheel', handleWheel, { passive: false });
    return () => {
      canvas.removeEventListener('wheel', handleWheel);
    };
  }, [viewport.handlers]);

  // ── Follow Robot ──────────────────────────────────────────

  useEffect(() => {
    if (followRobot && telem.x !== undefined) {
      viewport.followRobot(telem.x, telem.y);
    }
  }, [followRobot, telem.x, telem.y]);

  // ── Canvas Mouse Handlers ─────────────────────────────────

  const handleCanvasMouseMove = useCallback((e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    setCursorWorld(viewport.screenToWorld(sx, sy));

    // Delegate to viewport pan handler
    viewport.handlers.onMouseMove(e);
  }, [viewport]);

  const handleCanvasClick = useCallback((e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const world = viewport.screenToWorld(sx, sy);

    if (activeTool === 'goal') {
      if (!activeRobotId) {
        console.warn('[RVizTDTU] No robot selected for navigation');
        return;
      }

      // Set goal marker immediately for visual feedback
      setGoalMarker({ x: world.x, y: world.y });
      console.log(`[RVizTDTU] 🎯 Nav Goal: (${world.x.toFixed(2)}, ${world.y.toFixed(2)})`);

      // Get grid (mapper instance or occupancy grid)
      const grid = mapperInstances[activeRobotId] || (occupancyGrid[activeRobotId] ? occupancyGrid[activeRobotId] : null);

      if (!grid) {
        console.warn('[RVizTDTU] No map available — sending direct waypoint');
        // Fallback: send single waypoint directly
        const path = [{ x: telem.x ?? 0, y: telem.y ?? 0 }, { x: world.x, y: world.y }];
        setNavPath(path);
        startAppNavigation(activeRobotId, path);
        return;
      }

      // Run A* pathfinder on the grid (via Web Worker if available)
      const startX = telem.x ?? 0;
      const startY = telem.y ?? 0;

      if (navWorkerApi) {
        // Async: use Web Worker for non-blocking pathfinding
        navWorkerApi.findPath(grid.serialize(), startX, startY, world.x, world.y, false, true)
          .then((result) => {
            if (result.success && result.path.length > 1) {
              console.log(`[RVizTDTU] ✅ Path found: ${result.path.length} waypoints`);
              setNavPath(result.path);
              startAppNavigation(activeRobotId, result.path);
            } else {
              console.warn('[RVizTDTU] ❌ No path found to goal');
              setGoalMarker(null);
              setNavPath(null);
            }
          })
          .catch((err) => {
            console.error('[RVizTDTU] Path error:', err);
            setGoalMarker(null);
            setNavPath(null);
          });
      } else {
        // Sync fallback: import and run directly
        import('../../core/lidarPathfinder.js').then(({ findPathOnGrid }) => {
          const result = findPathOnGrid(grid, startX, startY, world.x, world.y);
          if (result.success && result.path.length > 1) {
            console.log(`[RVizTDTU] ✅ Path found: ${result.path.length} waypoints`);
            setNavPath(result.path);
            startAppNavigation(activeRobotId, result.path);
          } else {
            console.warn('[RVizTDTU] ❌ No path found to goal');
            setGoalMarker(null);
            setNavPath(null);
          }
        });
      }
    } else if (activeTool === 'pose') {
      if (activeRobotId) {
        const robot = robots[activeRobotId];
        if (robot?.connection) {
          robot.connection.setPose(world.x, world.y, (telem.heading ?? 0) * Math.PI / 180);
          console.log(`[RVizTDTU] 📍 Pose set: (${world.x.toFixed(2)}, ${world.y.toFixed(2)})`);
        }
      }
    } else if (activeTool === 'measure') {
      if (!measurePoints.p1) {
        setMeasurePoints({ p1: world, p2: null });
      } else if (!measurePoints.p2) {
        setMeasurePoints(prev => ({ ...prev, p2: world }));
      } else {
        setMeasurePoints({ p1: world, p2: null });
      }
    }
  }, [activeTool, measurePoints, viewport, activeRobotId, telem, mapperInstances, occupancyGrid, robots, startAppNavigation]);

  // ── Render Loop ───────────────────────────────────────────

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const render = () => {
      const { w, h } = canvasSize;
      canvas.width = w * window.devicePixelRatio;
      canvas.height = h * window.devicePixelRatio;
      ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

      // Clear
      ctx.fillStyle = '#0c1219';
      ctx.fillRect(0, 0, w, h);

      const vp = {
        worldToScreen: viewport.worldToScreen,
        screenToWorld: viewport.screenToWorld,
        scale: viewport.scale,
      };

      // Draw layers in order (back to front)
      if (layers.grid) drawGrid(ctx, w, h, vp);
      if (layers.walls) drawWorldSegments(ctx, w, h, vp, simWorldSegments);
      if (layers.map && grid) drawOccupancyMap(ctx, w, h, vp, grid);
      if (layers.costmap && grid) drawCostmapNav2(ctx, w, h, vp, grid);
      if (layers.frontier && grid && grid.frontierCells) drawFrontiers(ctx, w, h, vp, grid.frontierCells, grid);

      // Navigation path (from click-to-navigate or active nav session)
      const displayPath = navSession?.path || navPath || activePath;
      if (layers.path && displayPath) drawPath(ctx, w, h, vp, displayPath);

      // Robot-specific layers
      if (telem.x !== undefined) {
        const headingRad = (telem.heading ?? 0) * Math.PI / 180;
        if (layers.laser) drawLaserScan(ctx, w, h, vp, telem.x, telem.y, headingRad, telem.lidar);
        if (layers.robot) drawRobotPose(ctx, w, h, vp, telem.x, telem.y, headingRad);
        if (layers.tf) drawTFFrames(ctx, w, h, vp, { x: telem.x, y: telem.y, theta: headingRad }, tf);
      }

      // Goal marker
      if (goalMarker) drawGoalMarker(ctx, vp, goalMarker);

      // Nav status overlay
      if (navSession?.active) drawNavStatus(ctx, w, h, navSession);

      // Measure overlay
      if (measurePoints.p1) {
        drawMeasureLine(ctx, vp, measurePoints.p1, measurePoints.p2 || cursorWorld);
      }

      animFrameRef.current = requestAnimationFrame(render);
    };

    animFrameRef.current = requestAnimationFrame(render);
    return () => {
      if (animFrameRef.current) cancelAnimationFrame(animFrameRef.current);
    };
  }, [canvasSize, viewport, layers, grid, telem, activePath, simWorldSegments, tf, measurePoints, cursorWorld, goalMarker, navPath, navSession]);

  // ── JSX ───────────────────────────────────────────────────

  return (
    <div style={panelStyle}>
      {/* Header */}
      <div style={headerStyle}>
        <span style={titleStyle}>📊 RVizTDTU</span>
        <div style={{ display: 'flex', alignItems: 'center', gap: '2px' }}>
          <button
            style={headerBtnStyle(followRobot)}
            onClick={() => setFollowRobot(!followRobot)}
            title="Follow Robot"
          >
            🎯 Follow
          </button>
          <button
            style={headerBtnStyle(false)}
            onClick={() => viewport.resetZoom()}
            title="Reset View"
          >
            ↺ Reset
          </button>
          <button
            style={headerBtnStyle(false)}
            onClick={() => viewport.centerOn(telem.x ?? 5, telem.y ?? 5)}
            title="Center on Robot"
          >
            ⊕ Center
          </button>
        </div>
      </div>

      {/* Canvas Area */}
      <div style={canvasContainerStyle}>
        <RVizToolbar
          activeTool={activeTool}
          onToolChange={setActiveTool}
          layers={layers}
          onToggleLayer={toggleLayer}
        />
        <div ref={containerRef} style={canvasStyle}>
          <canvas
            ref={canvasRef}
            style={{
              width: canvasSize.w,
              height: canvasSize.h,
              cursor: activeTool === 'move' ? 'grab' : 'crosshair',
            }}
            onMouseDown={viewport.handlers.onMouseDown}
            onMouseMove={handleCanvasMouseMove}
            onMouseUp={viewport.handlers.onMouseUp}
            onMouseLeave={viewport.handlers.onMouseLeave}
            onClick={handleCanvasClick}
          />
        </div>
      </div>

      {/* Status Bar */}
      <div style={statusBarStyle}>
        <span>
          Cursor: ({cursorWorld.x.toFixed(2)}, {cursorWorld.y.toFixed(2)})m
        </span>
        <span>Zoom: {viewport.scale.toFixed(0)} px/m</span>
        {activeRobot && (
          <span style={{ color: '#10b981' }}>
            Robot: {activeRobot.name} ({telem.x?.toFixed(2)}, {telem.y?.toFixed(2)})
          </span>
        )}
        {grid && (
          <span>Map: {grid.width}×{grid.height} | Scans: {grid.scanCount}</span>
        )}
        {navSession?.active && (
          <span style={{ color: '#f59e0b', fontWeight: 600 }}>
            🧭 {navSession.status} WP:{navSession.currentWaypointIndex}/{navSession.path?.length}
          </span>
        )}
        {goalMarker && !navSession?.active && (
          <span style={{ color: '#3b82f6' }}>
            🎯 Goal: ({goalMarker.x.toFixed(2)}, {goalMarker.y.toFixed(2)})
          </span>
        )}
        <span style={{ marginLeft: 'auto' }}>
          {Object.entries(layers).filter(([, v]) => v).length} layers active
        </span>
      </div>

      {/* Topic Inspector */}
      <TopicInspector
        telemetry={telem}
        mapToOdom={tf}
        grid={grid}
        navSession={navSession}
        simInfo={firstSimInfo}
      />
    </div>
  );
}
