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

import React, { useRef, useEffect, useState, useCallback, useMemo } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import useNavStore from '../../stores/navStore.js';
import useMapStore from '../../stores/mapStore.js';
import { resolveMapFramePose } from '../../core/poseFrames.js';
import { buildDirectPath } from '../../core/navigationPathPolicy.js';
import { useRVizViewport } from './useRVizViewport.js';
import RVizToolbar from './RVizToolbar.jsx';
import TopicInspector from './TopicInspector.jsx';
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
  drawRobotFootprint,
  drawLocalCostmapWindow,
  drawDWATrajectory,
  drawAnimatedPath,
  drawGoalPoseArrow,
  drawRobotTrail,
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

const RVIZ_FRAME_MS = 1000 / 30;

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

import useUIStore from '../../stores/uiStore.js';

// ============================================================
//   COMPONENT
// ============================================================

function pathFromCurrentPose(path, currentWaypointIndex, pose) {
  if (!path || path.length < 2 || !pose) return path;
  const startIdx = Math.max(0, Math.min(currentWaypointIndex ?? 0, path.length - 1));
  const remaining = path.slice(startIdx);
  if (remaining.length === 0) return [{ x: pose.x, y: pose.y }, path[path.length - 1]];

  const first = remaining[0];
  if (Math.hypot(first.x - pose.x, first.y - pose.y) <= 0.03) {
    return remaining;
  }
  return [{ x: pose.x, y: pose.y }, ...remaining];
}

function headingDegFromRad(rad) {
  if (!Number.isFinite(rad)) return null;
  return ((rad * 180 / Math.PI) % 360 + 360) % 360;
}

export default function RVizPanel({ activePath }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const animFrameRef = useRef(null);
  const lastRenderTimeRef = useRef(0);
  const clearTrailTimeoutRef = useRef(null);
  const [canvasSize, setCanvasSize] = useState({ w: 400, h: 300 });
  const activeTool = useUIStore((s) => s.activeTool);
  const setActiveTool = useUIStore((s) => s.setActiveTool);
  const [followRobot, setFollowRobot] = useState(false);
  const [cursorWorld, setCursorWorld] = useState({ x: 0, y: 0 });
  const [measurePoints, setMeasurePoints] = useState({ p1: null, p2: null });
  const [goalMarker, setGoalMarker] = useState(null); // {x, y} world coords
  const [pendingGoalPose, setPendingGoalPose] = useState(null); // {x, y, headingRad}
  const [navPath, setNavPath] = useState(null); // planned path [{x,y}, ...]
  const [isNavLoading, setIsNavLoading] = useState(false); // pathfinding in progress

  // Layer visibility — clean defaults, user can toggle from toolbar
  const [layers, setLayers] = useState({
    grid: true, map: true, costmap: true, walls: true,
    laser: true, path: true, robot: true, tf: false, frontier: false,
    footprint: true, localCostmap: false, dwaPreview: false, trail: false,
  });

  const toggleLayer = useCallback((id) => {
    setLayers(prev => ({ ...prev, [id]: !prev[id] }));
  }, []);

  const mappingActive = useMapStore((s) => s.mappingActive);
  const startMapping = useMapStore((s) => s.startMapping);
  const stopMapping = useMapStore((s) => s.stopMapping);

  // Auto-clear measure points when switching away from measure tool
  useEffect(() => {
    if (activeTool !== 'measure') {
      setMeasurePoints({ p1: null, p2: null });
    }
    if (activeTool !== 'goal') {
      setPendingGoalPose(null);
    }
  }, [activeTool]);

  // Store data
  // Core robot data from robotStore
  const robots = useRobotStore((s) => s.robots);
  const occupancyGrid = useMapStore((s) => s.occupancyGrid);
  const mapperInstances = useMapStore((s) => s.mapperInstances);
  const mapToOdom = useMapStore((s) => s.mapToOdom);
  const selectedRobotId = useRobotStore((s) => s.selectedRobotId);
  const _simWorldSegments = useRobotStore((s) => s.simWorldSegments);
  const simInfo = useRobotStore((s) => s.simInfo);
  // Navigation from navStore
  const appNavigationSessions = useNavStore((s) => s.appNavigationSessions);
  const navigateToGoal = useNavStore((s) => s.navigateToGoal);
  const stopAppNavigation = useNavStore((s) => s.stopAppNavigation);
  const navStopRobot = useNavStore((s) => s.navStopRobot);
  const dwaTrajectories = useNavStore((s) => s.dwaTrajectories);
  const obstacleAvoidanceEnabled = useNavStore((s) => s.obstacleAvoidanceEnabled);
  const setObstacleAvoidanceEnabled = useNavStore((s) => s.setObstacleAvoidanceEnabled);

  // Robot trail tracking
  const robotTrailRef = useRef([]);
  const MAX_TRAIL_POINTS = 200;

  // Find active robot
  const robotList = Object.values(robots);
  const activeRobot = selectedRobotId
    ? robots[selectedRobotId]
    : robotList.find(r => r.status === 'connected');
  const activeRobotId = activeRobot?.id;
  const telem = activeRobot?.telemetry || {};
  const grid = activeRobotId ? occupancyGrid[activeRobotId] : null;
  const tf = activeRobotId ? (mapToOdom[activeRobotId] || { dx: 0, dy: 0, dTheta: 0 }) : { dx: 0, dy: 0, dTheta: 0 };
  const robotPose = activeRobotId
    ? resolveMapFramePose(activeRobot, { mapToOdom }, activeRobotId)
    : null;
  const navSession = activeRobotId ? appNavigationSessions[activeRobotId] : null;
  const firstSimInfo = Object.values(simInfo)[0];
  const isMapping = activeRobotId ? !!mappingActive[activeRobotId] : false;
  const hasCancelableGoal = !!pendingGoalPose || !!goalMarker || !!navSession?.active || ['TRACK', 'F_TURN', 'RECOVERY_SPIN', 'RECOVERY_BACKUP', 'RECOVERY_REPLAN', 'PAUSED'].includes(telem.nav);

  // Data source detection for mode badge
  const dataSource = activeRobot?._sim ? 'sim'
    : (activeRobot?.telemetry?.hitl || activeRobot?.connection?.hitlEnabled) ? 'hitl'
      : activeRobot ? 'real' : 'none';

  const costmapStats = useMemo(() => {
    if (!grid?.costmap) return { hasData: false, activeCells: 0 };
    let activeCells = 0;
    for (let i = 0; i < grid.costmap.length; i++) {
      if (grid.costmap[i] > 0) activeCells++;
    }
    return { hasData: true, activeCells };
  }, [grid, grid?.costmapVersion, grid?.width, grid?.height]);

  // Stabilize simWorldSegments: use empty array in real mode to prevent
  // sim engine's 60Hz state updates from triggering canvas re-renders/flicker.
  const emptySegments = useRef([]).current;
  const simWorldSegments = dataSource === 'real' ? emptySegments : _simWorldSegments;

  const handleToggleMapping = useCallback(() => {
    if (!activeRobotId) return;
    const getRobotStore = () => useRobotStore.getState();
    if (isMapping) {
      stopMapping(activeRobotId, getRobotStore);
    } else {
      startMapping(activeRobotId, getRobotStore);
    }
  }, [activeRobotId, isMapping, startMapping, stopMapping]);

  const submitNavigationGoal = useCallback((goal) => {
    if (!activeRobotId || !goal) return;

    const finalHeadingDeg = headingDegFromRad(goal.headingRad);
    setPendingGoalPose(null);
    setGoalMarker({ x: goal.x, y: goal.y, headingRad: goal.headingRad });
    setIsNavLoading(true);
    console.log(`[RVizTDTU] Nav Goal: (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}), heading=${finalHeadingDeg?.toFixed(1) ?? 'auto'} deg`);

    navigateToGoal(activeRobotId, goal.x, goal.y, finalHeadingDeg)
      .then((result) => {
        setIsNavLoading(false);
        if (result.success) {
          if (result.path && result.path.length > 0) {
            const acceptedGoal = result.path[result.path.length - 1];
            setGoalMarker({ x: acceptedGoal.x, y: acceptedGoal.y, headingRad: goal.headingRad });
            setNavPath(result.path);
          } else {
            setNavPath(null);
          }
        } else {
          console.warn(`[RVizTDTU] ${result.error}`);
          setGoalMarker(null);
          setNavPath(null);
        }
      })
      .catch((err) => {
        setIsNavLoading(false);
        console.error('[RVizTDTU] Nav error:', err);
        setGoalMarker(null);
        setNavPath(null);
      });

    setActiveTool('move');
  }, [activeRobotId, navigateToGoal, setActiveTool]);

  const cancelPendingGoal = useCallback(() => {
    setPendingGoalPose(null);
    setGoalMarker(null);
    setNavPath(null);
  }, []);

  useEffect(() => {
    if (!pendingGoalPose) return undefined;

    const handleKeyDown = (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        submitNavigationGoal(pendingGoalPose);
      } else if (e.key === 'Escape') {
        e.preventDefault();
        cancelPendingGoal();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [pendingGoalPose, submitNavigationGoal, cancelPendingGoal]);

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

  // ── Auto-Clear Goal & Path ──────────────────────────────
  const prevNavActive = useRef(false);
  useEffect(() => {
    const isActive = !!navSession?.active;
    if (prevNavActive.current && !isActive) {
      setGoalMarker(null);
      setNavPath(null);
    }
    prevNavActive.current = isActive;
  }, [navSession?.active]);

  // ── Auto-Clear Goal for Onboard mode (ESP32 manages nav) ──
  // When ESP32 telemetry reports DONE or ERROR, clear the goal marker
  const prevNavStatus = useRef(null);
  useEffect(() => {
    const navStatus = telem.nav;
    if (prevNavStatus.current && prevNavStatus.current !== 'IDLE' && prevNavStatus.current !== 'DONE' && prevNavStatus.current !== 'ERROR') {
      // Was navigating, now finished
      if (navStatus === 'DONE' || navStatus === 'ERROR' || navStatus === 'IDLE') {
        if (goalMarker && !navSession?.active) {
          // Onboard mode: clear goal after ESP32 finishes
          setTimeout(() => {
            setGoalMarker(null);
            setNavPath(null);
          }, 2000); // Show goal for 2s after completion for visual feedback
        }
      }
    }
    prevNavStatus.current = navStatus;
  }, [telem.nav, goalMarker, navSession?.active]);

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
    if (followRobot && robotPose) {
      viewport.followRobot(robotPose.x, robotPose.y);
    }
  }, [followRobot, robotPose?.x, robotPose?.y]);

  // ── Canvas Mouse Handlers ─────────────────────────────────

  const handleCanvasMouseMove = useCallback((e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const world = viewport.screenToWorld(sx, sy);
    setCursorWorld(world);

    if (pendingGoalPose) {
      const dx = world.x - pendingGoalPose.x;
      const dy = world.y - pendingGoalPose.y;
      if (Math.hypot(dx, dy) > 0.03) {
        setPendingGoalPose(prev => prev ? { ...prev, headingRad: Math.atan2(dy, dx) } : prev);
      }
    }

    // Pan in move tool; rotate with right mouse drag from any tool.
    if (!pendingGoalPose && (activeTool === 'move' || (e.buttons & 2))) {
      viewport.handlers.onMouseMove(e);
    }
  }, [viewport, activeTool, pendingGoalPose]);

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

      const headingRad = robotPose
        ? Math.atan2(world.y - robotPose.y, world.x - robotPose.x)
        : 0;
      setPendingGoalPose({ x: world.x, y: world.y, headingRad });
      setGoalMarker(null);
      setNavPath(null);
      console.log(`[RVizTDTU] Pending Nav Goal: (${world.x.toFixed(2)}, ${world.y.toFixed(2)})`);
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
        // Third click resets
        setMeasurePoints({ p1: world, p2: null });
      }
    }
  }, [activeTool, measurePoints, viewport, activeRobotId, telem, robots, robotPose]);

  const handleClearGoal = useCallback(() => {
    if (activeRobotId) {
      navStopRobot(activeRobotId);
      if (navSession?.active) {
        stopAppNavigation(activeRobotId, 'CANCELED', true);
      }
    }
    setPendingGoalPose(null);
    setGoalMarker(null);
    setNavPath(null);
  }, [activeRobotId, navSession, navStopRobot, stopAppNavigation]);

  const handleCanvasMouseDown = useCallback((e) => {
    if (pendingGoalPose && e.button === 2) {
      e.preventDefault();
      return;
    }
    if (activeTool === 'move' || e.button === 2) {
      viewport.handlers.onMouseDown(e);
    }
  }, [activeTool, viewport, pendingGoalPose]);

  const handleCanvasMouseUp = useCallback((e) => {
    if (pendingGoalPose && e.button === 2) {
      e.preventDefault();
      return;
    }
    if (activeTool === 'move' || e.button === 2) {
      viewport.handlers.onMouseUp(e);
    }
  }, [activeTool, viewport, pendingGoalPose]);

  const handleCanvasMouseLeave = useCallback((e) => {
    viewport.handlers.onMouseLeave(e);
  }, [viewport]);

  // Keep browser context menu out of the RViz canvas; right-click confirms pending goal pose.
  const handleContextMenu = useCallback((e) => {
    e.preventDefault();
    if (pendingGoalPose) {
      submitNavigationGoal(pendingGoalPose);
    }
  }, [pendingGoalPose, submitNavigationGoal]);

  // ── Render Loop ───────────────────────────────────────────

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const render = (timestamp = 0) => {
      if (timestamp - lastRenderTimeRef.current < RVIZ_FRAME_MS) {
        animFrameRef.current = requestAnimationFrame(render);
        return;
      }
      lastRenderTimeRef.current = timestamp;

      const { w, h } = canvasSize;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const pixelW = Math.max(1, Math.floor(w * dpr));
      const pixelH = Math.max(1, Math.floor(h * dpr));
      if (canvas.width !== pixelW || canvas.height !== pixelH) {
        canvas.width = pixelW;
        canvas.height = pixelH;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

      // Clear
      ctx.fillStyle = '#0c1219';
      ctx.fillRect(0, 0, w, h);

      const vp = {
        worldToScreen: viewport.worldToScreen,
        screenToWorld: viewport.screenToWorld,
        scale: viewport.scale,
        rotation: viewport.rotation,
      };

      // Draw layers in order (back to front)
      if (layers.grid) drawGrid(ctx, w, h, vp);
      // Sim world segments (virtual walls) — COMPLETELY skip in real mode
      // to prevent sim engine ticks from causing ANY visual artifacts
      if (layers.walls && dataSource !== 'real' && simWorldSegments?.length > 0) {
        drawWorldSegments(ctx, w, h, vp, simWorldSegments);
      }
      if (layers.map && grid) drawOccupancyMap(ctx, w, h, vp, grid);
      if (layers.costmap && grid) drawCostmapNav2(ctx, w, h, vp, grid);
      if (layers.frontier && grid && grid.frontierCells) drawFrontiers(ctx, w, h, vp, grid.frontierCells, grid);

      // Navigation path (from click-to-navigate or active nav session)
      const displayPath = navSession?.active
        ? pathFromCurrentPose(navSession.path, navSession.currentWaypointIndex, robotPose)
        : (navSession?.path || navPath || activePath);

      // Robot-specific layers
      if (robotPose) {
        const poseX = robotPose.x;
        const poseY = robotPose.y;
        const headingRad = robotPose.theta;
        const odomHeadingRad = Number.isFinite(telem.headingRad)
          ? telem.headingRad
          : (telem.heading ?? 0) * Math.PI / 180;

        // Trail tracking — record robot position
        if (navSession?.active && layers.trail) {
          const lastPt = robotTrailRef.current[robotTrailRef.current.length - 1];
          if (!lastPt || Math.hypot(poseX - lastPt.x, poseY - lastPt.y) > 0.03) {
            robotTrailRef.current.push({ x: poseX, y: poseY });
            if (robotTrailRef.current.length > MAX_TRAIL_POINTS) {
              robotTrailRef.current = robotTrailRef.current.slice(-MAX_TRAIL_POINTS);
            }
          }
        } else if (!navSession?.active && robotTrailRef.current.length > 0 && !clearTrailTimeoutRef.current) {
          // Clear trail once after nav ends; avoid queuing a timeout every frame.
          clearTrailTimeoutRef.current = setTimeout(() => {
            robotTrailRef.current = [];
            clearTrailTimeoutRef.current = null;
          }, 3000);
        }

        // Robot trail (breadcrumb)
        if (layers.trail) drawRobotTrail(ctx, vp, robotTrailRef.current);

        // Local costmap window
        if (layers.localCostmap && navSession?.active) drawLocalCostmapWindow(ctx, vp, poseX, poseY);

        // Animated path (replaces basic drawPath when navigating)
        if (layers.path && displayPath && navSession?.active) {
          drawAnimatedPath(ctx, w, h, vp, displayPath, poseX, poseY);
        } else if (layers.path && displayPath) {
          drawPath(ctx, w, h, vp, displayPath);
        }
        if (layers.path && pendingGoalPose) {
          const pendingRoute = buildDirectPath({ x: poseX, y: poseY }, pendingGoalPose);
          if (pendingRoute.length > 1) drawPath(ctx, w, h, vp, pendingRoute);
        }

        // DWA trajectory preview
        const dwaTrajectory = activeRobotId ? dwaTrajectories[activeRobotId] : null;
        if (layers.dwaPreview && dwaTrajectory) drawDWATrajectory(ctx, vp, dwaTrajectory);

        if (layers.laser) drawLaserScan(ctx, w, h, vp, poseX, poseY, headingRad, telem.lidar);
        if (layers.footprint) drawRobotFootprint(ctx, vp, poseX, poseY);
        if (layers.robot) {
          const odomTheta = telem.odomTheta !== undefined ? telem.odomTheta : null;
          drawRobotPose(ctx, w, h, vp, poseX, poseY, headingRad, 0.12, telem.linearVel || 0, odomTheta);
        }
        if (layers.tf) drawTFFrames(ctx, w, h, vp, { x: telem.x, y: telem.y, theta: odomHeadingRad }, tf);
      }

      // Goal marker — pending pose uses selected heading, active nav keeps final heading.
      if (pendingGoalPose && robotPose) {
        drawGoalPoseArrow(ctx, vp, pendingGoalPose, robotPose.x, robotPose.y, pendingGoalPose.headingRad);
      } else if (goalMarker && navSession?.active && robotPose) {
        drawGoalPoseArrow(ctx, vp, goalMarker, robotPose.x, robotPose.y, goalMarker.headingRad);
      } else if (goalMarker) {
        // Also show for Onboard mode where there's no browser navSession
        drawGoalMarker(ctx, vp, goalMarker);
      }

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
      if (clearTrailTimeoutRef.current) {
        clearTimeout(clearTrailTimeoutRef.current);
        clearTrailTimeoutRef.current = null;
      }
    };
    // NOTE: simWorldSegments intentionally excluded when dataSource==='real'
    // to prevent sim engine tick updates from causing canvas flicker.
    // When dataSource changes, the effect re-runs via dataSource dep.
  }, [canvasSize, viewport, layers, grid, telem, activePath,
    dataSource === 'real' ? null : simWorldSegments,
    tf, measurePoints, cursorWorld, pendingGoalPose, goalMarker, navPath, navSession, dwaTrajectories,
    activeRobotId, dataSource, robotPose?.x, robotPose?.y, robotPose?.theta]);

  // ── JSX ───────────────────────────────────────────────────

  return (
    <div style={panelStyle}>
      {/* Header */}
      <div style={headerStyle}>
        <span style={titleStyle}>📊 RVizTDTU</span>
        <span style={{
          fontSize: '9px', padding: '1px 8px', borderRadius: '4px', fontWeight: 600, marginLeft: '8px',
          background: dataSource === 'sim' ? 'rgba(139,92,246,0.2)' :
            dataSource === 'hitl' ? 'rgba(59,130,246,0.2)' :
              dataSource === 'real' ? 'rgba(16,185,129,0.2)' : 'rgba(100,116,139,0.2)',
          color: dataSource === 'sim' ? '#c4b5fd' :
            dataSource === 'hitl' ? '#93c5fd' :
              dataSource === 'real' ? '#6ee7b7' : '#94a3b8',
        }}>
          {dataSource === 'sim' && 'SimLidar (Browser)'}
          {dataSource === 'hitl' && 'HITL Hybrid'}
          {dataSource === 'real' && 'ESP32 Sensors'}
          {dataSource === 'none' && 'No Data'}
        </span>
        <div style={{ display: 'flex', alignItems: 'center', gap: '2px' }}>
          {hasCancelableGoal && (
            <button
              style={{ ...headerBtnStyle(false), color: '#ef4444', borderColor: 'rgba(239, 68, 68, 0.4)' }}
              onClick={handleClearGoal}
              title="Clear Goal & Cancel Navigation"
            >
              Cancel Goal
            </button>
          )}
          <button
            style={{
              ...headerBtnStyle(layers.costmap),
              color: layers.costmap ? '#fbbf24' : '#64748b',
              borderColor: layers.costmap ? 'rgba(251, 191, 36, 0.45)' : 'rgba(255,255,255,0.08)',
              background: layers.costmap ? 'rgba(251, 191, 36, 0.16)' : 'transparent',
              minWidth: '62px',
            }}
            onClick={() => toggleLayer('costmap')}
            title={costmapStats.hasData ? `Costmap: ${costmapStats.activeCells} active cells` : 'Costmap: no data yet'}
          >
            Costmap
          </button>
          <button
            style={{
              ...headerBtnStyle(obstacleAvoidanceEnabled),
              color: obstacleAvoidanceEnabled ? '#22c55e' : '#64748b',
              borderColor: obstacleAvoidanceEnabled ? 'rgba(34, 197, 94, 0.45)' : 'rgba(255,255,255,0.08)',
              background: obstacleAvoidanceEnabled ? 'rgba(34, 197, 94, 0.14)' : 'transparent',
              minWidth: '58px',
            }}
            onClick={() => setObstacleAvoidanceEnabled(!obstacleAvoidanceEnabled)}
            title={obstacleAvoidanceEnabled ? 'Né vật cản: ON' : 'Né vật cản: OFF'}
          >
            Né VC
          </button>
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
            onClick={() => viewport.centerOn(robotPose?.x ?? 5, robotPose?.y ?? 5)}
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
          isMapping={isMapping}
          onToggleMapping={handleToggleMapping}
          dataSource={dataSource}
        />
        <div ref={containerRef} style={canvasStyle}>
          {activeTool === 'goal' && !pendingGoalPose && (
            <div style={{
              position: 'absolute',
              top: '16px',
              left: '50%',
              transform: 'translateX(-50%)',
              background: 'rgba(139, 92, 246, 0.9)',
              color: '#fff',
              padding: '8px 16px',
              borderRadius: '20px',
              fontWeight: 'bold',
              pointerEvents: 'none',
              boxShadow: '0 4px 12px rgba(0,0,0,0.5)',
              zIndex: 10,
              border: '1px solid rgba(255,255,255,0.2)'
            }}>
              👆 Hãy nhấp chuột vào vị trí trên bản đồ để Robot di chuyển tới
            </div>
          )}
          {pendingGoalPose && (
            <div style={{
              position: 'absolute',
              top: '16px',
              left: '50%',
              transform: 'translateX(-50%)',
              background: 'rgba(15, 23, 42, 0.94)',
              color: '#fff',
              padding: '8px 12px',
              borderRadius: '8px',
              fontWeight: 700,
              boxShadow: '0 4px 12px rgba(0,0,0,0.5)',
              zIndex: 11,
              border: '1px solid rgba(168, 85, 247, 0.45)',
              display: 'flex',
              alignItems: 'center',
              gap: '8px',
            }}>
              <span>Auto waypoints</span>
              <span>Heading {headingDegFromRad(pendingGoalPose.headingRad)?.toFixed(0)} deg</span>
              <span style={{ color: '#c4b5fd', fontSize: '10px', fontWeight: 600 }}>
                move mouse to rotate, Enter / right-click to go
              </span>
              <button
                style={{ ...headerBtnStyle(false), color: '#cbd5e1', marginLeft: 0 }}
                onClick={cancelPendingGoal}
                title="Cancel goal pose"
              >
                Cancel
              </button>
            </div>
          )}
          {isNavLoading && (
            <div style={{
              position: 'absolute',
              top: '16px',
              left: '50%',
              transform: 'translateX(-50%)',
              background: 'rgba(59, 130, 246, 0.9)',
              color: '#fff',
              padding: '8px 16px',
              borderRadius: '20px',
              fontWeight: 'bold',
              pointerEvents: 'none',
              boxShadow: '0 4px 12px rgba(0,0,0,0.5)',
              zIndex: 10,
              animation: 'pulse 1.5s infinite',
            }}>
              ⏳ Đang tính đường đi...
            </div>
          )}
          <canvas
            ref={canvasRef}
            style={{
              width: canvasSize.w,
              height: canvasSize.h,
              cursor: activeTool === 'move' ? 'grab' : 'crosshair',
            }}
            onMouseDown={handleCanvasMouseDown}
            onMouseMove={handleCanvasMouseMove}
            onMouseUp={handleCanvasMouseUp}
            onMouseLeave={handleCanvasMouseLeave}
            onClick={handleCanvasClick}
            onContextMenu={handleContextMenu}
          />
        </div>
      </div>

      {/* Status Bar */}
      <div style={statusBarStyle}>
        <span>
          Cursor: ({cursorWorld.x.toFixed(2)}, {cursorWorld.y.toFixed(2)})m
        </span>
        <span>Zoom: {viewport.scale.toFixed(0)} px/m</span>
        <span>Rot: {(viewport.rotation * 180 / Math.PI).toFixed(0)} deg</span>
        {activeRobot && (
          <span style={{ color: '#10b981' }}>
            Robot: {activeRobot.name} ({robotPose?.x?.toFixed(2)}, {robotPose?.y?.toFixed(2)})
          </span>
        )}
        {grid && (
          <span>
            Map: {grid.width}x{grid.height} | Scans: {grid.scanCount}
            {' '}| Costmap: {costmapStats.hasData ? `${costmapStats.activeCells} cells` : 'none'}
          </span>
        )}
        {navSession?.active && (
          <span style={{ color: '#f59e0b', fontWeight: 600 }}>
            🧭 {navSession.status} WP:{navSession.currentWaypointIndex}/{navSession.path?.length}
          </span>
        )}
        {pendingGoalPose && (
          <span style={{ color: '#c084fc' }}>
            Goal: ({pendingGoalPose.x.toFixed(2)}, {pendingGoalPose.y.toFixed(2)}) H:{headingDegFromRad(pendingGoalPose.headingRad)?.toFixed(0)} deg
          </span>
        )}
        {goalMarker && !navSession?.active && !pendingGoalPose && (
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
