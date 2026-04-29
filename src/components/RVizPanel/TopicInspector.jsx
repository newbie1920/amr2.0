/**
 * RVizTDTU — Topic Inspector Panel
 * 
 * Hiển thị data streams real-time giống rqt_topic / RViz topics:
 *   - /cmd_vel → {v, w}
 *   - /scan → point count, min/max/avg distance
 *   - /odom → {x, y, theta}
 *   - /tf → mapToOdom transform
 *   - /map → grid stats
 *   - /nav_status → navigation state
 */

import React, { useState } from 'react';

// ============================================================
//   STYLES
// ============================================================

const panelStyle = {
  background: 'rgba(15, 25, 35, 0.98)',
  borderTop: '1px solid rgba(139, 92, 246, 0.2)',
  padding: '6px 10px',
  fontFamily: "'Inter', monospace",
  fontSize: '10px',
  color: '#94a3b8',
  maxHeight: '180px',
  overflowY: 'auto',
};

const headerStyle = {
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  marginBottom: '4px',
  cursor: 'pointer',
};

const titleStyle = {
  fontSize: '10px',
  fontWeight: 700,
  textTransform: 'uppercase',
  letterSpacing: '1px',
  color: '#8b5cf6',
};

const topicRowStyle = {
  display: 'grid',
  gridTemplateColumns: '90px 1fr 50px',
  gap: '6px',
  padding: '2px 0',
  borderBottom: '1px solid rgba(255,255,255,0.03)',
  alignItems: 'center',
};

const topicNameStyle = (active) => ({
  color: active ? '#a78bfa' : '#475569',
  fontWeight: 600,
  fontSize: '10px',
});

const topicValueStyle = {
  color: '#e2e8f0',
  fontFamily: "'JetBrains Mono', monospace",
  fontSize: '10px',
};

const topicHzStyle = (active) => ({
  color: active ? '#4ade80' : '#475569',
  fontSize: '9px',
  textAlign: 'right',
});

// ============================================================
//   COMPONENT
// ============================================================

export default function TopicInspector({ telemetry, mapToOdom, grid, navSession, simInfo }) {
  const [collapsed, setCollapsed] = useState(false);

  const telem = telemetry || {};
  const tf = mapToOdom || { dx: 0, dy: 0, dTheta: 0 };
  const lidar = telem.lidar || [];
  const hasData = !!telem.x;

  // Compute lidar stats
  let lidarMin = Infinity, lidarMax = 0, lidarSum = 0;
  for (const pt of lidar) {
    if (pt.d < lidarMin) lidarMin = pt.d;
    if (pt.d > lidarMax) lidarMax = pt.d;
    lidarSum += pt.d;
  }
  const lidarAvg = lidar.length > 0 ? lidarSum / lidar.length : 0;

  const topics = [
    {
      name: '/cmd_vel',
      value: `v=${(telem.linearVel ?? 0).toFixed(3)}  ω=${(telem.angularVel ?? 0).toFixed(3)}`,
      hz: '50Hz',
      active: Math.abs(telem.linearVel ?? 0) > 0.001 || Math.abs(telem.angularVel ?? 0) > 0.001,
    },
    {
      name: '/scan',
      value: `${lidar.length} pts  min=${(lidarMin / 1000).toFixed(2)}m  max=${(lidarMax / 1000).toFixed(2)}m  avg=${(lidarAvg / 1000).toFixed(2)}m`,
      hz: '10Hz',
      active: lidar.length > 0,
    },
    {
      name: '/odom',
      value: `x=${(telem.x ?? 0).toFixed(3)}  y=${(telem.y ?? 0).toFixed(3)}  θ=${(telem.heading ?? 0).toFixed(1)}°`,
      hz: '50Hz',
      active: hasData,
    },
    {
      name: '/tf',
      value: `Δx=${tf.dx.toFixed(4)}  Δy=${tf.dy.toFixed(4)}  Δθ=${(tf.dTheta * 180 / Math.PI).toFixed(2)}°`,
      hz: '—',
      active: tf.dx !== 0 || tf.dy !== 0,
    },
    {
      name: '/map',
      value: grid
        ? `${grid.width}×${grid.height}  res=${grid.resolution}m  scans=${grid.scanCount}`
        : 'No map loaded',
      hz: '2Hz',
      active: !!grid,
    },
    {
      name: '/nav_status',
      value: navSession?.active
        ? `${navSession.status}  wp=${navSession.currentWaypointIndex}/${navSession.path?.length}`
        : 'IDLE',
      hz: '10Hz',
      active: navSession?.active,
    },
  ];

  if (simInfo) {
    topics.push({
      name: '/sim_info',
      value: `RTF=${simInfo.rtf?.toFixed(2)}  t=${simInfo.simTime?.toFixed(1)}s  steps=${simInfo.stepCount}`,
      hz: `${(simInfo.speedFactor ?? 1)}x`,
      active: simInfo.running,
    });
  }

  return (
    <div style={panelStyle}>
      <div style={headerStyle} onClick={() => setCollapsed(!collapsed)}>
        <span style={titleStyle}>📡 Topics {collapsed ? '▶' : '▼'}</span>
        <span style={{ fontSize: '9px', color: '#475569' }}>
          {topics.filter(t => t.active).length}/{topics.length} active
        </span>
      </div>

      {!collapsed && topics.map(topic => (
        <div key={topic.name} style={topicRowStyle}>
          <span style={topicNameStyle(topic.active)}>{topic.name}</span>
          <span style={topicValueStyle}>{topic.value}</span>
          <span style={topicHzStyle(topic.active)}>{topic.hz}</span>
        </div>
      ))}
    </div>
  );
}
