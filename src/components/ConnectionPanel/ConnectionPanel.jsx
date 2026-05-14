import { useState, useEffect, useRef } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import useNavStore from '../../stores/navStore.js';
import vi from '../../i18n/vi.js';
// NOTE: useNavStore must be used as a hook (not .getState()) for React reactivity
import { CHARGING_STATIONS } from '../../core/warehouse.js';

export default function ConnectionPanel() {
  const [name, setName] = useState('');
  const [ip, setIp] = useState('192.168.1.');
  const [port, setPort] = useState('81');
  const [showForm, setShowForm] = useState(false);

  const { addRobot, removeRobot, connectRobot, disconnectRobot, selectRobot, selectedRobotId } = useRobotStore();
  const robots = useRobotStore((s) => s.robots);
  const explorationInfo = useRobotStore((s) => s.explorationInfo) || {};
  const discoveredRobots = useRobotStore((s) => s.discoveredRobots);
  const mqttConnected = useRobotStore((s) => s.mqttConnected);
  const navModes = useNavStore((s) => s.navModes);        // ← reactive!
  const setNavMode = useNavStore((s) => s.setNavMode);    // ← reactive!
  const getNavMode = useNavStore((s) => s.getNavMode);
  const robotList = Object.values(robots);
  const discoveredList = Object.values(discoveredRobots);

  const handleAdd = () => {
    if (!name.trim() || !ip.trim()) return;
    const id = addRobot(name.trim(), ip.trim(), parseInt(port) || 81);
    connectRobot(id);
    setName('');
    setIp('192.168.1.');
    setPort('81');
    setShowForm(false);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleAdd();
  };

  const handleConnectDiscovered = (info) => {
    const id = addRobot(info.name, info.ip, info.port);
    connectRobot(id);
    selectRobot(id);
  };

  return (
    <div className="panel">
      <div className="panel__header">
        <span className="panel__title">🤖 {vi.connection.title}</span>
        <button
          className="btn btn--primary btn--sm"
          onClick={() => setShowForm(!showForm)}
        >
          {showForm ? '✕' : '+ ' + vi.connection.addRobot}
        </button>
      </div>

      {showForm && (
        <div className="connection-form">
          <div className="input-group">
            <label className="input-group__label">{vi.connection.robotName}</label>
            <input
              className="input input--sm"
              type="text"
              placeholder={vi.connection.namePlaceholder}
              value={name}
              onChange={(e) => setName(e.target.value)}
              onKeyDown={handleKeyDown}
            />
          </div>
          <div className="connection-form__row">
            <div className="input-group">
              <label className="input-group__label">{vi.connection.ipAddress}</label>
              <input
                className="input input--sm"
                type="text"
                placeholder={vi.connection.ipPlaceholder}
                value={ip}
                onChange={(e) => setIp(e.target.value)}
                onKeyDown={handleKeyDown}
              />
            </div>
            <div className="input-group">
              <label className="input-group__label">{vi.connection.port}</label>
              <input
                className="input input--sm"
                type="number"
                value={port}
                onChange={(e) => setPort(e.target.value)}
                onKeyDown={handleKeyDown}
              />
            </div>
          </div>
          <button className="btn btn--success btn--full btn--sm" onClick={handleAdd}>
            {vi.connection.connect}
          </button>
        </div>
      )}

      {/* ── MQTT Auto-Discovery Section ─────────────────────── */}
      {discoveredList.length > 0 && (
        <div style={{ marginTop: '8px' }}>
          <div style={{
            display: 'flex', alignItems: 'center', gap: '6px',
            fontSize: '11px', color: 'var(--text-muted)', marginBottom: '6px',
          }}>
            <span style={{
              width: '6px', height: '6px', borderRadius: '50%',
              background: mqttConnected ? '#10b981' : '#ef4444',
              boxShadow: mqttConnected ? '0 0 6px #10b981' : 'none',
              animation: mqttConnected ? 'pulse 2s infinite' : 'none',
            }} />
            <span>🔍 Tự động phát hiện ({discoveredList.length} robot)</span>
          </div>
          {discoveredList.map((info) => {
            // Check if this robot is already added
            const alreadyAdded = Object.values(robots).some(
              r => r.ip === info.ip && r.port === info.port
            );
            return (
              <div key={info.id} style={{
                display: 'flex', alignItems: 'center', justifyContent: 'space-between',
                padding: '6px 10px', marginBottom: '4px',
                background: 'rgba(16, 185, 129, 0.08)',
                border: '1px solid rgba(16, 185, 129, 0.2)',
                borderRadius: '8px', fontSize: '11px',
              }}>
                <div>
                  <div style={{ fontWeight: '600', color: 'var(--text-primary)' }}>
                    📡 {info.name}
                  </div>
                  <div style={{ color: 'var(--text-muted)', fontSize: '10px' }}>
                    {info.ip}:{info.port} • 🔋{info.battery}% • v{info.firmware}
                  </div>
                </div>
                {alreadyAdded ? (
                  <span style={{ color: '#10b981', fontSize: '10px', fontWeight: '600' }}>
                    ✓ Đã thêm
                  </span>
                ) : (
                  <button
                    className="btn btn--success btn--sm"
                    style={{ fontSize: '10px', padding: '2px 10px' }}
                    onClick={() => handleConnectDiscovered(info)}
                  >
                    Kết nối
                  </button>
                )}
              </div>
            );
          })}
        </div>
      )}

      {/* Robot List */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
        {robotList.length === 0 && (
          <div style={{ textAlign: 'center', padding: '20px', color: 'var(--text-muted)', fontSize: '12px' }}>
            {vi.common.noData}
          </div>
        )}

        {robotList.map((robot) => {
          const isSelected = selectedRobotId === robot.id;
          const isConnected = robot.status === 'connected';
          const batteryColor = robot.telemetry.battery > 50 ? 'var(--accent-success)' :
                               robot.telemetry.battery > 20 ? 'var(--accent-warning)' : 'var(--accent-danger)';
          const expInfo = explorationInfo[robot.id];

          return (
            <div
              key={robot.id}
              className={`card robot-card ${isSelected ? 'card--active' : ''}`}
              onClick={() => selectRobot(robot.id)}
            >
              <div className="robot-card__header">
                <span className="robot-card__name">
                  {robot.name}
                  {robot._sim && (
                    <span style={{
                      marginLeft: '6px', fontSize: '9px', padding: '1px 6px',
                      background: 'rgba(139, 92, 246, 0.2)', color: '#a78bfa',
                      borderRadius: '4px', fontWeight: '600',
                    }}>🏭 SIM</span>
                  )}
                  {!robot._sim && (robot.telemetry?.hitl || robot.connection?.hitlEnabled) && (
                    <span style={{
                      marginLeft: '6px', fontSize: '9px', padding: '1px 6px',
                      background: 'rgba(59, 130, 246, 0.2)', color: '#93c5fd',
                      borderRadius: '4px', fontWeight: '600',
                    }}>🌐 HITL</span>
                  )}
                  {!robot._sim && !(robot.telemetry?.hitl || robot.connection?.hitlEnabled) && (
                    <span style={{
                      marginLeft: '6px', fontSize: '9px', padding: '1px 6px',
                      background: 'rgba(16, 185, 129, 0.2)', color: '#6ee7b7',
                      borderRadius: '4px', fontWeight: '600',
                    }}>🔌 REAL</span>
                  )}
                </span>
                <span className={`robot-card__status robot-card__status--${robot.status}`}>
                  {robot.status === 'connected' ? vi.connection.connected :
                   robot.status === 'connecting' ? vi.connection.connecting :
                   vi.connection.disconnected}
                </span>
              </div>

              <div style={{ fontSize: '11px', color: 'var(--text-muted)', marginBottom: '6px' }}>
                📡 {robot.ip}:{robot.port}
              </div>

              {isConnected && (
                <>
                  <div className="robot-card__info">
                    <div className="robot-card__info-item">
                      <span>X:</span>
                      <span className="robot-card__info-value">{robot.telemetry.x.toFixed(2)}m</span>
                    </div>
                    <div className="robot-card__info-item">
                      <span>Y:</span>
                      <span className="robot-card__info-value">{robot.telemetry.y.toFixed(2)}m</span>
                    </div>
                    <div className="robot-card__info-item">
                      <span>{vi.robot.heading}:</span>
                      <span className="robot-card__info-value">{robot.telemetry.heading.toFixed(1)}°</span>
                    </div>
                    <div className="robot-card__info-item">
                      <span>{vi.robot.speed}:</span>
                      <span className="robot-card__info-value">{robot.telemetry.linearVel.toFixed(2)}</span>
                    </div>
                    <div className="robot-card__info-item" style={{ gridColumn: 'span 2' }}>
                      <span>Trạng thái:</span>
                      <span className="robot-card__info-value" style={{ 
                        color: robot.telemetry.nav === 'PAUSED' ? 'var(--accent-warning)' : 'inherit',
                        fontWeight: robot.telemetry.nav === 'PAUSED' ? 'bold' : 'normal'
                      }}>
                        {robot.telemetry.nav === 'PAUSED' ? '⚠️ YIELDING (Nhường đường)' : robot.telemetry.nav}
                      </span>
                    </div>
                    {robot.telemetry.architecture === 'pc_slam' && (
                      <div className="robot-card__info-item" style={{ gridColumn: 'span 2' }}>
                        <span>SLAM Score:</span>
                        <span className="robot-card__info-value" style={{ color: (robot.telemetry.matchScore || 0) > 300 ? 'var(--accent-success)' : 'var(--accent-warning)' }}>
                          {robot.telemetry.matchScore ? Math.round(robot.telemetry.matchScore) : 'N/A'}
                        </span>
                      </div>
                    )}
                    {expInfo && expInfo.phase && expInfo.phase !== 'IDLE' && (
                      <div className="robot-card__info-item" style={{ gridColumn: 'span 2' }}>
                        <span>Khám phá:</span>
                        <span className="robot-card__info-value" style={{ color: 'var(--accent-warning)' }}>
                          {expInfo.phase} (Recov: {expInfo.recoveryCount})
                        </span>
                      </div>
                    )}
                  </div>

                  <div className="robot-card__battery">
                    <div
                      className="robot-card__battery-fill"
                      style={{
                        width: `${robot.telemetry.battery}%`,
                        background: batteryColor,
                      }}
                    />
                  </div>
                  <div style={{ fontSize: '10px', color: batteryColor, marginTop: '4px', textAlign: 'right' }}>
                    🔋 {robot.telemetry.battery}%
                  </div>

                  {/* Brain Info — Context-sensitive per mode */}
                  {robot._sim ? (
                    <div style={{
                      marginTop: '6px', padding: '6px 8px',
                      background: 'rgba(139, 92, 246, 0.08)',
                      borderRadius: '6px', fontSize: '10px',
                      color: 'var(--text-muted)',
                    }}>
                      <span>🧠 Brain: <b style={{ color: '#a78bfa' }}>SimEngine 50Hz (Browser)</b></span>
                    </div>
                  ) : (
                    <div style={{
                      marginTop: '6px', padding: '6px 8px',
                      background: 'rgba(255,255,255,0.03)',
                      borderRadius: '6px', fontSize: '10px',
                      color: 'var(--text-muted)',
                      display: 'grid', gridTemplateColumns: '1fr', gap: '3px',
                    }}>
                      <span>
                        Brain: <b style={{ color: 'var(--text-primary)' }}>
                          {(robot.telemetry?.hitl || robot.connection?.hitlEnabled) ? 'HITL (Hybrid)' :
                           robot.telemetry.architecture === 'pc_slam' ? 'PC-first SLAM' : 'ESP32 Onboard'}
                        </b>
                      </span>
                      <span>
                        Streams: grid <b style={{ color: robot.telemetry.gridStreamEnabled ? 'var(--accent-success)' : 'var(--accent-warning)' }}>
                          {robot.telemetry.gridStreamEnabled ? 'ON' : 'OFF'}
                        </b>, nav <b style={{ color: robot.telemetry.onboardNavEnabled ? 'var(--accent-success)' : 'var(--accent-warning)' }}>
                          {robot.telemetry.onboardNavEnabled ? 'ESP32' : 'PC'}
                        </b>
                      </span>
                    </div>
                  )}

                  {/* Nav Mode Selector — Only for REAL robots */}
                  {!robot._sim && isConnected && (() => {
                    const currentNavMode = getNavMode(robot.id);
                    return (
                      <div style={{
                        marginTop: '6px', padding: '6px 8px',
                        background: 'rgba(139, 92, 246, 0.06)',
                        borderRadius: '6px', fontSize: '10px',
                        border: '1px solid rgba(139, 92, 246, 0.15)',
                      }}>
                        <div style={{ color: 'var(--text-muted)', marginBottom: '4px', fontWeight: 600 }}>
                          🧠 Chế độ Điều hướng
                        </div>
                        <div style={{ display: 'flex', gap: '4px' }}>
                          <button
                            className={`btn btn--sm`}
                            style={{
                              flex: 1, fontSize: '9px', padding: '4px 6px',
                              background: currentNavMode === 'onboard' ? 'rgba(16,185,129,0.2)' : 'transparent',
                              color: currentNavMode === 'onboard' ? '#6ee7b7' : 'var(--text-muted)',
                              border: currentNavMode === 'onboard' ? '1px solid rgba(16,185,129,0.4)' : '1px solid var(--border-subtle)',
                            }}
                            onClick={(e) => { e.stopPropagation(); setNavMode(robot.id, 'onboard'); }}
                            title="ESP32 tự tìm đường bằng A* + DWA onboard. PC chỉ hiển thị."
                          >
                            📡 ESP32 Onboard
                          </button>
                          <button
                            className={`btn btn--sm`}
                            style={{
                              flex: 1, fontSize: '9px', padding: '4px 6px',
                              background: currentNavMode === 'pc' ? 'rgba(139,92,246,0.2)' : 'transparent',
                              color: currentNavMode === 'pc' ? '#c4b5fd' : 'var(--text-muted)',
                              border: currentNavMode === 'pc' ? '1px solid rgba(139,92,246,0.4)' : '1px solid var(--border-subtle)',
                            }}
                            onClick={(e) => { e.stopPropagation(); setNavMode(robot.id, 'pc'); }}
                            title="Browser tính đường A* + Pure Pursuit. Cần có map SLAM."
                          >
                            🖥️ PC (Browser)
                          </button>
                        </div>
                        <div style={{ fontSize: '8px', color: '#64748b', marginTop: '3px' }}>
                          {currentNavMode === 'onboard'
                            ? 'Robot tự tìm đường — PC chỉ hiển thị telemetry'
                            : 'Browser tính path A* + gửi waypoints cho ESP32'}
                        </div>
                      </div>
                    );
                  })()}

                  {/* INA3221 Power Monitor */}
                  {robot.telemetry.battV > 0 && (
                    <div style={{
                      marginTop: '6px',
                      padding: '6px 8px',
                      background: 'rgba(255,255,255,0.03)',
                      borderRadius: '6px',
                      fontSize: '10px',
                      color: 'var(--text-muted)',
                      display: 'grid',
                      gridTemplateColumns: '1fr 1fr',
                      gap: '3px 12px',
                    }}>
                      <span>🔋 Pin: <b style={{ color: 'var(--text-primary)' }}>{robot.telemetry.battV.toFixed(1)}V</b></span>
                      <span>⚡ {robot.telemetry.battA.toFixed(2)}A</span>
                      <span>⚙️ Motor: <b style={{ color: 'var(--text-primary)' }}>{robot.telemetry.motorV.toFixed(1)}V</b></span>
                      <span>⚡ {robot.telemetry.motorA.toFixed(2)}A</span>
                    </div>
                  )}
                </>
              )}

              <div style={{ display: 'flex', gap: '6px', marginTop: '8px' }}>
                {isConnected ? (
                  <button
                    className="btn btn--ghost btn--sm btn--full"
                    onClick={(e) => { e.stopPropagation(); disconnectRobot(robot.id); }}
                  >
                    {vi.connection.disconnect}
                  </button>
                ) : (
                  <button
                    className="btn btn--primary btn--sm btn--full"
                    onClick={(e) => { e.stopPropagation(); connectRobot(robot.id); }}
                  >
                    {vi.connection.connect}
                  </button>
                )}
                <button
                  className="btn btn--danger btn--sm btn--icon"
                  onClick={(e) => { e.stopPropagation(); removeRobot(robot.id); }}
                  title={vi.connection.remove}
                >
                  ✕
                </button>
              </div>

              {/* HITL Mode Toggle (visible only when connected to a physical robot) */}
              {isConnected && !robot.id.startsWith('sim_') && (
                <button
                  className={`btn btn--sm btn--full`}
                  style={{
                    marginTop: '8px',
                    fontSize: '11px',
                    background: (robot?.telemetry?.hitl || robot?.connection?.hitlEnabled) ? 'var(--accent-info)' : 'transparent',
                    borderColor: 'var(--accent-info)',
                    color: (robot?.telemetry?.hitl || robot?.connection?.hitlEnabled) ? '#fff' : 'var(--accent-info)',
                  }}
                  onClick={(e) => {
                    e.stopPropagation();
                    const next = !(robot?.telemetry?.hitl || robot?.connection?.hitlEnabled);
                    useRobotStore.getState().toggleHitlMode(robot.id, next);
                  }}
                >
                  {(robot?.telemetry?.hitl || robot?.connection?.hitlEnabled) ? '🌐 HITL: Đang giả lập (ON)' : '🔌 Bật HITL (Hardware-in-Loop)'}
                </button>
              )}
            </div>
          );
        })}
      </div>

      {/* Joystick — điều khiển tay robot đang chọn */}
      {selectedRobotId && robots[selectedRobotId]?.status === 'connected' && (
        <>
          <div className="panel__divider" />
          <div className="panel__header">
            <span className="panel__title">🎮 {vi.robot.controls.forward}</span>
          </div>
          <JoystickControl robotId={selectedRobotId} />
        </>
      )}
    </div>
  );
}

/**
 * Joystick Control Component
 */
function JoystickControl({ robotId }) {
  const { sendManualControl, stopRobot, resetOdometry, setPose, recalibrateGyro, setBrake } = useRobotStore();
  const robots = useRobotStore((s) => s.robots);
  const robot = robots[robotId];
  const isSim = !!robot?._sim;
  const imuCalibrated = robot?.telemetry?.imuCalibrated ?? false;
  const imuAvailable = !isSim && (robot?.telemetry?.imuAvailable ?? false);
  const [brakeOn, setBrakeOn] = useState(false);

  const maxLin = 0.3;
  const maxAng = 1.5;

  const moveIntervalRef = useRef(null);

  const startMove = (lin, ang) => {
    sendManualControl(robotId, lin * maxLin, ang * maxAng);
    if (moveIntervalRef.current) clearInterval(moveIntervalRef.current);
    moveIntervalRef.current = setInterval(() => {
      sendManualControl(robotId, lin * maxLin, ang * maxAng);
    }, 100);
  };

  const stopMove = () => {
    if (moveIntervalRef.current) clearInterval(moveIntervalRef.current);
    moveIntervalRef.current = null;
    sendManualControl(robotId, 0, 0);
  };

  // Keyboard WASD Controls
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA' || document.activeElement.tagName === 'SELECT') return;
      
      switch(e.key.toLowerCase()) {
        case 'w': startMove(1, 0); break;
        case 's': startMove(-1, 0); break;
        case 'a': startMove(0, 1); break; // Xoay trái (spin dương theo radian)
        case 'd': startMove(0, -1); break; // Xoay phải (spin âm theo radian)
        case ' ': stopMove(); break; // Space = Stop
      }
    };

    const handleKeyUp = (e) => {
      if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA' || document.activeElement.tagName === 'SELECT') return;
      if (['w','a','s','d'].includes(e.key.toLowerCase())) {
        stopMove();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      if (moveIntervalRef.current) clearInterval(moveIntervalRef.current);
    };
  }, [robotId]);

  const [initChargerId, setInitChargerId] = useState('charger_1');

  const applyPose = () => {
    const charger = CHARGING_STATIONS.find(c => c.id === initChargerId);
    if (!charger) return;
    
    // heading in degrees (270 degrees) needs to be converted to radians for the 2D engine
    // though the ESP32 might just take radians. In main.cpp "robotTheta = doc["theta"]".
    // 270 degrees = 1.5 * PI (or -0.5 * PI).
    const rotRad = (charger.heading * Math.PI) / 180.0;
    
    setPose(robotId, charger.x, charger.y, rotRad);
  };

  return (
    <div>
      <div className="joystick">
        <div /> {/* empty */}
        <button className="joystick__btn" onMouseDown={() => startMove(1, 0)} onMouseUp={stopMove} onMouseLeave={stopMove} onTouchStart={(e) => { e.preventDefault(); startMove(1, 0); }} onTouchEnd={stopMove}>▲</button>
        <div /> {/* empty */}
        <button className="joystick__btn" onMouseDown={() => startMove(0, 1)} onMouseUp={stopMove} onMouseLeave={stopMove} onTouchStart={(e) => { e.preventDefault(); startMove(0, 1); }} onTouchEnd={stopMove}>◄</button>
        <button className="joystick__btn joystick__btn--stop" onClick={stopMove} onTouchStart={(e) => { e.preventDefault(); stopMove(); }}>STOP</button>
        <button className="joystick__btn" onMouseDown={() => startMove(0, -1)} onMouseUp={stopMove} onMouseLeave={stopMove} onTouchStart={(e) => { e.preventDefault(); startMove(0, -1); }} onTouchEnd={stopMove}>►</button>
        <div /> {/* empty */}
        <button className="joystick__btn" onMouseDown={() => startMove(-1, 0)} onMouseUp={stopMove} onMouseLeave={stopMove} onTouchStart={(e) => { e.preventDefault(); startMove(-1, 0); }} onTouchEnd={stopMove}>▼</button>
        <div /> {/* empty */}
      </div>
      
      <div style={{ marginTop: '16px', display: 'flex', flexDirection: 'column', gap: '8px' }}>
        <div style={{ fontSize: '13px', fontWeight: 'bold' }}>Xác định vị trí nguồn:</div>
        <div style={{ display: 'flex', gap: '8px' }}>
          <select 
            className="input input--sm" 
            style={{ flex: 1 }}
            value={initChargerId}
            onChange={(e) => setInitChargerId(e.target.value)}
          >
            {CHARGING_STATIONS.map(c => (
              <option key={c.id} value={c.id}>{c.name}</option>
            ))}
          </select>
          <button className="btn btn--primary btn--sm" onClick={applyPose}>
            Gán Pose
          </button>
        </div>
        <button
          className="btn btn--ghost btn--sm btn--full"
          style={{ marginTop: '4px' }}
          onClick={() => resetOdometry(robotId)}
        >
          🔄 {vi.robot.controls.resetOdom}
        </button>

        {/* IMU Status + Recalibrate */}
        {imuAvailable && (
          <div style={{ marginTop: '8px' }}>
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: '6px' }}>
              <span style={{ fontSize: '11px', color: 'var(--text-muted)' }}>
                IMU: <span style={{ color: imuCalibrated ? 'var(--accent-success)' : 'var(--accent-warning)', fontWeight: '600' }}>
                  {imuCalibrated ? '✓ Đã Calibrate' : '⚠ Chưa Calibrate'}
                </span>
              </span>
            </div>
            <button
              className="btn btn--ghost btn--sm btn--full"
              style={{ fontSize: '11px', borderColor: 'var(--accent-warning)', color: 'var(--accent-warning)' }}
              onClick={() => {
                if (window.confirm('Robot phải đứng YÊN hoàn toàn trước khi calibrate gyro. Tiếp tục?')) {
                  recalibrateGyro(robotId);
                }
              }}
            >
              🧭 Recalibrate Gyro
            </button>
          </div>
        )}

        {/* Emergency Brake Toggle — Hardware only (hide for SimBot) */}
        {!isSim && (
          <button
            className={`btn btn--sm btn--full`}
            style={{
              marginTop: '8px',
              fontSize: '11px',
              background: brakeOn ? 'var(--accent-danger)' : 'transparent',
              borderColor: 'var(--accent-danger)',
              color: brakeOn ? '#fff' : 'var(--accent-danger)',
            }}
            onClick={() => {
              const next = !brakeOn;
              setBrakeOn(next);
              setBrake(robotId, next);
            }}
          >
            {brakeOn ? '🔴 Brake ON — Nhấn để Mở' : '⛔ Khóa Phanh (Brake)'}
          </button>
        )}
      </div>
    </div>
  );
}
