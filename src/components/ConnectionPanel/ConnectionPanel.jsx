/**
 * AMR 2.0 — Connection Panel Component
 * Panel nhập IP kết nối robot ESP32-S3
 */

import { useState, useEffect } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import vi from '../../i18n/vi.js';
import { CHARGING_STATIONS } from '../../core/warehouse.js';

export default function ConnectionPanel() {
  const [name, setName] = useState('');
  const [ip, setIp] = useState('192.168.1.');
  const [port, setPort] = useState('81');
  const [showForm, setShowForm] = useState(false);

  const { addRobot, removeRobot, connectRobot, disconnectRobot, selectRobot, selectedRobotId } = useRobotStore();
  const robots = useRobotStore((s) => s.robots);
  const robotList = Object.values(robots);

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

          return (
            <div
              key={robot.id}
              className={`card robot-card ${isSelected ? 'card--active' : ''}`}
              onClick={() => selectRobot(robot.id)}
            >
              <div className="robot-card__header">
                <span className="robot-card__name">{robot.name}</span>
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
  const { sendManualControl, stopRobot, resetOdometry, setPose } = useRobotStore();

  const maxLin = 0.3;
  const maxAng = 1.5;

  const move = (lin, ang) => sendManualControl(robotId, lin * maxLin, ang * maxAng);
  const stop = () => stopRobot(robotId);

  // Keyboard WASD Controls
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA' || document.activeElement.tagName === 'SELECT') return;
      
      switch(e.key.toLowerCase()) {
        case 'w': move(1, 0); break;
        case 's': move(-1, 0); break;
        case 'a': move(0, 1); break; // Xoay trái (spin dương theo radian)
        case 'd': move(0, -1); break; // Xoay phải (spin âm theo radian)
        case ' ': stop(); break; // Space = Stop
      }
    };

    const handleKeyUp = (e) => {
      if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA' || document.activeElement.tagName === 'SELECT') return;
      if (['w','a','s','d'].includes(e.key.toLowerCase())) {
        stop();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
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
        <button className="joystick__btn" onMouseDown={() => move(1, 0)} onMouseUp={stop} onMouseLeave={stop}>▲</button>
        <div /> {/* empty */}
        <button className="joystick__btn" onMouseDown={() => move(0, 1)} onMouseUp={stop} onMouseLeave={stop}>◄</button>
        <button className="joystick__btn joystick__btn--stop" onClick={stop}>STOP</button>
        <button className="joystick__btn" onMouseDown={() => move(0, -1)} onMouseUp={stop} onMouseLeave={stop}>►</button>
        <div /> {/* empty */}
        <button className="joystick__btn" onMouseDown={() => move(-1, 0)} onMouseUp={stop} onMouseLeave={stop}>▼</button>
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
      </div>
    </div>
  );
}
