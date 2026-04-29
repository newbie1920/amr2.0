/**
 * AMR 2.0 — DWA Tuning Panel (Phase 3)
 * 
 * Real-time parameter tuning UI for the DWA Local Planner.
 * Inspired by ROS2 rqt_reconfigure / Nav2 dynamic_reconfigure.
 * 
 * Features:
 *   - Grouped slider controls (Speed, Accel, Simulation, Safety, Scoring, Behavior)
 *   - Preset quick-switch (Cautious / Balanced / Aggressive)
 *   - Save/Load custom presets
 *   - localStorage persistence
 *   - Collapsible accordion groups
 */

import { useState, useCallback } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import './DWATuningPanel.css';

// ── Parameter Definitions ──────────────────────────────────────
const PARAM_GROUPS = [
  {
    id: 'speed',
    label: '🚀 Speed',
    icon: '🚀',
    params: [
      { key: 'maxSpeedTrans', label: 'Max Linear', min: 0.05, max: 1.0, step: 0.01, unit: 'm/s' },
      { key: 'minSpeedTrans', label: 'Min Linear', min: 0.0, max: 0.3, step: 0.01, unit: 'm/s' },
      { key: 'maxSpeedRot', label: 'Max Angular', min: 0.3, max: 3.0, step: 0.05, unit: 'rad/s' },
    ],
  },
  {
    id: 'accel',
    label: '⚡ Acceleration',
    icon: '⚡',
    params: [
      { key: 'maxAccelTrans', label: 'Linear Accel', min: 0.1, max: 2.0, step: 0.05, unit: 'm/s²' },
      { key: 'maxAccelRot', label: 'Angular Accel', min: 0.5, max: 5.0, step: 0.1, unit: 'rad/s²' },
    ],
  },
  {
    id: 'sim',
    label: '🔮 Simulation',
    icon: '🔮',
    params: [
      { key: 'simTime', label: 'Sim Horizon', min: 0.5, max: 4.0, step: 0.1, unit: 's' },
      { key: 'simGranularity', label: 'Sim Step', min: 0.02, max: 0.2, step: 0.01, unit: 's' },
      { key: 'vSamples', label: 'V Samples', min: 3, max: 15, step: 1, unit: '' },
      { key: 'wSamples', label: 'W Samples', min: 5, max: 31, step: 2, unit: '' },
    ],
  },
  {
    id: 'safety',
    label: '🛡 Safety',
    icon: '🛡',
    params: [
      { key: 'robotRadius', label: 'Robot Radius', min: 0.05, max: 0.5, step: 0.01, unit: 'm' },
      { key: 'preferredClearance', label: 'Pref. Clearance', min: 0.1, max: 1.0, step: 0.02, unit: 'm' },
      { key: 'stopOnClearance', label: 'Stop Clearance', min: 0.05, max: 0.5, step: 0.01, unit: 'm' },
    ],
  },
  {
    id: 'scoring',
    label: '🎯 Scoring Biases',
    icon: '🎯',
    params: [
      { key: 'pathDistBias', label: 'Path Distance', min: 0, max: 30, step: 0.5, unit: '' },
      { key: 'goalDistBias', label: 'Goal Distance', min: 0, max: 30, step: 0.5, unit: '' },
      { key: 'goalHeadingBias', label: 'Goal Heading', min: 0, max: 20, step: 0.5, unit: '' },
      { key: 'clearanceBias', label: 'Clearance', min: 0, max: 30, step: 0.5, unit: '' },
      { key: 'speedBias', label: 'Speed Reward', min: 0, max: 10, step: 0.5, unit: '' },
    ],
  },
  {
    id: 'behavior',
    label: '🔄 Behavior',
    icon: '🔄',
    params: [
      { key: 'headingLookahead', label: 'Heading Lookahead', min: 0.2, max: 2.0, step: 0.05, unit: 'm' },
      { key: 'rotateInPlaceAngle', label: 'Rotate-in-Place', min: 0.26, max: 1.57, step: 0.05, unit: 'rad', displayDeg: true },
    ],
  },
];

const BUILTIN_PRESETS = ['cautious', 'balanced', 'aggressive'];

const PRESET_LABELS = {
  cautious: { label: '🐢 Cautious', color: '#4ecdc4' },
  balanced: { label: '⚖️ Balanced', color: '#7c8cf8' },
  aggressive: { label: '🔥 Aggressive', color: '#ff6b6b' },
};

// ── Component ──────────────────────────────────────────────────

function DWATuningPanel() {
  const dwaConfig = useRobotStore((s) => s.dwaConfig);
  const dwaActivePreset = useRobotStore((s) => s.dwaActivePreset);
  const dwaCustomPresets = useRobotStore((s) => s.dwaCustomPresets);
  const setDWAConfig = useRobotStore((s) => s.setDWAConfig);
  const resetDWAConfig = useRobotStore((s) => s.resetDWAConfig);
  const loadDWAPreset = useRobotStore((s) => s.loadDWAPreset);
  const saveDWAPreset = useRobotStore((s) => s.saveDWAPreset);
  const deleteDWAPreset = useRobotStore((s) => s.deleteDWAPreset);

  const [collapsed, setCollapsed] = useState(true);
  const [openGroups, setOpenGroups] = useState({ speed: true }); // First group open by default
  const [showSaveDialog, setShowSaveDialog] = useState(false);
  const [presetName, setPresetName] = useState('');

  const toggleGroup = useCallback((groupId) => {
    setOpenGroups((prev) => ({ ...prev, [groupId]: !prev[groupId] }));
  }, []);

  const handleSliderChange = useCallback((key, value) => {
    setDWAConfig({ [key]: parseFloat(value) });
  }, [setDWAConfig]);

  const handleSavePreset = useCallback(() => {
    const name = presetName.trim().toLowerCase().replace(/\s+/g, '_');
    if (!name || BUILTIN_PRESETS.includes(name)) return;
    saveDWAPreset(name);
    setPresetName('');
    setShowSaveDialog(false);
  }, [presetName, saveDWAPreset]);

  const customPresetNames = Object.keys(dwaCustomPresets);

  return (
    <div className="dwa-panel">
      {/* ── Header ── */}
      <div className="dwa-panel__header" onClick={() => setCollapsed(!collapsed)}>
        <div className="dwa-panel__header-left">
          <span className="dwa-panel__icon">🎛</span>
          <span className="dwa-panel__title">DWA Tuning</span>
          <span className={`dwa-panel__preset-badge dwa-panel__preset-badge--${dwaActivePreset}`}>
            {PRESET_LABELS[dwaActivePreset]?.label || `📌 ${dwaActivePreset}`}
          </span>
        </div>
        <span className={`dwa-panel__chevron ${collapsed ? '' : 'dwa-panel__chevron--open'}`}>▸</span>
      </div>

      {!collapsed && (
        <div className="dwa-panel__body">
          {/* ── Preset Buttons ── */}
          <div className="dwa-panel__presets">
            {BUILTIN_PRESETS.map((name) => (
              <button
                key={name}
                className={`dwa-preset-btn ${dwaActivePreset === name ? 'dwa-preset-btn--active' : ''}`}
                style={{ '--preset-color': PRESET_LABELS[name].color }}
                onClick={() => loadDWAPreset(name)}
              >
                {PRESET_LABELS[name].label}
              </button>
            ))}
          </div>

          {/* ── Custom Presets ── */}
          {customPresetNames.length > 0 && (
            <div className="dwa-panel__custom-presets">
              {customPresetNames.map((name) => (
                <div key={name} className="dwa-custom-preset">
                  <button
                    className={`dwa-preset-btn dwa-preset-btn--custom ${dwaActivePreset === name ? 'dwa-preset-btn--active' : ''}`}
                    onClick={() => loadDWAPreset(name)}
                  >
                    📌 {name}
                  </button>
                  <button
                    className="dwa-preset-btn--delete"
                    onClick={() => deleteDWAPreset(name)}
                    title="Delete preset"
                  >
                    ×
                  </button>
                </div>
              ))}
            </div>
          )}

          {/* ── Parameter Groups ── */}
          {PARAM_GROUPS.map((group) => (
            <div key={group.id} className="dwa-group">
              <div
                className="dwa-group__header"
                onClick={() => toggleGroup(group.id)}
              >
                <span className="dwa-group__label">{group.label}</span>
                <span className={`dwa-group__chevron ${openGroups[group.id] ? 'dwa-group__chevron--open' : ''}`}>▸</span>
              </div>

              {openGroups[group.id] && (
                <div className="dwa-group__body">
                  {group.params.map((param) => {
                    const value = dwaConfig[param.key] ?? 0;
                    const displayValue = param.displayDeg
                      ? `${(value * 180 / Math.PI).toFixed(0)}°`
                      : `${value.toFixed(param.step < 0.1 ? 2 : 1)}${param.unit ? ' ' + param.unit : ''}`;

                    return (
                      <div key={param.key} className="dwa-param">
                        <div className="dwa-param__header">
                          <label className="dwa-param__label">{param.label}</label>
                          <span className="dwa-param__value">{displayValue}</span>
                        </div>
                        <input
                          type="range"
                          className="dwa-param__slider"
                          min={param.min}
                          max={param.max}
                          step={param.step}
                          value={value}
                          onChange={(e) => handleSliderChange(param.key, e.target.value)}
                        />
                      </div>
                    );
                  })}
                </div>
              )}
            </div>
          ))}

          {/* ── Actions ── */}
          <div className="dwa-panel__actions">
            <button className="dwa-action-btn dwa-action-btn--reset" onClick={resetDWAConfig}>
              ↺ Reset
            </button>
            <button
              className="dwa-action-btn dwa-action-btn--save"
              onClick={() => setShowSaveDialog(!showSaveDialog)}
            >
              💾 Save Preset
            </button>
          </div>

          {/* ── Save Dialog ── */}
          {showSaveDialog && (
            <div className="dwa-save-dialog">
              <input
                type="text"
                className="dwa-save-dialog__input"
                placeholder="Preset name..."
                value={presetName}
                onChange={(e) => setPresetName(e.target.value)}
                onKeyDown={(e) => e.key === 'Enter' && handleSavePreset()}
                autoFocus
              />
              <button className="dwa-save-dialog__btn" onClick={handleSavePreset}>
                Save
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

export default DWATuningPanel;
