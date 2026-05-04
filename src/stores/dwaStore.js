/**
 * AMR 2.0 — DWA Tuning Store (Zustand)
 * Quản lý DWA (Dynamic Window Approach) parameters & presets.
 * Tách từ robotStore.js để giữ single-responsibility.
 */

import { create } from 'zustand';
import { DWA_DEFAULTS, DWA_PRESETS } from '../core/dwaPlanner.js';

const DWA_CONFIG_KEY = 'amr_dwa_config';
const DWA_PRESETS_KEY = 'amr_dwa_presets';

function loadDWAConfigFromStorage() {
  try {
    const raw = localStorage.getItem(DWA_CONFIG_KEY);
    return raw ? { ...DWA_DEFAULTS, ...JSON.parse(raw) } : { ...DWA_DEFAULTS };
  } catch (e) {
    return { ...DWA_DEFAULTS };
  }
}

function loadCustomPresetsFromStorage() {
  try {
    const raw = localStorage.getItem(DWA_PRESETS_KEY);
    return raw ? JSON.parse(raw) : {};
  } catch (e) {
    return {};
  }
}

function loadActivePresetFromStorage() {
  try {
    const raw = localStorage.getItem('amr_dwa_active_preset');
    return raw || 'balanced';
  } catch (e) {
    return 'balanced';
  }
}

const useDWAStore = create((set, get) => ({
  // State
  dwaConfig: loadDWAConfigFromStorage(),
  dwaActivePreset: loadActivePresetFromStorage(),
  dwaCustomPresets: loadCustomPresetsFromStorage(),

  // Actions

  /**
   * Cập nhật một hoặc nhiều thông số DWA
   * @param {object} partial - { maxSpeedTrans: 0.5, clearanceBias: 15, ... }
   */
  setDWAConfig: (partial) => {
    set((s) => {
      const merged = { ...s.dwaConfig, ...partial };
      localStorage.setItem(DWA_CONFIG_KEY, JSON.stringify(merged));
      localStorage.setItem('amr_dwa_active_preset', 'custom');
      return { dwaConfig: merged, dwaActivePreset: 'custom' };
    });
  },

  /**
   * Reset DWA config về giá trị mặc định
   */
  resetDWAConfig: () => {
    localStorage.removeItem(DWA_CONFIG_KEY);
    localStorage.setItem('amr_dwa_active_preset', 'balanced');
    set({ dwaConfig: { ...DWA_DEFAULTS }, dwaActivePreset: 'balanced' });
  },

  /**
   * Tải một preset DWA (cautious | balanced | aggressive | custom)
   */
  loadDWAPreset: (name) => {
    const state = get();
    const preset = DWA_PRESETS[name] || state.dwaCustomPresets[name];
    if (!preset) {
      console.warn(`[DWA] Unknown preset: ${name}`);
      return;
    }
    const config = { ...DWA_DEFAULTS, ...preset };
    localStorage.setItem(DWA_CONFIG_KEY, JSON.stringify(config));
    localStorage.setItem('amr_dwa_active_preset', name);
    set({ dwaConfig: config, dwaActivePreset: name });
  },

  /**
   * Lưu config hiện tại thành custom preset
   */
  saveDWAPreset: (name) => {
    const state = get();
    const updated = { ...state.dwaCustomPresets, [name]: { ...state.dwaConfig } };
    localStorage.setItem(DWA_PRESETS_KEY, JSON.stringify(updated));
    localStorage.setItem('amr_dwa_active_preset', name);
    set({ dwaCustomPresets: updated, dwaActivePreset: name });
  },

  /**
   * Xoá custom preset
   */
  deleteDWAPreset: (name) => {
    const state = get();
    const { [name]: _, ...rest } = state.dwaCustomPresets;
    localStorage.setItem(DWA_PRESETS_KEY, JSON.stringify(rest));
    set({ dwaCustomPresets: rest });
    if (state.dwaActivePreset === name) {
      localStorage.setItem('amr_dwa_active_preset', 'balanced');
      set({ dwaActivePreset: 'balanced' });
    }
  },
}));

export default useDWAStore;
