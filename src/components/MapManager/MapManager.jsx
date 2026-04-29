/**
 * AMR 2.0 — Map Manager Component
 * Bảng điều khiển quản lý bản đồ Lidar đã quét
 * Tính năng: List, Load, Delete, Rename, Import, Export, Preview
 */

import React, { useState, useRef, useCallback, useEffect } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import { exportMapToROS2 } from '../../core/mapExporter.js';

// ============================================================
//   MINI MAP PREVIEW (Canvas-based thumbnail)
// ============================================================

function MapPreview({ mapData, size = 80 }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!canvasRef.current || !mapData) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    canvas.width = mapData.width || 40;
    canvas.height = mapData.height || 40;

    // Decode logOdds from base64 and render
    try {
      const binary = atob(mapData.logOdds);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) {
        bytes[i] = binary.charCodeAt(i);
      }
      const logOdds = new Float32Array(bytes.buffer);
      const imgData = ctx.createImageData(canvas.width, canvas.height);

      for (let gy = 0; gy < canvas.height; gy++) {
        for (let gx = 0; gx < canvas.width; gx++) {
          const idx = gy * canvas.width + gx;
          const lo = logOdds[idx] || 0;
          const pxIdx = ((canvas.height - 1 - gy) * canvas.width + gx) * 4;

          if (lo > 0.5) {
            imgData.data[pxIdx] = 239;
            imgData.data[pxIdx + 1] = 68;
            imgData.data[pxIdx + 2] = 68;
            imgData.data[pxIdx + 3] = Math.round(100 + 155 * Math.min(1, (lo - 0.5) / 4.5));
          } else if (lo < -0.5) {
            imgData.data[pxIdx] = 16;
            imgData.data[pxIdx + 1] = 185;
            imgData.data[pxIdx + 2] = 129;
            imgData.data[pxIdx + 3] = Math.round(40 + 80 * Math.min(1, (-lo - 0.5) / 4.5));
          } else {
            imgData.data[pxIdx] = 51;
            imgData.data[pxIdx + 1] = 65;
            imgData.data[pxIdx + 2] = 85;
            imgData.data[pxIdx + 3] = 30;
          }
        }
      }

      ctx.putImageData(imgData, 0, 0);
    } catch (e) {
      // Fallback: draw placeholder
      ctx.fillStyle = '#1e293b';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#64748b';
      ctx.font = '8px sans-serif';
      ctx.fillText('?', canvas.width / 2 - 3, canvas.height / 2 + 3);
    }
  }, [mapData]);

  return (
    <canvas
      ref={canvasRef}
      style={{
        width: `${size}px`,
        height: `${size}px`,
        borderRadius: '6px',
        border: '1px solid rgba(255,255,255,0.1)',
        imageRendering: 'pixelated',
        background: '#0f172a',
      }}
    />
  );
}

// ============================================================
//   MAP MANAGER PANEL
// ============================================================

export default function MapManager({ onClose }) {
  const savedMaps = useRobotStore((s) => s.savedMaps);
  const deleteSavedMap = useRobotStore((s) => s.deleteSavedMap);
  const renameSavedMap = useRobotStore((s) => s.renameSavedMap);
  const loadSavedMap = useRobotStore((s) => s.loadSavedMap);
  const exportSavedMap = useRobotStore((s) => s.exportSavedMap);
  const importMapFromJSON = useRobotStore((s) => s.importMapFromJSON);
  const selectedRobotId = useRobotStore((s) => s.selectedRobotId);
  const robots = useRobotStore((s) => s.robots);

  const [editingId, setEditingId] = useState(null);
  const [editName, setEditName] = useState('');
  const [confirmDeleteId, setConfirmDeleteId] = useState(null);
  const fileInputRef = useRef(null);

  const activeRobotId = selectedRobotId || Object.keys(robots)[0] || null;

  // Import handler
  const handleImport = useCallback((e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (evt) => {
      try {
        const json = JSON.parse(evt.target.result);
        importMapFromJSON(json, file.name.replace('.json', ''));
      } catch (err) {
        console.error('Lỗi import map:', err);
        alert('File không hợp lệ!');
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  }, [importMapFromJSON]);

  // Rename handler
  const handleStartRename = (map) => {
    setEditingId(map.id);
    setEditName(map.name);
  };

  const handleFinishRename = () => {
    if (editingId && editName.trim()) {
      renameSavedMap(editingId, editName.trim());
    }
    setEditingId(null);
    setEditName('');
  };

  // Time formatting
  const formatTime = (ts) => {
    const d = new Date(ts);
    return `${d.toLocaleDateString('vi-VN')} ${d.toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit' })}`;
  };

  return (
    <div style={styles.overlay}>
      <div style={styles.panel}>
        {/* Header */}
        <div style={styles.header}>
          <div style={styles.headerTitle}>
            <span style={{ fontSize: '18px' }}>🗺️</span>
            <h3 style={styles.title}>Quản lý Bản đồ</h3>
            <span style={styles.badge}>{savedMaps.length}</span>
          </div>
          <button style={styles.closeBtn} onClick={onClose}>✕</button>
        </div>

        {/* Toolbar */}
        <div style={styles.toolbar}>
          <button
            style={styles.importBtn}
            onClick={() => fileInputRef.current?.click()}
          >
            📂 Import Map
          </button>
          <input
            ref={fileInputRef}
            type="file"
            accept=".json"
            style={{ display: 'none' }}
            onChange={handleImport}
          />
          <div style={styles.hint}>
            💡 Map tự động lưu khi dừng quét
          </div>
        </div>

        {/* Map List */}
        <div style={styles.mapList}>
          {savedMaps.length === 0 ? (
            <div style={styles.emptyState}>
              <span style={{ fontSize: '32px' }}>📡</span>
              <p style={{ margin: '8px 0 4px', fontWeight: 600 }}>Chưa có bản đồ nào</p>
              <p style={{ fontSize: '11px', color: '#64748b' }}>
                Chuyển sang chế độ Lidar → Bắt đầu quét → Map sẽ tự động lưu ở đây
              </p>
            </div>
          ) : (
            savedMaps.slice().reverse().map((map) => (
              <div key={map.id} style={styles.mapCard}>
                {/* Preview Thumbnail */}
                <MapPreview mapData={map.data} size={64} />

                {/* Map Info */}
                <div style={styles.mapInfo}>
                  {editingId === map.id ? (
                    <input
                      style={styles.renameInput}
                      value={editName}
                      onChange={(e) => setEditName(e.target.value)}
                      onBlur={handleFinishRename}
                      onKeyDown={(e) => e.key === 'Enter' && handleFinishRename()}
                      autoFocus
                    />
                  ) : (
                    <div
                      style={styles.mapName}
                      onDoubleClick={() => handleStartRename(map)}
                      title="Double-click để đổi tên"
                    >
                      {map.name}
                    </div>
                  )}
                  <div style={styles.mapMeta}>
                    📊 {map.scanCount} scans &nbsp;·&nbsp; 📐 {map.width}×{map.height} ({map.resolution}m)
                  </div>
                  <div style={styles.mapMeta}>
                    🕐 {formatTime(map.createdAt)}
                  </div>
                </div>

                {/* Actions */}
                <div style={styles.mapActions}>
                  <button
                    style={styles.actionBtn}
                    title="Áp dụng map này cho robot"
                    onClick={() => {
                      if (activeRobotId) {
                        loadSavedMap(map.id, activeRobotId);
                      } else {
                        alert('Chưa có robot nào được chọn');
                      }
                    }}
                  >
                    ▶️
                  </button>
                  <button
                    style={styles.actionBtn}
                    title="Export ra file JSON"
                    onClick={() => exportSavedMap(map.id)}
                  >
                    JSON
                  </button>
                  <button
                    style={{...styles.actionBtn, fontWeight: 'bold'}}
                    title="Export chuẩn ROS2 (PGM/YAML)"
                    onClick={() => exportMapToROS2(map, map.name || 'warehouse_map')}
                  >
                    ROS2
                  </button>
                  <button
                    style={styles.actionBtn}
                    title="Đổi tên"
                    onClick={() => handleStartRename(map)}
                  >
                    ✏️
                  </button>
                  {confirmDeleteId === map.id ? (
                    <button
                      style={{ ...styles.actionBtn, ...styles.deleteConfirmBtn }}
                      onClick={() => {
                        deleteSavedMap(map.id);
                        setConfirmDeleteId(null);
                      }}
                    >
                      🗑️ Xác nhận?
                    </button>
                  ) : (
                    <button
                      style={{ ...styles.actionBtn, color: '#ef4444' }}
                      title="Xóa map"
                      onClick={() => setConfirmDeleteId(map.id)}
                    >
                      🗑️
                    </button>
                  )}
                </div>
              </div>
            ))
          )}
        </div>

        {/* Footer */}
        <div style={styles.footer}>
          <span style={{ fontSize: '10px', color: '#475569' }}>
            Maps được lưu trong localStorage của trình duyệt
          </span>
        </div>
      </div>
    </div>
  );
}

// ============================================================
//   STYLES
// ============================================================

const styles = {
  overlay: {
    position: 'fixed',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    background: 'rgba(0,0,0,0.6)',
    backdropFilter: 'blur(4px)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 9999,
  },
  panel: {
    background: 'linear-gradient(135deg, #0f172a 0%, #1e293b 100%)',
    borderRadius: '16px',
    border: '1px solid rgba(255,255,255,0.1)',
    width: '480px',
    maxWidth: '95vw',
    maxHeight: '80vh',
    display: 'flex',
    flexDirection: 'column',
    boxShadow: '0 20px 60px rgba(0,0,0,0.5)',
    overflow: 'hidden',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '16px 20px',
    borderBottom: '1px solid rgba(255,255,255,0.08)',
  },
  headerTitle: {
    display: 'flex',
    alignItems: 'center',
    gap: '8px',
  },
  title: {
    margin: 0,
    fontSize: '15px',
    fontWeight: 700,
    color: '#f1f5f9',
  },
  badge: {
    background: '#3b82f6',
    color: 'white',
    borderRadius: '10px',
    padding: '2px 8px',
    fontSize: '11px',
    fontWeight: 700,
  },
  closeBtn: {
    background: 'none',
    border: 'none',
    color: '#94a3b8',
    fontSize: '18px',
    cursor: 'pointer',
    padding: '4px 8px',
    borderRadius: '6px',
    transition: 'all 0.15s',
  },
  toolbar: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '10px 20px',
    borderBottom: '1px solid rgba(255,255,255,0.05)',
  },
  importBtn: {
    background: 'linear-gradient(135deg, #3b82f6, #2563eb)',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    padding: '6px 14px',
    fontSize: '12px',
    fontWeight: 600,
    cursor: 'pointer',
    transition: 'all 0.2s',
  },
  hint: {
    fontSize: '10px',
    color: '#64748b',
  },
  mapList: {
    flex: 1,
    overflowY: 'auto',
    padding: '8px 12px',
  },
  emptyState: {
    textAlign: 'center',
    padding: '40px 20px',
    color: '#94a3b8',
  },
  mapCard: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    padding: '10px 12px',
    background: 'rgba(255,255,255,0.03)',
    borderRadius: '10px',
    marginBottom: '6px',
    border: '1px solid rgba(255,255,255,0.05)',
    transition: 'all 0.15s',
  },
  mapInfo: {
    flex: 1,
    minWidth: 0,
  },
  mapName: {
    fontSize: '13px',
    fontWeight: 600,
    color: '#e2e8f0',
    cursor: 'pointer',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  mapMeta: {
    fontSize: '10px',
    color: '#64748b',
    marginTop: '2px',
  },
  renameInput: {
    width: '100%',
    background: '#1e293b',
    border: '1px solid #3b82f6',
    borderRadius: '4px',
    padding: '3px 6px',
    color: '#f1f5f9',
    fontSize: '12px',
    fontWeight: 600,
    outline: 'none',
  },
  mapActions: {
    display: 'flex',
    flexDirection: 'column',
    gap: '3px',
    flexShrink: 0,
  },
  actionBtn: {
    background: 'rgba(255,255,255,0.06)',
    border: 'none',
    borderRadius: '6px',
    padding: '4px 8px',
    fontSize: '12px',
    cursor: 'pointer',
    transition: 'all 0.15s',
    color: '#94a3b8',
  },
  deleteConfirmBtn: {
    background: 'rgba(239,68,68,0.15)',
    color: '#ef4444',
    fontSize: '10px',
    fontWeight: 700,
  },
  footer: {
    padding: '8px 20px',
    borderTop: '1px solid rgba(255,255,255,0.05)',
    textAlign: 'center',
  },
};
