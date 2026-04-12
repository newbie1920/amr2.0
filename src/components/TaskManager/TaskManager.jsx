/**
 * AMR 2.0 — Task Manager Component
 * Quản lý nhiệm vụ nhập/xuất hàng
 */

import { useState, useEffect } from 'react';
import useTaskStore from '../../stores/taskStore.js';
import useRobotStore from '../../stores/robotStore.js';
import { SHELVES, GATES, findSlotById } from '../../core/warehouse.js';
import { findPath } from '../../core/pathfinder.js';
import vi from '../../i18n/vi.js';
import { supabase } from '../../utils/supabaseClient.js';

// === VISUAL SHELF COMPONENT ===
function VisualShelfSelector({ inventory, selectedSlotId, onSelectSlot, taskType }) {
  // Helpers to draw shelf visually
  return (
    <div style={{ marginTop: '12px', padding: '10px', background: 'var(--bg-lighter)', borderRadius: 'var(--radius-md)' }}>
      <div style={{ fontSize: '11px', color: 'var(--text-muted)', marginBottom: '8px', textAlign: 'center' }}>
        {taskType === 'import' ? '👇 Bấm vào ô TRỐNG để chọn vị trí cất hàng' : '👇 Bấm vào ô CÓ HÀNG để chọn món xuất kho'}
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px', paddingBottom: '8px' }}>
        {SHELVES.map(shelf => (
          <div key={shelf.id} style={{ background: 'var(--bg-dark)', padding: '8px', borderRadius: '6px', border: '1px solid var(--border)' }}>
            <div style={{ fontSize: '12px', fontWeight: 'bold', textAlign: 'center', marginBottom: '8px', color: 'var(--text-primary)' }}>{shelf.name}</div>
            
            {/* Render levels from top to bottom (Level 2 then Level 1) */}
            {[...shelf.levels].reverse().map(l => (
              <div key={l.level} style={{ display: 'flex', gap: '4px', marginBottom: '4px', justifyContent: 'center' }}>
                 {l.slots.map(slot => {
                   const itemInSlot = inventory.find(i => i.slot_id === slot.id);
                   const isSelected = selectedSlotId === slot.id;
                   
                   // Logic
                   const isOccupied = !!itemInSlot;
                   const canSelect = taskType === 'import' ? !isOccupied : isOccupied;
                   
                   // styling
                   let bgColor = 'rgba(255,255,255,0.03)';
                   let borderColor = 'rgba(255,255,255,0.1)';
                   let textColor = 'var(--text-secondary)';
                   
                   if (isOccupied) {
                     bgColor = 'rgba(239, 68, 68, 0.15)'; // redish
                     borderColor = 'var(--accent-danger)';
                     textColor = 'var(--text-primary)';
                   }
                   
                   if (isSelected) {
                     borderColor = 'var(--accent-primary)';
                     bgColor = 'rgba(5b, 130, 246, 0.2)'; // blueish selection
                     if (isOccupied) bgColor = 'rgba(239, 68, 68, 0.4)'; 
                   }
                   
                   return (
                     <div 
                       key={slot.id}
                       onClick={() => canSelect && onSelectSlot(slot.id, itemInSlot)}
                       style={{
                         flex: 1, minWidth: 0, height: '40px', 
                         display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                         border: `1px solid ${borderColor}`,
                         background: bgColor,
                         borderRadius: '4px',
                         cursor: canSelect ? 'pointer' : 'not-allowed',
                         opacity: canSelect ? 1 : 0.5,
                         transition: 'all 0.2s',
                         overflow: 'hidden'
                       }}
                       title={isOccupied ? `${itemInSlot.name} (SL:${itemInSlot.quantity})` : 'Trống'}
                     >
                       <span style={{ fontSize: '9px', color: 'var(--text-muted)' }}>{l.name} - Ô {slot.id.split('_').pop()}</span>
                       <span style={{ fontSize: '10px', fontWeight: 'bold', color: textColor, whiteSpace: 'nowrap', textOverflow: 'ellipsis', overflow: 'hidden', width: '100%', textAlign: 'center' }}>
                         {isOccupied ? itemInSlot.sku : '---'}
                       </span>
                     </div>
                   );
                 })}
              </div>
            ))}
          </div>
        ))}
      </div>
    </div>
  );
}

export default function TaskManager({ onPathGenerated }) {
  const [taskType, setTaskType] = useState('import');
  const [showForm, setShowForm] = useState(false);
  
  // WMS Form States
  const [inventory, setInventory] = useState([]);
  const [loadingInv, setLoadingInv] = useState(false);
  
  const [importType, setImportType] = useState('new'); // 'new' | 'existing'
  const [selectedSku, setSelectedSku] = useState('');
  const [newSku, setNewSku] = useState('');
  const [newName, setNewName] = useState('');
  const [requireSku, setRequireSku] = useState(true);
  const [quantity, setQuantity] = useState(1);
  const [userSelectedSlotId, setUserSelectedSlotId] = useState('');
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedGateId, setSelectedGateId] = useState('');

  const { createTask, tasks } = useTaskStore();
  const robots = useRobotStore((s) => s.robots);
  const connectedRobots = Object.values(robots).filter(r => r.status === 'connected');

  // Load Inventory from Supabase
  const loadInventory = async () => {
    setLoadingInv(true);
    try {
      const { data, error } = await supabase.from('inventory').select('*');
      if (!error && data) {
        setInventory(data);
      }
    } catch (e) {
      console.log('No supabase connection yet', e);
    }
    setLoadingInv(false);
  };

  useEffect(() => {
    loadInventory();
  }, [showForm]);

  const handleSelectSlotVisual = (slotId, itemInSlot) => {
    setUserSelectedSlotId(slotId);
    if (taskType === 'export' && itemInSlot) {
      setSelectedSku(itemInSlot.sku);
    }
  };

  const handleSelectExistingItem = (sku) => {
    setSelectedSku(sku);
    const item = inventory.find(i => i.sku === sku);
    if (item) {
      // automatically highlight its slot
      setUserSelectedSlotId(item.slot_id);
    }
  };

  const handleCreateTask = async () => {
    let targetSlotId = null;
    let finalSku = '';
    let finalName = '';

    if (!selectedGateId) return alert('Vui lòng chọn cổng (Gate)!');

    if (taskType === 'import') {
      if (importType === 'new') {
        if (!userSelectedSlotId) return alert('Vui lòng chọn 1 ô trống trên Kệ để chứa hàng mới!');
        if (!newName) return alert('Vui lòng nhập Tên Hàng Hóa!');
        if (requireSku && !newSku) return alert('Vui lòng nhập Mã SKU!');
        
        targetSlotId = userSelectedSlotId;
        finalSku = newSku || `FREERUN-${Date.now()}`;
        finalName = newName;
      } else {
        targetSlotId = inventory.find(i => i.sku === selectedSku)?.slot_id;
        if (!targetSlotId) return alert('Vui lòng chọn 1 mã hàng có sẵn!');
        finalSku = selectedSku;
        finalName = inventory.find(i => i.sku === selectedSku)?.name;
      }
      
      // Chúng ta KHÔNG cập nhật Supabase ở đây nữa mà để vào orderInfo
      // Database chỉ update khi xe thực sự báo NAV_DONE
    } else {
      // Export
      const invItem = inventory.find(i => i.sku === selectedSku);
      if (!invItem) return alert('Vui lòng chọn hàng để xuất kho!');
      targetSlotId = invItem.slot_id;
    }

    if (!targetSlotId) return;

    const task = createTask(taskType, targetSlotId, {
      sku: finalSku || selectedSku,
      name: finalName || inventory.find(i => i.sku === selectedSku)?.name,
      qty: quantity,
      importType: taskType === 'import' ? importType : undefined
    });

    // Auto routing → Gửi lộ trình xuống ESP32, xe tự lái
    if (connectedRobots.length > 0) {
      const robot = connectedRobots[0];
      const slotInfo = findSlotById(targetSlotId);
      if (slotInfo) {
        // Bước 1: Tìm đường A* từ xe → ô kệ
        const result = findPath(
          robot.telemetry.x, robot.telemetry.y,
          slotInfo.slot.approach.x, slotInfo.slot.approach.y
        );
        if (result.success) {
          // Vẽ đường trên bản đồ 3D
          if (onPathGenerated) onPathGenerated(result.path);
          
          // Bước 2: Gửi path + heading xuống ESP32
          const { navigateRobot } = useRobotStore.getState();
          navigateRobot(robot.id, result.path, slotInfo.slot.heading || 90);
          
          // Gắn robot vào task và đổi trạng thái
          const { updateTask } = useTaskStore.getState();
          updateTask(task.id, { 
            status: 'in_progress', 
            assignedRobotId: robot.id,
            // Đánh dấu để check 1 lần duy nhất khi DONE
            dbUpdated: false 
          });
        }
      }
    }

    // Reset
    setShowForm(false);
    setNewSku('');
    setNewName('');
    setSelectedSku('');
    setUserSelectedSlotId('');
    setSelectedGateId('');
    setSearchQuery('');
    setQuantity(1);
  };

  // ==========================================
  // SYNC LOOP: ĐỢI LỆNH DONE TỪ ROBOT THÌ MỚI UPDATE DB
  // ==========================================
  useEffect(() => {
    const checkCompletedTasks = async () => {
      const { tasks, updateTask } = useTaskStore.getState();
      const { robots } = useRobotStore.getState();

      const activeList = tasks.filter(t => t.status === 'in_progress' && t.assignedRobotId);

      for (const t of activeList) {
        const robot = robots[t.assignedRobotId];
        if (!robot || !robot.telemetry) continue;
        
        const navState = robot.telemetry.nav;
        
        // ⛔ Robot bị lỗi (timeout/kẹt) → đánh dấu task thất bại
        if (navState === 'ERROR' && !t.dbUpdated) {
          updateTask(t.id, { 
            dbUpdated: true, 
            status: 'failed', 
            error: 'Robot navigation timeout — xe bị kẹt hoặc mất tọa độ' 
          });
          console.warn(`[Task Sync] Task ${t.id} FAILED — NAV_ERROR`);
          continue;
        }
        
        // ✅ Robot đã tới đích → update database
        if (navState === 'DONE' && !t.dbUpdated) {
          
          // 1. Phải khóa lại ngay tránh chạy 2 lần
          updateTask(t.id, { dbUpdated: true });

          try {
            const info = t.orderInfo;
            if (t.type === 'import') {
              if (info.importType === 'new') {
                await supabase.from('inventory').insert([{
                  sku: info.sku,
                  name: info.name,
                  quantity: info.qty,
                  slot_id: t.slotId
                }]);
              } else {
                const currentQty = inventory.find(i => i.sku === info.sku)?.quantity || 0;
                await supabase.from('inventory').update({ quantity: currentQty + info.qty }).eq('sku', info.sku);
              }
            } else {
              // Export
              const invItem = inventory.find(i => i.sku === info.sku);
              if (invItem) {
                if (info.qty >= invItem.quantity) {
                  await supabase.from('inventory').delete().eq('sku', info.sku);
                } else {
                  await supabase.from('inventory').update({ quantity: invItem.quantity - info.qty }).eq('sku', info.sku);
                }
              }
            }
            
            // 2. Chuyển task sang Hoàn thành
            updateTask(t.id, { status: 'completed', completedAt: Date.now() });
            loadInventory();
            console.log(`[Task Sync] Task ${t.id} completed. DB Updated!`);

          } catch (e) {
            console.error('Lỗi khi update database:', e);
            updateTask(t.id, { status: 'failed', error: e.message });
          }
        }
      }
    };

    // Chạy loop check liên tục mỗi 1s
    const interval = setInterval(checkCompletedTasks, 1000);
    return () => clearInterval(interval);
  }, [inventory, connectedRobots]); // Phụ thuộc vào robots để react khi telemetry đổi


  const activeTasks = tasks.filter(t => t.status === 'in_progress' || t.status === 'assigned');
  const pendingTasks = tasks.filter(t => t.status === 'pending');
  const completedTasks = tasks.filter(t => t.status === 'completed').slice(-5);
  
  const filteredInventory = inventory.filter(i => 
    i.name.toLowerCase().includes(searchQuery.toLowerCase()) || 
    i.sku.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <div className="panel">
      <div className="panel__header">
        <span className="panel__title">📋 Quản Lý Nhiệm Vụ (WMS)</span>
      </div>

      <div style={{ display: 'flex', gap: '8px', marginBottom: '12px' }}>
        <button
          className={`btn btn--full ${taskType === 'import' ? 'btn--primary' : 'btn--ghost'}`}
          onClick={() => { setTaskType('import'); setShowForm(true); setImportType('new'); setUserSelectedSlotId(''); setSelectedSku(''); setSelectedGateId(''); }}
        >
          📦 Nhập Hàng
        </button>
        <button
          className={`btn btn--full ${taskType === 'export' ? 'btn--primary' : 'btn--ghost'}`}
          onClick={() => { setTaskType('export'); setShowForm(true); setUserSelectedSlotId(''); setSelectedSku(''); setSelectedGateId(''); }}
        >
          🚚 Xuất Hàng
        </button>
      </div>

      {showForm && (
        <div className="connection-form">
          <div style={{
            fontSize: '13px', fontWeight: '600', marginBottom: '12px',
            color: taskType === 'import' ? 'var(--accent-primary)' : 'var(--accent-secondary)'
          }}>
            {taskType === 'import' ? '📦 Lập phiếu nhập kho' : '🚚 Lập phiếu xuất kho'}
          </div>

          {taskType === 'import' && (
            <div style={{ display: 'flex', gap: '10px', marginBottom: '12px', fontSize: '12px' }}>
               <label>
                  <input type="radio" checked={importType === 'new'} onChange={() => { setImportType('new'); setSelectedSku(''); setUserSelectedSlotId(''); }} /> Hàng Mới
               </label>
               <label>
                  <input type="radio" checked={importType === 'existing'} onChange={() => { setImportType('existing'); setUserSelectedSlotId(''); }} /> Hàng Có Sẵn
               </label>
            </div>
          )}

          <div className="input-group">
            <label className="input-group__label">Chọn Cổng Giao Dịch</label>
            <select
              className="select"
              value={selectedGateId}
              onChange={(e) => setSelectedGateId(e.target.value)}
            >
              <option value="">-- Chọn Cổng --</option>
              {Object.values(GATES)
                .filter(g => taskType === 'import' ? (g.type === 'import' || g.type === 'error') : (g.type === 'export' || g.type === 'error'))
                .map(g => (
                  <option key={g.id} value={g.id}>{g.name}</option>
                ))
              }
            </select>
          </div>

          {((taskType === 'import' && importType === 'existing') || taskType === 'export') && (
            <div className="input-group">
              <label className="input-group__label">Tìm / Chọn Hàng Hóa {loadingInv ? '(Đang tải...)' : ''}</label>
              <input 
                type="text" 
                className="input" 
                placeholder="🔍 Nhập Tên hoặc SKU..." 
                value={searchQuery}
                onChange={e => setSearchQuery(e.target.value)}
                style={{ marginBottom: '8px' }}
              />
              <select
                className="select"
                value={selectedSku}
                onChange={(e) => handleSelectExistingItem(e.target.value)}
                size="4" // shows multiple lines like a listbox
              >
                {filteredInventory.length === 0 && <option disabled>Không tìm thấy hàng phù hợp</option>}
                {filteredInventory.map(s => (
                  <option key={s.id} value={s.sku}>[{s.sku}] {s.name} (SL:{s.quantity})</option>
                ))}
              </select>
            </div>
          )}

          {taskType === 'import' && importType === 'new' && (
            <>
              <div className="input-group">
                <label className="input-group__label">Tên Hàng Hóa *</label>
                <input
                  type="text"
                  className="input"
                  placeholder="VD: Bánh kẹo, Bo mạch..."
                  value={newName}
                  onChange={(e) => setNewName(e.target.value)}
                />
              </div>

              <div 
                style={{ display: 'flex', alignItems: 'center', marginBottom: '8px', cursor: 'pointer', padding: '6px', background: 'var(--bg-dark)', borderRadius: '4px' }} 
                onClick={() => setRequireSku(!requireSku)}
              >
                <input type="checkbox" checked={requireSku} readOnly style={{ marginRight: '8px' }} />
                <span style={{ fontSize: '13px', color: 'var(--text-primary)' }}>Mã SKU đi kèm (Bỏ chọn nếu muốn tạo mã ngẫu nhiên)</span>
              </div>
              
              {requireSku && (
                <div className="input-group">
                  <label className="input-group__label">Mã SKU *</label>
                  <input
                    type="text"
                    className="input"
                    placeholder="VD: SKU-BK-001"
                    value={newSku}
                    onChange={(e) => setNewSku(e.target.value)}
                  />
                </div>
              )}
            </>
          )}

          {/* TRỰC QUAN HÓA KỆ HÀNG */}
          {(taskType === 'export' || (taskType === 'import' && importType === 'new')) ? (
            <VisualShelfSelector 
               inventory={inventory}
               selectedSlotId={userSelectedSlotId}
               onSelectSlot={handleSelectSlotVisual}
               taskType={taskType}
            />
          ) : (
            <div style={{ marginTop: '12px', fontSize: '11px', color: 'var(--text-muted)' }}>
              * Hàng có sẵn sẽ tự động cất vào vị trí {selectedSku && inventory.find(i => i.sku === selectedSku)?.slot_id} cũ của nó.
            </div>
          )}

          <div className="input-group" style={{ marginTop: '12px' }}>
            <label className="input-group__label">Số Lượng {taskType === 'import' ? 'nhập' : 'cần xuất'}</label>
            <input
              type="number"
              className="input"
              min="1"
              value={quantity}
              onChange={(e) => setQuantity(Number(e.target.value))}
            />
          </div>

          <div style={{ display: 'flex', gap: '8px', marginTop: '12px' }}>
            <button
              className="btn btn--success btn--full btn--sm"
              onClick={handleCreateTask}
            >
              ✓ {taskType === 'import' ? 'Giao Robot Đi Cất Hàng' : 'Giao Robot Đi Lấy Hàng'}
            </button>
            <button
              className="btn btn--ghost btn--sm"
              onClick={() => setShowForm(false)}
            >
              Hủy
            </button>
          </div>
        </div>
      )}

      <div className="panel__divider" />

      {/* Nhiệm vụ đang chạy */}
      {activeTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              🔄 {vi.task.status.inProgress} ({activeTasks.length})
            </span>
          </div>
          {activeTasks.map(task => (
            <TaskCard key={task.id} task={task} />
          ))}
        </>
      )}

      {/* Nhiệm vụ chờ */}
      {pendingTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              ⏳ {vi.task.status.pending} ({pendingTasks.length})
            </span>
          </div>
          {pendingTasks.map(task => (
            <TaskCard key={task.id} task={task} />
          ))}
        </>
      )}

      {/* Nhiệm vụ hoàn thành */}
      {completedTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              ✅ {vi.task.status.completed} ({completedTasks.length})
            </span>
          </div>
          {completedTasks.map(task => (
            <TaskCard key={task.id} task={task} />
          ))}
        </>
      )}

      {tasks.length === 0 && !showForm && (
        <div style={{ textAlign: 'center', padding: '24px', color: 'var(--text-muted)', fontSize: '12px' }}>
          Chưa có nhiệm vụ nào.<br />
          Nhấn "Nhập Hàng" hoặc "Xuất Hàng" để bắt đầu.
        </div>
      )}
    </div>
  );
}

/**
 * Task Card
 */
function TaskCard({ task }) {
  const statusColors = {
    pending: 'var(--text-muted)',
    assigned: 'var(--accent-warning)',
    in_progress: 'var(--accent-primary)',
    completed: 'var(--accent-success)',
    failed: 'var(--accent-danger)',
  };

  const statusBorder = task.status === 'completed' ? 'card--success' :
                       task.status === 'failed' ? 'card--danger' :
                       task.status === 'in_progress' ? '' : '';

  return (
    <div className={`card task-card ${statusBorder}`}>
      <div className="task-card__header">
        <span className={`task-card__type task-card__type--${task.type}`}>
          {task.type === 'import' ? '📦 Nhập' : '🚚 Xuất'}
        </span>
        <span className="task-card__id">#{task.id}</span>
      </div>

      <div style={{ fontSize: '12px', color: 'var(--text-secondary)' }}>
        📍 {task.slotId}
      </div>

      <div style={{ fontSize: '11px', color: statusColors[task.status], marginTop: '4px', fontWeight: '500' }}>
        {vi.task.status[task.status === 'in_progress' ? 'inProgress' : task.status] || task.status}
      </div>

      {task.steps.length > 0 && (
        <div className="task-card__steps">
          {task.steps.map((step, idx) => {
            const stepClass = idx < task.currentStepIdx ? 'task-step--completed' :
                              idx === task.currentStepIdx ? 'task-step--active' : 'task-step--pending';
            return (
              <div key={idx} className={`task-step ${stepClass}`}>
                <div className="task-step__icon">
                  {idx < task.currentStepIdx ? '✓' : idx === task.currentStepIdx ? '▶' : (idx + 1)}
                </div>
                <span>{step.description}</span>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}

