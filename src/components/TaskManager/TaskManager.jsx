/**
 * AMR 2.0 — Task Manager Component
 * Quản lý nhiệm vụ nhập/xuất hàng
 */

import { useState, useEffect } from 'react';
import useTaskStore from '../../stores/taskStore.js';
import useRobotStore from '../../stores/robotStore.js';
import { SHELVES, GATES, findSlotById } from '../../core/warehouse.js';
import { navWorkerApi } from '../../core/navWorkerSetup.js';
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
                     bgColor = 'rgba(91, 130, 246, 0.2)'; // blueish selection
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
  const [activeTab, setActiveTab] = useState('active'); // 'active' | 'history'
  const [selectedTaskId, setSelectedTaskId] = useState(null); // Modal
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

  const { createTask, tasks, updateTask, cancelTask } = useTaskStore();
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

    // Auto routing → Tìm xe rảnh rỗi và đủ pin
    // 1. Lọc xe đang rảnh (nav === 'IDLE' hoặc DONE/ERROR) và pin > 20%
    const availableRobots = connectedRobots.filter(r => {
      const navStatus = r.telemetry?.nav || 'IDLE';
      const batt = r.telemetry?.batt ?? 100;
      // Xe chưa nhận nhiệm vụ chạy nào hoặc đã xong
      const isIdle = navStatus === 'IDLE' || navStatus === 'DONE' || navStatus === 'ERROR';
      return isIdle && batt > 20;
    });

    if (availableRobots.length > 0) {
      // 2. Tìm xe gần điểm bắt đầu nhất
      let startPoint = { x: 0, y: 0 };
      const slotInfo = findSlotById(targetSlotId);
      
      if (taskType === 'import') {
         const gate = Object.values(GATES).find(g => g.id === selectedGateId);
         if (gate) { startPoint = { x: gate.x, y: gate.y }; }
      } else {
         if (slotInfo) { startPoint = { x: slotInfo.slot.approach.x, y: slotInfo.slot.approach.y }; }
      }

      availableRobots.sort((a, b) => {
         const distA = Math.hypot((a.telemetry?.x || 0) - startPoint.x, (a.telemetry?.y || 0) - startPoint.y);
         const distB = Math.hypot((b.telemetry?.x || 0) - startPoint.x, (b.telemetry?.y || 0) - startPoint.y);
         return distA - distB;
      });

      const robot = availableRobots[0];

      if (slotInfo) {
        // Bước 1: Tìm đường A* từ xe → ô kệ
        const gridData = useRobotStore.getState().grid.serialize();
        const result = await navWorkerApi.findPath(
          gridData,
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
    } else {
      console.warn("Không có robot nào đang rảnh và đủ pin! Nhiệm vụ đang ở trạng thái chờ.");
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
  // Lắng nghe sự kiện hoàn thành Task từ Store
  // ==========================================
  useEffect(() => {
    const handleInventoryChange = () => {
       loadInventory();
    };
    window.addEventListener('inventory_changed', handleInventoryChange);
    return () => window.removeEventListener('inventory_changed', handleInventoryChange);
  }, []);


  const activeTasksList = tasks.filter(t => !['completed', 'canceled'].includes(t.status));
  const activeTasks = activeTasksList.filter(t => t.status === 'in_progress' || t.status === 'assigned' || t.status === 'paused' || t.status === 'failed');
  const pendingTasks = activeTasksList.filter(t => t.status === 'pending');
  const historyTasks = tasks.filter(t => ['completed', 'canceled'].includes(t.status)).reverse(); // Mới nhất lên đầu
  const selectedTask = tasks.find(t => t.id === selectedTaskId);
  
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
        <button className={`btn btn--sm ${activeTab === 'active' ? 'btn--primary' : 'btn--ghost'}`} onClick={() => setActiveTab('active')} style={{ flex: 1 }}>
          ⚡ Hiện tại ({activeTasksList.length})
        </button>
        <button className={`btn btn--sm ${activeTab === 'history' ? 'btn--primary' : 'btn--ghost'}`} onClick={() => setActiveTab('history')} style={{ flex: 1 }}>
          📜 Lịch sử ({historyTasks.length})
        </button>
      </div>

      {activeTab === 'active' && (
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
      )}

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

      {/* Nhiệm vụ đang chạy hoặc lỗi */}
      {activeTab === 'active' && activeTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              🔄 Đang xử lý / Lỗi ({activeTasks.length})
            </span>
          </div>
          {activeTasks.map(task => (
            <TaskCard key={task.id} task={task} onClick={() => setSelectedTaskId(task.id)} />
          ))}
        </>
      )}

      {/* Nhiệm vụ chờ */}
      {activeTab === 'active' && pendingTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              ⏳ {vi.task.status.pending} ({pendingTasks.length})
            </span>
          </div>
          {pendingTasks.map(task => (
            <TaskCard key={task.id} task={task} onClick={() => setSelectedTaskId(task.id)} />
          ))}
        </>
      )}

      {/* Lịch sử */}
      {activeTab === 'history' && historyTasks.length > 0 && (
        <>
          <div className="panel__header">
            <span className="panel__title" style={{ fontSize: '11px' }}>
              📜 Lịch sử nhiệm vụ
            </span>
          </div>
          {historyTasks.map(task => (
            <TaskCard key={task.id} task={task} onClick={() => setSelectedTaskId(task.id)} />
          ))}
        </>
      )}

      {tasks.length === 0 && !showForm && (
        <div style={{ textAlign: 'center', padding: '24px', color: 'var(--text-muted)', fontSize: '12px' }}>
          Chưa có nhiệm vụ nào.<br />
          Nhấn "Nhập Hàng" hoặc "Xuất Hàng" để bắt đầu.
        </div>
      )}

      {activeTab === 'history' && historyTasks.length === 0 && (
        <div style={{ textAlign: 'center', padding: '24px', color: 'var(--text-muted)', fontSize: '12px' }}>
          Chưa có lịch sử nhiệm vụ.
        </div>
      )}

      {selectedTask && (
        <TaskDetailsModal 
          task={selectedTask} 
          onClose={() => setSelectedTaskId(null)} 
          onPathGenerated={onPathGenerated}
        />
      )}
    </div>
  );
}

/**
 * Task Card
 */
function TaskCard({ task, onClick }) {
  const { navStopRobot } = useRobotStore();
  const robots = useRobotStore((s) => s.robots);
  const robot = task.assignedRobotId ? robots[task.assignedRobotId] : null;

  const statusColors = {
    pending: 'var(--text-muted)',
    assigned: 'var(--accent-warning)',
    in_progress: 'var(--accent-primary)',
    completed: 'var(--accent-success)',
    failed: 'var(--accent-danger)',
  };

  const statusBorder = task.status === 'completed' ? 'card--success' :
                       task.status === 'failed' ? 'card--danger' :
                       task.status === 'paused' ? 'card--warning' :
                       task.status === 'canceled' ? 'card--danger' :
                       task.status === 'in_progress' ? '' : '';

  // Navigator waypoint progress (from robot telemetry)
  const navWp = robot?.telemetry?.navWp ?? 0;
  const navTotal = robot?.telemetry?.navTotal ?? 0;
  const navState = robot?.telemetry?.nav ?? 'IDLE';
  const wpProgress = navTotal > 0 ? Math.round((navWp / navTotal) * 100) : 0;

  return (
    <div className={`card task-card ${statusBorder}`} onClick={onClick} style={{ cursor: 'pointer' }}>
      <div className="task-card__header">
        <span className={`task-card__type task-card__type--${task.type}`}>
          {task.type === 'import' ? '📦 Nhập' : '🚚 Xuất'}
        </span>
        <span className="task-card__id">#{task.id}</span>
      </div>

      <div style={{ fontSize: '12px', color: 'var(--text-secondary)' }}>
        📍 {task.slotId}
      </div>

      <div style={{ display: 'flex', alignItems: 'center', gap: '6px', marginTop: '4px' }}>
        <span style={{ fontSize: '11px', color: statusColors[task.status], fontWeight: '500' }}>
          {vi.task.status[task.status === 'in_progress' ? 'inProgress' : task.status] || task.status}
        </span>
        {task.status === 'in_progress' && navState !== 'IDLE' && (
          <span style={{ fontSize: '10px', color: 'var(--text-muted)', marginLeft: 'auto' }}>
            [{navState}]
          </span>
        )}
      </div>

      {/* Waypoint progress bar — hiển thị khi robot đang tự lái */}
      {task.status === 'in_progress' && navTotal > 0 && (
        <div style={{ marginTop: '6px' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', color: 'var(--text-muted)', marginBottom: '3px' }}>
            <span>WP {navWp}/{navTotal}</span>
            <span>{wpProgress}%</span>
          </div>
          <div style={{ height: '4px', background: 'var(--bg-dark)', borderRadius: '2px', overflow: 'hidden' }}>
            <div style={{
              height: '100%',
              width: `${wpProgress}%`,
              background: navState === 'DONE' ? 'var(--accent-success)' : 'var(--accent-primary)',
              borderRadius: '2px',
              transition: 'width 0.3s ease',
            }} />
          </div>
        </div>
      )}

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

      {/* Task Error Reason */}
      {(task.status === 'failed' || task.status === 'paused' || task.status === 'canceled') && task.error && (
        <div style={{ marginTop: '8px', fontSize: '10px', color: 'var(--accent-danger)', background: 'rgba(239, 68, 68, 0.1)', padding: '6px', borderRadius: '4px' }}>
          ⚠️ {task.error}
        </div>
      )}

      {/* Nav Stop button — chỉ hiện khi xe đang tự lái */}
      {task.status === 'in_progress' && task.assignedRobotId && robot?.status === 'connected' && navState !== 'IDLE' && navState !== 'DONE' && navState !== 'ERROR' && navState !== 'PAUSED' && (
        <button
          className="btn btn--danger btn--sm btn--full"
          style={{ marginTop: '8px', fontSize: '11px' }}
          onClick={(e) => { e.stopPropagation(); navStopRobot(task.assignedRobotId); }}
        >
          ⛔ Dừng Tự Lái
        </button>
      )}
    </div>
  );
}

/**
 * Task Details Modal
 */
function TaskDetailsModal({ task, onClose, onPathGenerated }) {
  const { cancelTask, updateTask } = useTaskStore();
  const { navigateRobot, resumeRobot } = useRobotStore();
  const robots = useRobotStore(s => s.robots);
  const robot = task.assignedRobotId ? robots[task.assignedRobotId] : null;

  const handleResume = async () => {
    if (!robot || robot.status !== 'connected') return alert('Robot không kết nối!');
    
    if (task.status === 'paused') {
      // Nếu chỉ tạm dừng (pause mềm), gửi lệnh tiếp tục
      resumeRobot(robot.id);
      updateTask(task.id, { status: 'in_progress', error: null });
    } else if (task.status === 'failed') {
      // Nếu lỗi timeout (ESP32 đã xóa path), cần vẽ lại đường đi từ vị trí hiện tại
      const slotInfo = findSlotById(task.slotId);
      if (!slotInfo) return alert('Không tìm thấy tọa độ đích!');
      
      const targetX = task.type === 'import' ? slotInfo.slot.approach.x : slotInfo.slot.approach.x; // Simplified
      const targetY = task.type === 'import' ? slotInfo.slot.approach.y : slotInfo.slot.approach.y;

      const gridData = useRobotStore.getState().grid.serialize();
      const result = await navWorkerApi.findPath(gridData, robot.telemetry.x, robot.telemetry.y, targetX, targetY);
      if (result.success) {
        if (onPathGenerated) onPathGenerated(result.path);
        navigateRobot(robot.id, result.path, slotInfo.slot.heading || 90);
        updateTask(task.id, { status: 'in_progress', error: null });
      } else {
        alert('Không tìm được đường đi mới từ vị trí hiện tại!');
      }
    }
    onClose();
  };

  const handleCancel = () => {
    if (window.confirm('Bạn có chắc chắn muốn hủy nhiệm vụ này? Robot sẽ dừng lại.')) {
      if (task.status === 'in_progress' || task.status === 'paused' || task.status === 'failed') {
        cancelTask(task.id, 'Người dùng chủ động hủy');
        if (robot) {
          useRobotStore.getState().navStopRobot(robot.id);
        }
      }
      onClose();
    }
  };

  return (
    <div style={{
      position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
      background: 'rgba(0,0,0,0.6)', zIndex: 1000,
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      padding: '20px'
    }}>
      <div style={{
        background: 'var(--bg-darker)', borderRadius: 'var(--radius-lg)',
        width: '100%', maxWidth: '400px', border: '1px solid var(--border)',
        boxShadow: '0 10px 30px rgba(0,0,0,0.5)', overflow: 'hidden'
      }}>
        <div style={{ padding: '16px', borderBottom: '1px solid var(--border)', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <div style={{ fontWeight: 'bold' }}>Chi tiết nhiệm vụ #{task.id}</div>
          <button onClick={onClose} style={{ background: 'none', border: 'none', color: 'var(--text-muted)', cursor: 'pointer', fontSize: '18px' }}>✕</button>
        </div>

        <div style={{ padding: '16px', fontSize: '13px', display: 'flex', flexDirection: 'column', gap: '12px' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Loại nhiệm vụ:</span>
            <span style={{ fontWeight: '500' }}>{task.type === 'import' ? '📦 Nhập kho' : '🚚 Xuất kho'}</span>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Vị trí / Ô kệ:</span>
            <span>{task.slotId}</span>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Hàng hóa:</span>
            <span>{task.orderInfo?.name} (SKU: {task.orderInfo?.sku})</span>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Số lượng:</span>
            <span>{task.orderInfo?.qty}</span>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Robot đảm nhận:</span>
            <span>{robot ? robot.name : 'Chưa phân công'}</span>
          </div>
          
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ color: 'var(--text-muted)' }}>Tạo lúc:</span>
            <span>{new Date(task.createdAt).toLocaleTimeString()}</span>
          </div>

          {task.completedAt && (
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span style={{ color: 'var(--text-muted)' }}>Kết thúc lúc:</span>
              <span>{new Date(task.completedAt).toLocaleTimeString()}</span>
            </div>
          )}

          {(task.status === 'failed' || task.status === 'paused' || task.status === 'canceled') && task.error && (
            <div style={{ background: 'rgba(239, 68, 68, 0.15)', padding: '10px', borderRadius: '6px', color: 'var(--accent-danger)', marginTop: '8px', border: '1px solid rgba(239, 68, 68, 0.3)' }}>
              <strong>Lý do dừng:</strong> {task.error}
            </div>
          )}
        </div>

        <div style={{ padding: '16px', borderTop: '1px solid var(--border)', display: 'flex', gap: '10px' }}>
          {(task.status === 'failed' || task.status === 'paused') && (
            <button className="btn btn--primary" style={{ flex: 1 }} onClick={handleResume}>
              ▶ Tiếp Tục
            </button>
          )}
          
          {(task.status === 'in_progress' || task.status === 'failed' || task.status === 'paused') && (
            <button className="btn btn--danger" style={{ flex: 1 }} onClick={handleCancel}>
              ⏹ Hủy Nhiệm Vụ
            </button>
          )}
          
          <button className="btn btn--ghost" style={{ flex: 1 }} onClick={onClose}>
            Đóng
          </button>
        </div>
      </div>
    </div>
  );
}

