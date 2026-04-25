/**
 * AMR 2.0 — Task Store (Zustand)
 * Quản lý state cho nhiệm vụ nhập/xuất hàng
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import { supabase } from '../utils/supabaseClient.js';

const useTaskStore = create(
  persist(
    (set, get) => ({
      tasks: [],
      taskIdCounter: 1,

      /**
       * Xử lý lỗi/hoàn thành Task và Sync DB
       */
      processTaskCompletion: async (taskId, navState, errorMsg = '') => {
        const t = get().tasks.find(t => t.id === taskId);
        if (!t || t.dbUpdated) return;

        set((state) => ({
          tasks: state.tasks.map(task =>
            task.id === taskId ? { ...task, dbUpdated: true } : task
          ),
        }));

        if (navState === 'ERROR') {
          get().updateTask(taskId, { status: 'failed', error: errorMsg });
          return;
        }

        if (navState === 'DONE') {
          try {
            const info = t.orderInfo;
            // Get current inventory
            const { data: inventory } = await supabase.from('inventory').select('*');
            
            if (t.type === 'import') {
              if (info.importType === 'new') {
                await supabase.from('inventory').insert([{
                  sku: info.sku,
                  name: info.name,
                  quantity: info.qty,
                  slot_id: t.slotId
                }]);
              } else {
                const item = inventory?.find(i => i.sku === info.sku);
                const currentQty = item?.quantity || 0;
                await supabase.from('inventory').update({ quantity: currentQty + info.qty }).eq('sku', info.sku);
              }
            } else {
              // Export
              const invItem = inventory?.find(i => i.sku === info.sku);
              if (invItem) {
                if (info.qty >= invItem.quantity) {
                  await supabase.from('inventory').delete().eq('sku', info.sku);
                } else {
                  await supabase.from('inventory').update({ quantity: invItem.quantity - info.qty }).eq('sku', info.sku);
                }
              }
            }
            get().updateTask(taskId, { status: 'completed', completedAt: Date.now() });
            
            // Note: Bắn event để TaskManager biết load lại inventory nếu cần
            window.dispatchEvent(new Event('inventory_changed'));
          } catch (e) {
            console.error('Lỗi khi update database:', e);
            get().updateTask(taskId, { status: 'failed', error: e.message });
          }
        }
      },

      /**
       * Tạo nhiệm vụ mới
       */
      createTask: (type, slotId, orderInfo = {}) => {
        const id = get().taskIdCounter;
        const task = {
          id,
          type,          // 'import' | 'export'
          slotId,
          orderInfo,
          status: 'pending',
          assignedRobotId: null,
          steps: [],
          currentStepIdx: 0,
          createdAt: Date.now(),
          completedAt: null,
          error: null,
        };

        set((state) => ({
          tasks: [...state.tasks, task],
          taskIdCounter: state.taskIdCounter + 1,
        }));

        return task;
      },

      /**
       * Cập nhật task
       */
      updateTask: (taskId, updates) => {
        set((state) => ({
          tasks: state.tasks.map(t =>
            t.id === taskId ? { ...t, ...updates } : t
          ),
        }));
      },

      /**
       * Xóa task
       */
      removeTask: (taskId) => {
        set((state) => ({
          tasks: state.tasks.filter(t => t.id !== taskId),
        }));
      },

      /**
       * Lấy task theo status
       */
      getTasksByStatus: (status) => get().tasks.filter(t => t.status === status),
      getActiveTasks: () => get().tasks.filter(t => t.status === 'in_progress' || t.status === 'assigned'),
      getPendingTasks: () => get().tasks.filter(t => t.status === 'pending'),
      getCompletedTasks: () => get().tasks.filter(t => t.status === 'completed'),
    }),
    {
      name: 'amr-tasks', // unique name for localStorage key
    }
  )
);

export default useTaskStore;
