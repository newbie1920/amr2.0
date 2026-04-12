/**
 * AMR 2.0 — Task Store (Zustand)
 * Quản lý state cho nhiệm vụ nhập/xuất hàng
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';

const useTaskStore = create(
  persist(
    (set, get) => ({
      tasks: [],
      taskIdCounter: 1,

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
