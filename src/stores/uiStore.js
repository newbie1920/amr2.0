import { create } from 'zustand';

const useUIStore = create((set) => ({
  activeTool: 'move', // 'move', 'goal', 'pose', 'measure'
  setActiveTool: (tool) => set({ activeTool: tool }),
}));

export default useUIStore;
