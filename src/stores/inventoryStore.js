import { create } from 'zustand';
import { supabase } from '../utils/supabaseClient.js';

const useInventoryStore = create((set, get) => ({
  inventory: [],
  isLoading: false,

  fetchInventory: async () => {
    if (!supabase) return;
    set({ isLoading: true });
    try {
      const { data, error } = await supabase.from('inventory').select('*');
      if (error) {
        console.error('Error fetching inventory:', error);
      } else {
        set({ inventory: data || [] });
      }
    } catch (e) {
      console.error('Error fetching inventory:', e);
    } finally {
      set({ isLoading: false });
    }
  },

  // Setup a subscription to auto-refresh inventory
  setupSubscription: () => {
    if (!supabase) return;
    const channel = supabase.channel('custom-all-channel')
      .on(
        'postgres_changes',
        { event: '*', schema: 'public', table: 'inventory' },
        (payload) => {
          console.log('[Inventory] Update received:', payload);
          get().fetchInventory();
        }
      )
      .subscribe();
      
    // Also listen to local window events triggered by taskStore.js
    const localListener = () => {
      get().fetchInventory();
    };
    window.addEventListener('inventory_changed', localListener);
    
    return () => {
      supabase.removeChannel(channel);
      window.removeEventListener('inventory_changed', localListener);
    };
  }
}));

export default useInventoryStore;
