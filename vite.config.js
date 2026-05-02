import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const host = process.env.TAURI_DEV_HOST;

// https://vite.dev/config/
export default defineConfig(async () => ({
  plugins: [react()],

  // Vite options tailored for Tauri development and only applied in `tauri dev` or `tauri build`
  //
  // 1. prevent Vite from obscuring rust errors
  clearScreen: false,
  // 2. tauri expects a fixed port, fail if that port is not available
  server: {
    port: 1420,
    strictPort: true,
    host: host || false,
    hmr: host
      ? {
          protocol: "ws",
          host,
          port: 1421,
        }
      : undefined,
    watch: {
      // 3. tell Vite to ignore watching `src-tauri`
      ignored: ["**/src-tauri/**"],
    },
  },

  // Code-splitting: tách bundle 2MB thành chunks nhỏ hơn
  build: {
    rollupOptions: {
      output: {
        manualChunks: {
          // React core + state management
          'vendor-react': ['react', 'react-dom', 'zustand'],
          // 3D visualization (react-three-fiber, three.js)
          'vendor-3d': ['@react-three/fiber', '@react-three/drei', 'three'],
          // Database & messaging
          'vendor-data': ['@supabase/supabase-js', '@msgpack/msgpack'],
          // Simulation engine (heavy computation)
          'sim-engine': [
            './src/core/sim/simEngine.js',
            './src/core/sim/simLidar.js',
            './src/core/sim/simWorld.js',
          ],
        },
      },
    },
  },
}));
