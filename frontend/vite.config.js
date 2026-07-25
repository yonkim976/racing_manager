import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, '.', '')
  const backendUrl = env.F1_BACKEND_URL || 'http://localhost:8000'
  const websocketUrl = backendUrl.replace(/^http/, 'ws')

  return {
    plugins: [react()],
    build: {
      // Three.js is loaded only when the explicit THREE TEST renderer is
      // selected. Keep its stable vendor chunk separately cached and set the
      // warning threshold just above the current minified engine size.
      chunkSizeWarningLimit: 530,
      rollupOptions: {
        output: {
          manualChunks(id) {
            if (id.includes('/node_modules/three/')) return 'three-vendor'
            return undefined
          },
        },
      },
    },
    server: {
      proxy: {
        '/api': {
          target: backendUrl,
          changeOrigin: true,
        },
        '/ws': {
          target: websocketUrl,
          ws: true,
        },
      },
    },
  }
})
