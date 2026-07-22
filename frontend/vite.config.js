import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig(() => {
  const backendUrl = process.env.F1_BACKEND_URL || 'http://localhost:8000'
  const websocketUrl = backendUrl.replace(/^http/, 'ws')

  return {
    plugins: [react()],
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
