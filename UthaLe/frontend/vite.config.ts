import tailwindcss from '@tailwindcss/vite'
import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// The dev server proxies /api to the FastAPI backend, so the browser talks to a
// single origin (MJPEG stream included). Override with VITE_BACKEND_URL.
const backend = process.env.VITE_BACKEND_URL ?? 'http://127.0.0.1:8000'

export default defineConfig({
  plugins: [react(), tailwindcss()],
  server: {
    port: 5173,
    proxy: { '/api': { target: backend, changeOrigin: true } },
  },
  preview: {
    proxy: { '/api': { target: backend, changeOrigin: true } },
  },
})
