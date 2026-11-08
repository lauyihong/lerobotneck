import { defineConfig } from 'vite';

// base './' = relative asset paths, so the built folder works from any sub-path (GitHub Pages project site, file server, ...)
export default defineConfig({
  base: './',
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },   // keep the emscripten module as-is so it finds mujoco.wasm next to itself
  build: { target: 'es2022', chunkSizeWarningLimit: 2000 },
  server: { port: 5180 },
});
