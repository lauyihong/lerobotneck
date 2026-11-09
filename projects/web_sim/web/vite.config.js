import { defineConfig } from 'vite';
import { resolve } from 'node:path';

// base './' = relative asset paths, so the built folder works from any sub-path (GitHub Pages project site, file server, ...)
// Pages: index.html, play/, neck/ = the LeRobotNeck showcase (one app, three entry files so every URL opens directly);
// sim/ = the full wallerobot simulator.
export default defineConfig({
  base: './',
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },   // keep the emscripten module as-is so it finds mujoco.wasm next to itself
  build: {
    target: 'es2022', chunkSizeWarningLimit: 2000,
    rollupOptions: {
      input: {
        main: resolve(import.meta.dirname, 'index.html'),
        play: resolve(import.meta.dirname, 'play/index.html'),
        neck: resolve(import.meta.dirname, 'neck/index.html'),
        sim: resolve(import.meta.dirname, 'sim/index.html'),
      },
    },
  },
  server: { port: 5180 },
});
