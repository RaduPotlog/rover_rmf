/// <reference types="vitest" />
// Built inside rmf-web as packages/rmf-dashboard-framework/examples/rover (Dockerfile.dashboard).
import react from '@vitejs/plugin-react-swc';
import path from 'path';
import { defineConfig } from 'vitest/config';

export default defineConfig({
  plugins: [react()],
  publicDir: path.resolve(__dirname, 'public'),
  test: {
    // The task models are plain TypeScript; the forms are exercised in the browser.
    environment: 'node',
    include: ['**/*.test.ts'],
  },
});
