import { createMicroApp } from 'rmf-dashboard-framework/components';

// Set at build time (Dockerfile.dashboard build arg ROVER_API_URL).
const apiUrl = import.meta.env.VITE_ROVER_API_URL || 'http://localhost:8030';

/** The Rover window: drive mode and fleet membership (rover-control.tsx). */
export const roverControlApp = createMicroApp(
  'rover.control',
  'Rover',
  () => import('./rover-control'),
  () => ({ apiUrl }),
);
