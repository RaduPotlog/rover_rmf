// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
