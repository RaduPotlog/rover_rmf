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
//
// Derived from open-rmf/rmf-web packages/rmf-dashboard-framework/examples/demo/main.tsx
// (Apache-2.0) at 85a34c3ec6481796d7c0eba745d9221055121656; modified by Mechatronics Academy
// as described below.

/*
 * Rover A1's Open-RMF dashboard: rmf-web's demo dashboard with our branding, only the task
 * types our fleet accepts, the ROVER TASK dialog (tasks/) and the Rover window (rover/: drive
 * mode, in/out of the fleet). Built by docker/Dockerfile.dashboard.
 */
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';

import React from 'react';
import ReactDOM from 'react-dom/client';
import {
  InitialWindow,
  LocallyPersistentWorkspace,
  MicroAppManifest,
  RmfDashboard,
  Workspace,
} from 'rmf-dashboard-framework/components';
import {
  createMapApp,
  doorsApp,
  liftsApp,
  robotMutexGroupsApp,
  robotsApp,
  tasksApp,
} from 'rmf-dashboard-framework/micro-apps';
import { StubAuthenticator } from 'rmf-dashboard-framework/services';

import { roverControlApp } from './rover/rover-control-app';
import { RoverTaskAppbarItem } from './tasks/rover-task-dialog';
import { roverTheme } from './theme';

// Set at build time (Dockerfile.dashboard build args API_URL / TRAJECTORY_URL).
const apiServerUrl = import.meta.env.VITE_RMF_API_URL || 'http://localhost:8010';
const trajectoryServerUrl = import.meta.env.VITE_RMF_TRAJECTORY_URL || 'http://localhost:8006';

const mapApp = createMapApp({
  attributionPrefix: 'Open-RMF',
  defaultMapLevel: 'L1',
  defaultRobotZoom: 20,
  defaultZoom: 6,
});

const appRegistry: MicroAppManifest[] = [
  roverControlApp,
  mapApp,
  doorsApp,
  liftsApp,
  robotsApp,
  robotMutexGroupsApp,
  tasksApp,
];

const homeWorkspace: InitialWindow[] = [
  { layout: { x: 0, y: 0, w: 9, h: 6 }, microApp: mapApp },
  { layout: { x: 9, y: 0, w: 3, h: 6 }, microApp: roverControlApp },
];

// The site has no doors, lifts or mutex groups: the Rover window takes their place.
const robotsWorkspace: InitialWindow[] = [
  { layout: { x: 0, y: 0, w: 7, h: 4 }, microApp: roverControlApp },
  { layout: { x: 0, y: 4, w: 7, h: 4 }, microApp: robotsApp },
  { layout: { x: 7, y: 0, w: 5, h: 8 }, microApp: mapApp },
];

const tasksWorkspace: InitialWindow[] = [
  { layout: { x: 0, y: 0, w: 7, h: 8 }, microApp: tasksApp },
  { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
];

/** Every tab registers the ROVER TASK button, whichever one the dashboard opens on. */
function withRoverTasks(element: React.ReactNode) {
  return (
    <>
      <RoverTaskAppbarItem />
      {element}
    </>
  );
}

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl={apiServerUrl}
      trajectoryServerUrl={trajectoryServerUrl}
      authenticator={new StubAuthenticator()}
      helpLink="https://osrf.github.io/ros2multirobotbook/rmf-core.html"
      reportIssueLink="https://github.com/open-rmf/rmf-web/issues"
      themes={{ default: roverTheme }}
      resources={{ fleets: {}, logos: { header: '/resources/logo.png' } }}
      tasks={{
        // Delivery and clean are left out: the rover fleet refuses them (fleet config
        // task_capabilities). Rover-specific tasks live in the ROVER TASK dialog.
        allowedTasks: [{ taskDefinitionId: 'patrol' }, { taskDefinitionId: 'custom_compose' }],
        pickupZones: [],
        cartIds: [],
      }}
      tabs={[
        {
          name: 'Map',
          route: '',
          element: withRoverTasks(<Workspace initialWindows={homeWorkspace} />),
        },
        {
          name: 'Robots',
          route: 'robots',
          element: withRoverTasks(<Workspace initialWindows={robotsWorkspace} />),
        },
        {
          name: 'Tasks',
          route: 'tasks',
          element: withRoverTasks(<Workspace initialWindows={tasksWorkspace} />),
        },
        {
          name: 'Custom',
          route: 'custom',
          element: withRoverTasks(
            <LocallyPersistentWorkspace
              defaultWindows={[]}
              allowDesignMode
              appRegistry={appRegistry}
              storageKey="custom-workspace"
            />,
          ),
        },
      ]}
    />
  );
}

const root = ReactDOM.createRoot(document.getElementById('root') as HTMLElement);
root.render(<App />);
