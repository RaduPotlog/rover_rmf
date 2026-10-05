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

import { describe, expect, it } from 'vitest';

import { describeRover, RobotSnapshot } from './rover-api';

const automatic: RobotSnapshot = {
  name: 'rover_a1',
  connection: 'ONLINE',
  operating_mode: 'AUTOMATIC',
  in_fleet: true,
  blocked_reason: null,
  localized: true,
  battery_soc: 0.784,
  drive_mode_request: null,
};

describe('describeRover', () => {
  it('shows an automatic rover in the fleet', () => {
    const view = describeRover(automatic);
    expect(view.driveMode).toEqual({ label: 'Automatic', tone: 'success' });
    expect(view.fleet).toEqual({ label: 'In fleet', tone: 'success' });
    expect(view.problem).toBeNull();
    expect(view.battery).toBe('78 %');
    expect([view.canManual, view.canAutomatic]).toEqual([true, false]);
  });

  it('shows a manual rover out of the fleet', () => {
    const view = describeRover({ ...automatic, operating_mode: 'MANUAL', in_fleet: false });
    expect(view.driveMode.label).toBe('Manual');
    expect(view.fleet).toEqual({ label: 'Out of fleet (Manual)', tone: 'warning' });
    expect([view.canManual, view.canAutomatic]).toEqual([false, true]);
  });

  it('says why an automatic rover would not drive', () => {
    const view = describeRover({ ...automatic, blocked_reason: 'motionLocked: Motion is locked' });
    expect(view.problem).toBe('motionLocked: Motion is locked');
  });

  it('offers nothing while the rover is offline', () => {
    const view = describeRover({ ...automatic, connection: 'CONNECTIONBROKEN' });
    expect(view.connection).toEqual({ label: 'Offline', tone: 'error' });
    expect(view.problem).toMatch(/not connected/);
    expect([view.canManual, view.canAutomatic]).toEqual([false, false]);
  });

  it('copes with a rover that has not reported yet', () => {
    const view = describeRover({
      ...automatic,
      connection: 'UNKNOWN',
      operating_mode: '',
      in_fleet: null,
      battery_soc: null,
      localized: false,
    });
    expect(view.driveMode.label).toBe('unknown');
    expect(view.fleet.label).toBe('Not known yet');
    expect(view.battery).toBe('unknown');
  });
});
