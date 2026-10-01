/*
 * Client for the fleet adapter's rover control API (rover_rmf_fleet_adapter
 * presentation/control_api.py), plus the pure logic the Rover card shows.
 */

export type DriveModeTarget = 'MANUAL' | 'AUTOMATIC';

export interface RobotSnapshot {
  name: string;
  /** VDA 5050 connection state: ONLINE, OFFLINE, CONNECTIONBROKEN, ... */
  connection: string;
  /** VDA 5050 operatingMode: AUTOMATIC, MANUAL (Manual or Assisted), SERVICE, ... or ''. */
  operating_mode: string;
  /** null until the rover has reported a mode. */
  in_fleet: boolean | null;
  blocked_reason: string | null;
  localized: boolean;
  battery_soc: number | null;
  drive_mode_request: {
    mode: DriveModeTarget;
    done: boolean;
    ok: boolean | null;
    message: string;
  } | null;
}

export interface DriveModeAnswer {
  ok: boolean;
  message: string;
  robot?: RobotSnapshot;
}

export async function fetchRobots(baseUrl: string): Promise<RobotSnapshot[]> {
  const response = await fetch(`${baseUrl}/api/robots`);
  if (!response.ok) {
    throw new Error(`rover control API answered ${response.status}`);
  }
  return ((await response.json()) as { robots: RobotSnapshot[] }).robots;
}

export async function setDriveMode(
  baseUrl: string,
  robot: string,
  mode: DriveModeTarget,
): Promise<DriveModeAnswer> {
  const response = await fetch(`${baseUrl}/api/robots/${encodeURIComponent(robot)}/drive_mode`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode }),
  });
  const body = await response.json();
  if (!response.ok) {
    return { ok: false, message: body.error ?? `rover control API answered ${response.status}` };
  }
  return body as DriveModeAnswer;
}

export type Tone = 'success' | 'warning' | 'error' | 'default';

export interface RoverView {
  online: boolean;
  connection: { label: string; tone: Tone };
  driveMode: { label: string; tone: Tone };
  fleet: { label: string; tone: Tone };
  /** Why the rover would not drive now, if anything. */
  problem: string | null;
  battery: string;
  /** Which buttons make sense: the mode it is not in, and only while connected. */
  canManual: boolean;
  canAutomatic: boolean;
}

const MODE_LABELS: Record<string, string> = {
  AUTOMATIC: 'Automatic',
  SEMIAUTOMATIC: 'Semi-automatic',
  MANUAL: 'Manual',
  SERVICE: 'Service',
  TEACHIN: 'Teach-in',
};

/** What the Rover card shows for one robot. */
export function describeRover(robot: RobotSnapshot): RoverView {
  const online = robot.connection === 'ONLINE';
  const mode = robot.operating_mode;
  const modeLabel = mode ? (MODE_LABELS[mode] ?? mode) : 'unknown';
  const automatic = mode === 'AUTOMATIC' || mode === 'SEMIAUTOMATIC';

  let fleet: RoverView['fleet'];
  if (robot.in_fleet === null) {
    fleet = { label: 'Not known yet', tone: 'default' };
  } else if (robot.in_fleet) {
    fleet = { label: 'In fleet', tone: 'success' };
  } else {
    fleet = { label: `Out of fleet (${modeLabel})`, tone: 'warning' };
  }

  let problem: string | null = null;
  if (!online) {
    problem = 'The rover is not connected to the fleet broker.';
  } else if (!robot.localized) {
    problem = 'The rover is not localized.';
  } else if (automatic && robot.blocked_reason) {
    problem = robot.blocked_reason;
  }

  return {
    online,
    connection: online
      ? { label: 'Online', tone: 'success' }
      : { label: robot.connection === 'UNKNOWN' ? 'No news' : 'Offline', tone: 'error' },
    driveMode: { label: modeLabel, tone: automatic ? 'success' : mode ? 'warning' : 'default' },
    fleet,
    problem,
    battery: robot.battery_soc === null ? 'unknown' : `${Math.round(robot.battery_soc * 100)} %`,
    canManual: online && mode !== 'MANUAL',
    canAutomatic: online && !automatic,
  };
}
