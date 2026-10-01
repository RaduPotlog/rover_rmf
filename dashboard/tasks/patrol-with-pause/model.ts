/*
 * Patrol with pause: visit the stops in order, waiting at each, for a number of rounds.
 *
 * Jazzy compose tasks have no wait event, so the pause is a `perform_action` of category `wait`
 * that the rover's fleet adapter times (domain/actions.py Wait). One phase per stop visit, so
 * the task log shows which stop the rover is at.
 */
import type { RoverTaskRequest } from '../types';

export const TASK_DEFINITION_ID = 'patrol_with_pause';
export const WAIT_ACTION = 'wait';
/** Same limit as the fleet adapter's MAX_WAIT_SEC. */
export const MAX_PAUSE_SEC = 3600;
export const MAX_ROUNDS = 100;
export const DEFAULT_PAUSE_SEC = 30;

export interface PatrolStop {
  place: string;
  /** 0 = drive on without stopping. */
  pauseSec: number;
}

export interface PatrolWithPause {
  stops: PatrolStop[];
  rounds: number;
}

export function makeDefault(): PatrolWithPause {
  return { stops: [], rounds: 1 };
}

export function validate(task: PatrolWithPause): string | null {
  if (task.stops.length === 0) {
    return 'Add at least one stop.';
  }
  for (const [i, stop] of task.stops.entries()) {
    if (!stop.place) {
      return `Stop ${i + 1}: pick a place.`;
    }
    if (!Number.isFinite(stop.pauseSec) || stop.pauseSec < 0 || stop.pauseSec > MAX_PAUSE_SEC) {
      return `Stop ${i + 1}: the pause must be 0 to ${MAX_PAUSE_SEC} s.`;
    }
  }
  if (!Number.isInteger(task.rounds) || task.rounds < 1 || task.rounds > MAX_ROUNDS) {
    return `Rounds must be a whole number from 1 to ${MAX_ROUNDS}.`;
  }
  return null;
}

function visit(stop: PatrolStop) {
  const activities: object[] = [{ category: 'go_to_place', description: stop.place }];
  if (stop.pauseSec > 0) {
    activities.push({
      category: 'perform_action',
      description: {
        category: WAIT_ACTION,
        description: { duration_sec: stop.pauseSec },
        // RMF plans with this; the adapter decides when the wait is over.
        unix_millis_action_duration_estimate: Math.round(stop.pauseSec * 1000),
      },
    });
  }
  return { activity: { category: 'sequence', description: { activities } } };
}

export function toRequest(task: PatrolWithPause): RoverTaskRequest {
  const phases = [];
  for (let round = 0; round < task.rounds; round++) {
    phases.push(...task.stops.map(visit));
  }
  return {
    category: 'compose',
    description: { category: TASK_DEFINITION_ID, phases },
    labels: [
      `task_definition_id=${TASK_DEFINITION_ID}`,
      `destination=${task.stops[task.stops.length - 1].place}`,
    ],
  };
}
