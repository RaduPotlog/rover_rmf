import { patrolWithPause } from './patrol-with-pause';
import type { AnyRoverTaskType } from './types';

/** The task types the ROVER TASK dialog offers, in menu order. */
export const roverTaskTypes: AnyRoverTaskType[] = [patrolWithPause];
