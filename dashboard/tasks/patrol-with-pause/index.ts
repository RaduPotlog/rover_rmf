import type { RoverTaskType } from '../types';
import { PatrolWithPauseForm } from './form';
import { makeDefault, PatrolWithPause, TASK_DEFINITION_ID, toRequest, validate } from './model';

export const patrolWithPause: RoverTaskType<PatrolWithPause> = {
  id: TASK_DEFINITION_ID,
  displayName: 'Patrol with pause',
  makeDefault,
  validate,
  toRequest,
  Form: PatrolWithPauseForm,
};
