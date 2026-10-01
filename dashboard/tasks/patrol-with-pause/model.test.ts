import { describe, expect, it } from 'vitest';

import { makeDefault, MAX_PAUSE_SEC, toRequest, validate } from './model';

const twoStops = {
  stops: [
    { place: 'north', pauseSec: 10 },
    { place: 'east', pauseSec: 0 },
  ],
  rounds: 2,
};

describe('patrol with pause', () => {
  it('needs at least one stop', () => {
    expect(validate(makeDefault())).toMatch(/at least one stop/);
  });

  it('refuses an empty place, a bad pause and bad rounds', () => {
    expect(validate({ stops: [{ place: '', pauseSec: 5 }], rounds: 1 })).toMatch(/Stop 1/);
    expect(validate({ stops: [{ place: 'a', pauseSec: -1 }], rounds: 1 })).toMatch(/pause/);
    expect(validate({ stops: [{ place: 'a', pauseSec: MAX_PAUSE_SEC + 1 }], rounds: 1 })).toMatch(
      /pause/,
    );
    expect(validate({ stops: [{ place: 'a', pauseSec: NaN }], rounds: 1 })).toMatch(/pause/);
    expect(validate({ stops: [{ place: 'a', pauseSec: 5 }], rounds: 0 })).toMatch(/Rounds/);
    expect(validate({ stops: [{ place: 'a', pauseSec: 5 }], rounds: 1.5 })).toMatch(/Rounds/);
    expect(validate(twoStops)).toBeNull();
  });

  it('builds one compose phase per stop visit, rounds unrolled', () => {
    const request = toRequest(twoStops);
    expect(request.category).toBe('compose');
    expect(request.description.category).toBe('patrol_with_pause');
    const visits = request.description.phases.map(
      (p: { activity: { description: { activities: { category: string }[] } } }) =>
        p.activity.description.activities,
    );
    expect(visits).toHaveLength(4);
    // north: drive + wait; east: pause 0, drive only.
    expect(visits[0]).toEqual([
      { category: 'go_to_place', description: 'north' },
      {
        category: 'perform_action',
        description: {
          category: 'wait',
          description: { duration_sec: 10 },
          unix_millis_action_duration_estimate: 10000,
        },
      },
    ]);
    expect(visits[1]).toEqual([{ category: 'go_to_place', description: 'east' }]);
    expect(visits[2]).toEqual(visits[0]);
    expect(visits[3]).toEqual(visits[1]);
  });

  it('labels the task for the task list', () => {
    expect(toRequest(twoStops).labels).toEqual([
      'task_definition_id=patrol_with_pause',
      'destination=east',
    ]);
  });
});
