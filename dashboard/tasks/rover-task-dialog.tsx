/*
 * The ROVER TASK button and dialog: any task type from registry.ts, dispatched to the fleet or a
 * chosen robot. Uses only rmf-dashboard-framework's public hooks, so it survives rmf-web upgrades
 * that keep them.
 */
import AssignmentIcon from '@mui/icons-material/Assignment';
import {
  Alert,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  MenuItem,
  Stack,
  TextField,
} from '@mui/material';
import type { TaskRequest } from 'api-client';
import React from 'react';
import {
  useAppController,
  useRmfApi,
  useTaskFormData,
  useUserProfile,
} from 'rmf-dashboard-framework/hooks';

import { roverTaskTypes } from './registry';
import type { AnyRoverTaskType } from './types';

// Not a fleet/robot pair (those always contain a '/').
const ANY_ROBOT = 'any';

function errorMessage(e: unknown): string {
  // The api-server answers a refused task with 400 and RMF's reason in `detail`.
  const detail = (e as { response?: { data?: { detail?: unknown } } }).response?.data?.detail;
  if (detail !== undefined) {
    return typeof detail === 'string' ? detail : JSON.stringify(detail);
  }
  return (e as Error).message;
}

export function RoverTaskDialog({ open, onClose }: { open: boolean; onClose: () => void }) {
  const rmfApi = useRmfApi();
  const { showAlert } = useAppController();
  const profile = useUserProfile();
  const { waypointNames, fleets } = useTaskFormData(rmfApi);

  const [taskType, setTaskType] = React.useState<AnyRoverTaskType>(roverTaskTypes[0]);
  const [value, setValue] = React.useState<unknown>(() => roverTaskTypes[0].makeDefault());
  // "fleet/robot", or ANY_ROBOT to let RMF pick.
  const [target, setTarget] = React.useState(ANY_ROBOT);
  const [submitting, setSubmitting] = React.useState(false);

  const robots = Object.entries(fleets).flatMap(([fleet, names]) =>
    names.map((robot) => ({ fleet, robot })),
  );
  const problem = taskType.validate(value);

  const selectType = (id: string) => {
    const next = roverTaskTypes.find((t) => t.id === id)!;
    setTaskType(next);
    setValue(next.makeDefault());
  };

  const submit = async () => {
    const request: TaskRequest = {
      ...taskType.toRequest(value),
      unix_millis_earliest_start_time: 0,
      unix_millis_request_time: Date.now(),
      priority: { type: 'binary', value: 0 },
      requester: profile.user.username,
    };
    setSubmitting(true);
    try {
      if (target === ANY_ROBOT) {
        await rmfApi.tasksApi.postDispatchTaskTasksDispatchTaskPost({
          type: 'dispatch_task_request',
          request,
        });
      } else {
        const { fleet, robot } = robots.find((r) => `${r.fleet}/${r.robot}` === target)!;
        await rmfApi.tasksApi.postRobotTaskTasksRobotTaskPost({
          type: 'robot_task_request',
          fleet,
          robot,
          request,
        });
      }
      showAlert('success', `${taskType.displayName} requested`);
      onClose();
    } catch (e) {
      console.error(e);
      showAlert('error', `${taskType.displayName} refused: ${errorMessage(e)}`, 8000);
    } finally {
      setSubmitting(false);
    }
  };

  return (
    <Dialog open={open} onClose={onClose} maxWidth="md" fullWidth>
      <DialogTitle>Rover task</DialogTitle>
      <DialogContent>
        <Stack spacing={3} sx={{ pt: 1 }}>
          <Stack direction="row" spacing={2}>
            <TextField
              select
              sx={{ flex: 1 }}
              label="Task type"
              value={taskType.id}
              onChange={(ev) => selectType(ev.target.value)}
            >
              {roverTaskTypes.map((t) => (
                <MenuItem key={t.id} value={t.id}>
                  {t.displayName}
                </MenuItem>
              ))}
            </TextField>
            <TextField
              select
              sx={{ flex: 1 }}
              label="Robot"
              value={target}
              onChange={(ev) => setTarget(ev.target.value)}
            >
              <MenuItem value={ANY_ROBOT}>Any robot (RMF assigns)</MenuItem>
              {robots.map(({ fleet, robot }) => (
                <MenuItem key={`${fleet}/${robot}`} value={`${fleet}/${robot}`}>
                  {robot} ({fleet})
                </MenuItem>
              ))}
            </TextField>
          </Stack>
          <taskType.Form value={value} onChange={setValue} context={{ places: waypointNames }} />
          {problem && <Alert severity="info">{problem}</Alert>}
        </Stack>
      </DialogContent>
      <DialogActions>
        <Button onClick={onClose}>Cancel</Button>
        <Button variant="contained" disabled={problem !== null || submitting} onClick={submit}>
          Submit now
        </Button>
      </DialogActions>
    </Dialog>
  );
}

export function RoverTaskButton() {
  const [open, setOpen] = React.useState(false);
  return (
    <>
      <Button
        id="rover-task-button"
        color="secondary"
        variant="contained"
        startIcon={<AssignmentIcon />}
        sx={{ mx: 1, whiteSpace: 'nowrap' }}
        onClick={() => setOpen(true)}
      >
        Rover task
      </Button>
      {/* Mounted only while open: every opening starts from a fresh form. */}
      {open && <RoverTaskDialog open onClose={() => setOpen(false)} />}
    </>
  );
}

/**
 * Puts the ROVER TASK button into the app bar. The framework only takes app bar items through
 * its AppController, which exists inside the dashboard, so each tab renders this once.
 */
export function RoverTaskAppbarItem() {
  const { setExtraAppbarItems } = useAppController();
  React.useEffect(() => {
    setExtraAppbarItems(<RoverTaskButton />);
  }, [setExtraAppbarItems]);
  return null;
}
