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

/*
 * The Rover card: drive mode (Manual / Automatic) and fleet membership of each rover, from the
 * fleet adapter's rover control API. Manual takes the rover out of the fleet; the fleet adapter
 * does that by itself when the rover's mode changes, whoever changed it.
 */
import {
  Alert,
  Box,
  Button,
  Card,
  CardActions,
  CardContent,
  Chip,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  Stack,
  Typography,
} from '@mui/material';
import React from 'react';
import { useAppController, useRmfApi } from 'rmf-dashboard-framework/hooks';

import { describeRover, DriveModeTarget, fetchRobots, RobotSnapshot, setDriveMode } from './rover-api';

const POLL_MS = 2000;

interface RmfRobotInfo {
  status?: string | null;
  task_id?: string | null;
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <Box display="flex" alignItems="center" gap={1} minHeight={32}>
      <Typography variant="body2" color="text.secondary" sx={{ width: 96 }}>
        {label}
      </Typography>
      {children}
    </Box>
  );
}

function RoverCard({
  robot,
  rmf,
  busy,
  onMode,
}: {
  robot: RobotSnapshot;
  rmf?: RmfRobotInfo;
  busy: boolean;
  onMode: (mode: DriveModeTarget) => void;
}) {
  const view = describeRover(robot);
  const request = robot.drive_mode_request;
  return (
    <Card variant="outlined" sx={{ minWidth: 300, flex: 1 }}>
      <CardContent sx={{ pb: 1 }}>
        <Typography variant="h6" gutterBottom>
          {robot.name}
        </Typography>
        <Row label="Connection">
          <Chip size="small" label={view.connection.label} color={view.connection.tone} />
        </Row>
        <Row label="Drive mode">
          <Chip size="small" label={view.driveMode.label} color={view.driveMode.tone} />
        </Row>
        <Row label="Fleet">
          <Chip size="small" label={view.fleet.label} color={view.fleet.tone} />
        </Row>
        <Row label="RMF">
          <Typography variant="body2">
            {rmf?.status ?? 'unknown'}
            {rmf?.task_id ? ` · task ${rmf.task_id}` : ''}
          </Typography>
        </Row>
        <Row label="Battery">
          <Typography variant="body2">{view.battery}</Typography>
        </Row>
        {view.problem && (
          <Alert severity="warning" sx={{ mt: 1 }}>
            {view.problem}
          </Alert>
        )}
        {request && request.done && request.ok === false && (
          <Alert severity="error" sx={{ mt: 1 }}>
            {request.mode === 'MANUAL' ? 'Manual' : 'Automatic'} refused: {request.message}
          </Alert>
        )}
      </CardContent>
      <CardActions sx={{ px: 2, pb: 2 }}>
        <Button
          variant="contained"
          color="inherit"
          disabled={busy || !view.canManual}
          onClick={() => onMode('MANUAL')}
        >
          Manual
        </Button>
        <Button
          variant="contained"
          disabled={busy || !view.canAutomatic}
          onClick={() => onMode('AUTOMATIC')}
        >
          Automatic
        </Button>
      </CardActions>
    </Card>
  );
}

export interface RoverControlProps {
  /** The fleet adapter's rover control API, e.g. http://10.8.0.1:8030. */
  apiUrl: string;
}

export function RoverControl({ apiUrl }: RoverControlProps) {
  const rmfApi = useRmfApi();
  const { showAlert } = useAppController();
  const [robots, setRobots] = React.useState<RobotSnapshot[] | null>(null);
  const [error, setError] = React.useState<string | null>(null);
  const [busy, setBusy] = React.useState<string | null>(null);
  const [confirm, setConfirm] = React.useState<string | null>(null);
  const [rmfRobots, setRmfRobots] = React.useState<Record<string, RmfRobotInfo>>({});

  const refresh = React.useCallback(async () => {
    try {
      setRobots(await fetchRobots(apiUrl));
      setError(null);
    } catch (e) {
      setError(`The rover control API (${apiUrl}) is not reachable: ${(e as Error).message}`);
    }
  }, [apiUrl]);

  React.useEffect(() => {
    refresh();
    const timer = setInterval(refresh, POLL_MS);
    return () => clearInterval(timer);
  }, [refresh]);

  React.useEffect(() => {
    const sub = rmfApi.fleetsObs.subscribe((fleets) => {
      const info: Record<string, RmfRobotInfo> = {};
      for (const fleet of fleets) {
        for (const [name, state] of Object.entries(fleet.robots ?? {})) {
          info[name] = { status: state.status, task_id: state.task_id };
        }
      }
      setRmfRobots(info);
    });
    return () => sub.unsubscribe();
  }, [rmfApi]);

  const switchMode = async (robot: string, mode: DriveModeTarget) => {
    setBusy(robot);
    try {
      const answer = await setDriveMode(apiUrl, robot, mode);
      if (answer.ok) {
        showAlert('success', `${robot}: ${answer.message}`);
      } else {
        showAlert('error', `${robot}: ${answer.message}`, 8000);
      }
    } catch (e) {
      showAlert('error', `${robot}: ${(e as Error).message}`, 8000);
    } finally {
      setBusy(null);
      refresh();
    }
  };

  return (
    <Box sx={{ p: 2, height: '100%', overflow: 'auto' }}>
      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}
      {robots && robots.length === 0 && <Typography>No rovers in this fleet adapter.</Typography>}
      <Stack direction="row" flexWrap="wrap" gap={2}>
        {(robots ?? []).map((robot) => (
          <RoverCard
            key={robot.name}
            robot={robot}
            rmf={rmfRobots[robot.name]}
            busy={busy === robot.name}
            onMode={(mode) =>
              mode === 'AUTOMATIC' ? setConfirm(robot.name) : switchMode(robot.name, mode)
            }
          />
        ))}
      </Stack>
      <Dialog open={confirm !== null} onClose={() => setConfirm(null)}>
        <DialogTitle>Switch {confirm} to Automatic?</DialogTitle>
        <DialogContent>
          <DialogContentText>
            The rover may start driving on its own: it rejoins the fleet and resumes any RMF task
            it was given. Is the area around it clear?
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setConfirm(null)}>Cancel</Button>
          <Button
            variant="contained"
            onClick={() => {
              const robot = confirm!;
              setConfirm(null);
              switchMode(robot, 'AUTOMATIC');
            }}
          >
            Area is clear, switch to Automatic
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
}

export default RoverControl;
