import AddIcon from '@mui/icons-material/Add';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import DeleteIcon from '@mui/icons-material/Delete';
import {
  Autocomplete,
  Box,
  Button,
  IconButton,
  Stack,
  TextField,
  Tooltip,
  Typography,
} from '@mui/material';
import React from 'react';

import type { RoverTaskFormProps } from '../types';
import { DEFAULT_PAUSE_SEC, MAX_PAUSE_SEC, MAX_ROUNDS, PatrolStop, PatrolWithPause } from './model';

function numberOr(text: string, fallback: number): number {
  const value = Number(text);
  return text.trim() === '' || Number.isNaN(value) ? fallback : value;
}

export function PatrolWithPauseForm({
  value,
  onChange,
  context,
}: RoverTaskFormProps<PatrolWithPause>) {
  const [pauseForAll, setPauseForAll] = React.useState(DEFAULT_PAUSE_SEC);

  const setStops = (stops: PatrolStop[]) => onChange({ ...value, stops });
  const updateStop = (i: number, stop: Partial<PatrolStop>) =>
    setStops(value.stops.map((s, j) => (j === i ? { ...s, ...stop } : s)));
  const move = (i: number, by: number) => {
    const stops = [...value.stops];
    [stops[i], stops[i + by]] = [stops[i + by], stops[i]];
    setStops(stops);
  };

  return (
    <Stack spacing={2}>
      <Typography variant="body2" color="text.secondary">
        The rover drives to each stop in order and waits there before going on. A pause of 0
        drives straight through.
      </Typography>

      {value.stops.map((stop, i) => (
        <Box key={i} display="flex" alignItems="center" gap={1}>
          <Typography sx={{ width: 24 }} color="text.secondary">
            {i + 1}.
          </Typography>
          <Autocomplete
            sx={{ flex: 1 }}
            options={context.places}
            value={stop.place || null}
            onChange={(_ev, place) => updateStop(i, { place: place ?? '' })}
            renderInput={(params) => <TextField {...params} label="Place" required />}
          />
          <TextField
            sx={{ width: 120 }}
            label="Pause (s)"
            type="number"
            inputProps={{ min: 0, max: MAX_PAUSE_SEC, step: 1 }}
            value={stop.pauseSec}
            onChange={(ev) => updateStop(i, { pauseSec: numberOr(ev.target.value, 0) })}
          />
          <Tooltip title="Move up">
            <span>
              <IconButton aria-label="move up" disabled={i === 0} onClick={() => move(i, -1)}>
                <ArrowUpwardIcon fontSize="small" />
              </IconButton>
            </span>
          </Tooltip>
          <Tooltip title="Move down">
            <span>
              <IconButton
                aria-label="move down"
                disabled={i === value.stops.length - 1}
                onClick={() => move(i, 1)}
              >
                <ArrowDownwardIcon fontSize="small" />
              </IconButton>
            </span>
          </Tooltip>
          <Tooltip title="Remove">
            <IconButton
              aria-label="remove stop"
              onClick={() => setStops(value.stops.filter((_s, j) => j !== i))}
            >
              <DeleteIcon fontSize="small" />
            </IconButton>
          </Tooltip>
        </Box>
      ))}

      <Box display="flex" alignItems="center" gap={2} flexWrap="wrap">
        <Button
          startIcon={<AddIcon />}
          onClick={() => setStops([...value.stops, { place: '', pauseSec: pauseForAll }])}
        >
          Add stop
        </Button>
        <Box flex={1} />
        <TextField
          sx={{ width: 140 }}
          size="small"
          label="Pause for all (s)"
          type="number"
          inputProps={{ min: 0, max: MAX_PAUSE_SEC, step: 1 }}
          value={pauseForAll}
          onChange={(ev) => setPauseForAll(numberOr(ev.target.value, 0))}
        />
        <Button
          disabled={value.stops.length === 0}
          onClick={() => setStops(value.stops.map((s) => ({ ...s, pauseSec: pauseForAll })))}
        >
          Apply to all
        </Button>
      </Box>

      <TextField
        sx={{ width: 120 }}
        label="Rounds"
        type="number"
        inputProps={{ min: 1, max: MAX_ROUNDS, step: 1 }}
        value={value.rounds}
        onChange={(ev) => onChange({ ...value, rounds: numberOr(ev.target.value, 1) })}
      />
    </Stack>
  );
}
