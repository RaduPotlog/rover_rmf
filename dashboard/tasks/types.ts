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
 * The extension point for rover tasks in the ROVER TASK dialog.
 *
 * A task type is a pure model (default value, validation, the RMF task request) plus a form that
 * edits the value. To add one: a folder next to patrol-with-pause/ exporting a RoverTaskType, and
 * one line in registry.ts. If the task needs the robot to do something other than drive, its
 * request uses a `perform_action` whose category the fleet adapter handles
 * (rover_rmf_fleet_adapter/application/actions.py) and the fleet config lists under `actions`.
 */
import type { TaskRequest } from 'api-client';
import type React from 'react';

/** What a form can offer the user, read from RMF. */
export interface TaskFormContext {
  /** Named nav-graph waypoints of the active site. */
  places: string[];
}

export interface RoverTaskFormProps<D> {
  value: D;
  onChange: (value: D) => void;
  context: TaskFormContext;
}

/** The part of the request a task type owns; the dialog fills in requester, times, priority. */
export type RoverTaskRequest = Pick<TaskRequest, 'category' | 'description' | 'labels'>;

export interface RoverTaskType<D> {
  /** Stable id, sent as the `task_definition_id` booking label. Never rename once used. */
  id: string;
  displayName: string;
  makeDefault: () => D;
  /** A message for the user, or null when the value can be submitted. */
  validate: (value: D) => string | null;
  toRequest: (value: D) => RoverTaskRequest;
  Form: React.ComponentType<RoverTaskFormProps<D>>;
}

// The registry holds task types of different value types; the dialog only passes each type's
// own values back to it.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type AnyRoverTaskType = RoverTaskType<any>;
