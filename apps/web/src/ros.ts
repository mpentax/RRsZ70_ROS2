import { json } from './api.ts';

export type Task = { id: string; kind: string; status: string; phase: number; feedback: unknown; result: unknown };
export type RosState = { available: boolean; session_enabled?: boolean | null; state: { connected: boolean; battery_known: boolean; battery_fraction: number; evidence: { valid: boolean; stale: boolean } } | null; camera: unknown; arm: unknown; calibration: unknown };
export type Setup = { device: { id: string; name: string; model: string; host: string; localProtocol: string } | null; accountConfigured: boolean; pinConfigured: boolean };
export type TaskRequest = { kind: 'deliver_garment' | 'patrol_collect' | 'pin_and_go' | 'pick' | 'stack' | 'base_pulse' | 'arm_step' | 'hand_step' | 'arm_operation' | 'deploy_arm' | 'align_target' | 'move_arm' | 'grasp_and_lift' | 'release_and_stow'; goal: Record<string, unknown> };
export const terminal = (task: Task) => ['succeeded', 'failed', 'cancelled', 'rejected'].includes(task.status);
export const taskPath = (id: string) => {
  if (!/^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/.test(id)) throw new Error('Invalid task ID.');
  return `/api/ros/tasks/${id}`;
};
export const startTask = (task: TaskRequest) => json<Task>('/api/ros/tasks', { body: task, action: true, timeout: 5000 });
export const listTasks = (signal?: AbortSignal) => json<{tasks:Task[]}>('/api/ros/tasks', {signal});
export const getTask = (id: string) => json<Task>(taskPath(id));
export const cancelTask = (id: string) => json<{id:string;cancel_requested:boolean}>(`${taskPath(id)}/cancel`, {body:{}, action:true, timeout:5000});
export const stopRobot = () => json('/api/ros/stop', {body:{scope:3}, action:true, timeout:18000});
export type ControlSession = {enabled:boolean;streams_released:boolean;cessation_observed:boolean;outcomes:{dispatch:number;response:number;error_code:string}[]};
export const setControlSession = (enabled:boolean) => json<ControlSession>('/api/ros/control/session', {body:{enabled},action:true,timeout:38000});

export function setupError(error: unknown): string {
  // api.ts only exposes fixed local service messages, never upstream exception text.
  if (error instanceof Error) return error.message;
  return 'Setup request failed. Check the selected region, account, device IP and camera pattern in this form.';
}
