import { json } from './api.ts';

export type Task = { id: string; kind: string; status: string; phase: number; feedback: unknown; result: unknown };
export type RosState = { available: boolean; session_enabled?: boolean | null; state: { connected: boolean; battery_known: boolean; battery_fraction: number; evidence: { valid: boolean; stale: boolean } } | null; camera: unknown; arm: unknown; calibration: unknown };
export type Setup = { device: { id: string; name: string; model: string; host: string; localProtocol: string } | null; accountConfigured: boolean; pinConfigured: boolean };
export type TaskRequest = { kind: string; goal: Record<string, unknown> };
export type GoalCatalog = { goals: { kind: string; name: string; topic: string; action: string; fields: string[]; navigation: boolean; description: string }[]; context_primitives: { name: string; signature: string; summary: string }[] };
export interface RpcInput { method: string; params: unknown; transport?: 'local' | 'cloud'; timeoutMs?: number }
export interface RpcOutcome { outcome: { command_id: unknown; dispatch: number; response: number; error_code: string }; result_json: string }
export const robotRpc = (input: RpcInput) => json<RpcOutcome>('/api/ros/rpc', { body: input, action: true, timeout: (input.timeoutMs ?? 12000) + 18000 });
export const terminal = (task: Task) => ['succeeded', 'failed', 'cancelled', 'rejected'].includes(task.status);
export const taskPath = (id: string) => {
  if (!/^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/.test(id)) throw new Error('Invalid task ID.');
  return `/api/ros/tasks/${id}`;
};
export const listGoals = (signal?: AbortSignal) => json<GoalCatalog>('/api/ros/goals', { signal });
export const validateGoal = (input: { kind: string; goal: Record<string, unknown>; simulate?: boolean; context?: Record<string, unknown> }) => json('/api/ros/goals/validate', { body: input });
export const startTask = (task: TaskRequest) => json<Task>('/api/ros/tasks', { body: task, action: true, timeout: 5000 });
export const listTasks = (signal?: AbortSignal) => json<{tasks:Task[]}>('/api/ros/tasks', {signal});
export const cancelTask = (id: string) => json<{id:string;cancel_requested:boolean}>(`${taskPath(id)}/cancel`, {body:{}, action:true, timeout:5000});
export const stopRobot = () => json('/api/ros/stop', {body:{scope:3}, action:true, timeout:18000});
export type ControlSession = {enabled:boolean;streams_released:boolean;cessation_observed:boolean;outcomes:{dispatch:number;response:number;error_code:string}[]};
export const setControlSession = (enabled:boolean) => json<ControlSession>('/api/ros/control/session', {body:{enabled},action:true,timeout:38000});
