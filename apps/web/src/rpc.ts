import { json } from "./api.ts";
export interface RpcInput { method: string; params: unknown; transport?: "local" | "cloud"; timeoutMs?: number }
export interface RpcOutcome { outcome: { command_id: unknown; dispatch: number; response: number; physical_completion: number }; result_json: string }
export function robotRpc(input: RpcInput): Promise<RpcOutcome> {
  return json('/api/ros/rpc', { body: input, action: true, timeout: (input.timeoutMs ?? 12000) + 18000 });
}
