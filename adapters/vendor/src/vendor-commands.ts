const SLOW_METHODS = new Set([
  "app_arm_out_compartment", "app_arm_in_compartment", "app_open_gripper", "app_grip",
  "app_pick_up", "app_put_down", "app_pick_up_enter_exit", "app_arm_move", "app_arm_direction_move",
  "app_stop_grasp", "app_start_program", "app_program_enter_exit",
]);

/** Response deadline used when a caller does not pass one. */
export function defaultRpcTimeoutMs(method: string): number {
  return SLOW_METHODS.has(method) ? 30_000 : 5_000;
}
