import { record } from "./helpers.ts";

export type UiOutcome = "rejected" | "failed" | "accepted" | "in-progress" | "acknowledged" | "unknown";
export const UNKNOWN_ACTION_MESSAGE = "Outcome unknown; the request may have executed. Do not retry automatically. Refresh state and verify physically before deciding whether to send another action.";

/** Consume explicit evidence only; HTTP success and controller state are not robot ACKs. */
export function actionOutcome(value: unknown): UiOutcome {
  const data = record(value);
  const evidence = record(data.actionOutcome);
  // Also support an outcome envelope directly, without importing server-only code.
  const envelope = Object.keys(evidence).length ? evidence : data;
  const kind = envelope.outcome;
  if (["unknown", "rejected", "failed", "accepted", "in-progress", "acknowledged"].includes(String(kind))) {
    return kind as UiOutcome;
  }
  if (data.acknowledged === true) return "acknowledged";
  if (data.accepted === true) return "accepted";
  const state = record(data.state);
  if (data.inProgress === true || state.busy === true ||
    ["preparing", "moving", "braking", "stopping", "running"].includes(String(state.phase))) return "in-progress";
  return "unknown";
}

export function outcomeMessage(outcome: UiOutcome): string {
  switch (outcome) {
    case "rejected": return "Request explicitly rejected. Physical state remains unverified; check the robot before continuing.";
    case "failed": return "Execution failure was reported. Partial motion may have occurred; verify physically before continuing.";
    case "accepted": return "Request accepted, not yet acknowledged by the robot. Physical completion is unverified.";
    case "in-progress": return "Request is in progress. Physical completion is unverified; do not repeat the action.";
    case "acknowledged": return "Command acknowledged; physical completion is unverified. Check robot status and verify physically before continuing.";
    case "unknown": return UNKNOWN_ACTION_MESSAGE;
  }
}

export function actionErrorMessage(error: unknown): string {
  return error instanceof Error ? error.message : UNKNOWN_ACTION_MESSAGE;
}
