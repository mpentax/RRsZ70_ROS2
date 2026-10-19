import { record } from "./helpers.ts";

export type UiOutcome = "rejected" | "failed" | "accepted" | "in-progress" | "acknowledged" | "unknown";
export const UNKNOWN_ACTION_MESSAGE = "Outcome unknown.";

export function actionOutcome(value: unknown): UiOutcome {
  const data = record(value);
  const evidence = record(data.actionOutcome);
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
    case "rejected": return "Request rejected.";
    case "failed": return "Execution failed.";
    case "accepted": return "Request accepted.";
    case "in-progress": return "Request in progress.";
    case "acknowledged": return "Command acknowledged.";
    case "unknown": return UNKNOWN_ACTION_MESSAGE;
  }
}

export function actionErrorMessage(error: unknown): string {
  return error instanceof Error ? error.message : UNKNOWN_ACTION_MESSAGE;
}
