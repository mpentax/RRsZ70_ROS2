export function record(value: unknown): Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value)
    ? value as Record<string, unknown> : {};
}

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
  const rosOutcome = record(data.outcome);
  if (rosOutcome.dispatch === 2 || data.dispatch === "not_sent") return "rejected";
  if (rosOutcome.response === 2) return "acknowledged";
  if (rosOutcome.response === 3 || rosOutcome.response === 4) return "failed";
  if (rosOutcome.dispatch === 1 && rosOutcome.response === 1) return "accepted";
  if (data.status === "accepted" || data.status === "pending") return "accepted";
  if (data.status === "executing") return "in-progress";
  if (data.status === "succeeded") return "acknowledged";
  if (data.status === "failed" || data.status === "cancelled") return "failed";
  if (data.status === "rejected") return "rejected";
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
