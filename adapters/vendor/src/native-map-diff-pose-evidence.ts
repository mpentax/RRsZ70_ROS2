type Point = { x: number; y: number };

/** Offline research only. A native diff may contain robot_data, but Hermes v96
 * supplies no pose-specific capture clock/sequence or proven V1 map frame.
 * Never translate this result into PinGoEvidence.pose or a navigation target.
 * See Hermes #12147/#12150/#12157 and #34754/#34755. */
export type NativeDiffPoseAssessment = Readonly<{
  verification: "unknown";
  reason: "invalid-observation" | "uncorrelated" | "no-robot-data" | "invalid-pose" |
    "pose-capture-unproved" | "map-frame-unproved";
  /** Structural observation only; not current localization or a verified frame. */
  candidate: Point | null;
}>;

export interface NativeDiffObservation {
  readonly selectedDeviceId: string;
  readonly sourceDeviceId: string;
  /** Independent transport correlation, not a map_msgId or an app-generated round. */
  readonly requestNonce: string;
  readonly responseNonce: string;
  readonly receivedAtMs: number;
  readonly response: unknown;
}

const unknown = (reason: NativeDiffPoseAssessment["reason"], candidate: Point | null = null): NativeDiffPoseAssessment =>
  ({ verification: "unknown", reason, candidate });
const record = (x: unknown): x is Record<string, unknown> => x !== null && typeof x === "object" && !Array.isArray(x);
const token = (x: unknown): x is string => typeof x === "string" && x.length > 0 && x.length <= 128;

/** Parse only the documented structural robot_data field. Ignore report IDs,
 * arrival time, nonce, map index and `round` as pose timing or frame evidence.
 * Even distinct coordinates across successive reports do not prove when they
 * were captured (a replay or cached robot block can change). */
export function assessNativeMapDiffPose(observation: NativeDiffObservation): NativeDiffPoseAssessment {
  if (!record(observation) || !token(observation.selectedDeviceId) || !token(observation.sourceDeviceId) ||
    !Number.isSafeInteger(observation.receivedAtMs) || observation.receivedAtMs <= 0) return unknown("invalid-observation");
  if (observation.selectedDeviceId !== observation.sourceDeviceId || !token(observation.requestNonce) ||
    observation.requestNonce !== observation.responseNonce) return unknown("uncorrelated");
  const response = observation.response;
  // Hermes #12150 accepts result=0 + matching nonce and reads robot_data;
  // result=1 is a different map-success path, result=2 an error. Do not
  // unwrap arbitrary status/map blobs or DPS 142/143 into pose evidence.
  if (!record(response) || response.result !== 0 || response.nonce !== observation.requestNonce ||
    !record(response.robot_data)) return unknown("no-robot-data");
  const data = response.robot_data;
  // parseDynamic ultimately supplies robot.x/y/angle (#12157), but raw binary
  // decoding and coordinate frame are not established here. This only accepts
  // an explicitly decoded structural candidate; never trusts its time/frame.
  const robot = record(data.robot) ? data.robot : null;
  if (!robot || typeof robot.x !== "number" || !Number.isFinite(robot.x) ||
    typeof robot.y !== "number" || !Number.isFinite(robot.y)) return unknown("invalid-pose");
  const candidate = { x: robot.x, y: robot.y };
  // Neither map_msgId nor a change in x/y/angle is a pose sample sequence.
  // `round` and receivedAtMs are app/transport clocks; no proven robot clock.
  // The V1 mapIndex is not equivalent to the status-derived plugin map ID.
  return unknown("pose-capture-unproved", candidate);
}
