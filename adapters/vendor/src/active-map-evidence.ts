import { parseMapStatusEvidence } from "./map-status-evidence.ts";

/** Offline-only interpretation of Hermes v96 #8421/#11957/#11958/#11948.
 * This module accepts caller-correlated observations; it never performs RPCs.
 * Neither a matched mapFlag nor a V1 header index establishes active-map binding.
 */
export interface CorrelatedMapInputs {
  selectedDeviceId: string;
  status: { deviceId: string; requestId: string; observedAt: number; response: unknown } | null;
  maps: { deviceId: string; requestId: string; observedAt: number; response: unknown } | null;
  /** Integrity-verified separately by the private V1 map decoder/store. No bytes or pose. */
  header: { deviceId: string; mapIndex: number; mapSequence: number; integrityVerified: boolean } | null;
}

export interface ActiveMapEvidence {
  verdict: "UNKNOWN";
  reason: "uncorrelated" | "invalid-status" | "invalid-list" | "no-candidate" | "ambiguous-list" | "namespace-unproven";
  statusCandidate: number | null;
  listedMapFlags: number[];
  pluginListMatch: boolean | null;
  v1MapIndex: number | null;
  numericHeaderMatch: boolean | null;
  activeMapVerified: false;
  currentPoseVerified: false;
}

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}
function uint32(value: unknown): value is number {
  return typeof value === "number" && Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
}
function observationValid(value: CorrelatedMapInputs["status"] | CorrelatedMapInputs["maps"], id: string): boolean {
  return !!value && id.length > 0 && value.deviceId === id &&
    typeof value.requestId === "string" && value.requestId.length > 0 && value.requestId.length <= 128 &&
    Number.isSafeInteger(value.observedAt) && value.observedAt > 0;
}

/** Accepts only a *decoded method result*, not an uncorrelated transport envelope.
 * Hermes #11957 reads map_info.map; #11958 copies each mapFlag to the app's id.
 * Reject malformed/duplicate flags rather than selecting an arbitrary item.
 */
export function parseMultiMapsFlags(response: unknown): number[] | null {
  const result = Array.isArray(response) && response.length === 1 ? response[0] : response;
  if (!record(result) || !record(result.map_info) || !Array.isArray(result.map_info.map) ||
      result.map_info.map.length > 64) return null;
  const flags: number[] = [];
  for (const item of result.map_info.map) {
    if (!record(item) || !uint32(item.mapFlag) || flags.includes(item.mapFlag)) return null;
    flags.push(item.mapFlag);
  }
  return flags;
}

/** No time-window heuristic can certify two separately obtained responses. The
 * caller must have independently bound each reply to its own request/device.
 * Even a same-device pair is not atomic: changes between reads remain possible.
 */
export function assessActiveMapEvidence(input: CorrelatedMapInputs): ActiveMapEvidence {
  let reason: ActiveMapEvidence["reason"] = "uncorrelated";
  let statusCandidate: number | null = null;
  let listedMapFlags: number[] = [];
  let pluginListMatch: boolean | null = null;
  let v1MapIndex: number | null = null;
  let numericHeaderMatch: boolean | null = null;
  const header = input.header;
  if (header && header.integrityVerified === true && header.deviceId === input.selectedDeviceId &&
      uint32(header.mapIndex) && uint32(header.mapSequence)) v1MapIndex = header.mapIndex;
  if (!observationValid(input.status, input.selectedDeviceId) ||
      !observationValid(input.maps, input.selectedDeviceId) ||
      input.status!.requestId === input.maps!.requestId) {
    return { verdict: "UNKNOWN", reason, statusCandidate, listedMapFlags, pluginListMatch,
      v1MapIndex, numericHeaderMatch, activeMapVerified: false, currentPoseVerified: false };
  }
  const status = parseMapStatusEvidence(input.status!.response);
  const flags = parseMultiMapsFlags(input.maps!.response);
  if (!status) reason = "invalid-status";
  else if (!flags) reason = "invalid-list";
  else {
    listedMapFlags = flags;
    statusCandidate = status.candidateUsable ? status.pluginMapIdCandidate : null;
    if (statusCandidate === null) reason = "no-candidate";
    else if (flags.length === 0) reason = "ambiguous-list";
    else {
      pluginListMatch = flags.includes(statusCandidate);
      // Numeric equality across namespaces is diagnostic only, never binding.
      numericHeaderMatch = v1MapIndex === null ? null : statusCandidate === v1MapIndex;
      reason = "namespace-unproven";
    }
  }
  return { verdict: "UNKNOWN", reason, statusCandidate, listedMapFlags, pluginListMatch,
    v1MapIndex, numericHeaderMatch, activeMapVerified: false, currentPoseVerified: false };
}
