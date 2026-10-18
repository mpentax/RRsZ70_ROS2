import { parseMultiMapsFlags } from "../../adapters/vendor/src/active-map-evidence.ts";

/** OFFLINE research classifier. Inputs are synthetic or independently decoded observations;
 * this module has no socket, credential, robot client, command, or service integration.
 * `frameValid`/`outerDecrypted` are assertions supplied by a future audited V1 decoder,
 * NOT validations performed by this classifier. Never feed raw MQTT bytes directly.
 */
export const SPLIT_LIST_RESEARCH_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
export const MAX_LIST_FRAME_BYTES = 64 * 1024;
export const MAX_LIST_FRAMES = 128;
const MAX_DEPTH = 24;
const MAX_NODES = 4096;
const WINDOW_MS = 12_000;

type Reply = Readonly<{ requestId: number; result?: unknown; errorCode?: unknown }>;
type Lan = Readonly<{ source: "lan"; at: number; correlated: boolean; reply: Reply }>;
type Cloud = Readonly<{ source: "cloud"; at: number; topic: string; protocol: number;
  frameBytes: number; frameValid: boolean; outerDecrypted: boolean; reply?: Reply }>;
export type SplitListObservation = Lan | Cloud;
export type SplitListInput = Readonly<{
  requestId: number; incomingTopic: string; sentAt: number;
  /** No network setup or publishing occurs here; false fails closed. */
  setupReady: boolean;
  observations: readonly SplitListObservation[];
}>;
type Shape = "ack-ok" | "plugin-list-shape" | "rpc-rejected" | "other-structured" | "malformed";
type Outcome = "LAN_list" | "LAN_ACK + cloud_list" | "cloud_list" | "ACK_only_timeout" |
  "cloud_unbound" | "RPC_rejected" | "setup_failed" | "no_correlated_response" | "correlated_malformed";
export type SplitListAssessment = Readonly<{
  status: typeof SPLIT_LIST_RESEARCH_STATUS; outcome: Outcome;
  lanAck: boolean; lanList: boolean; cloudAck: boolean; cloudList: boolean;
  /** Count only; never return mapFlag values, unknown keys, names, hashes, or frames. */
  itemCount: number | null;
  ignoredUnbound: number; ignoredLate: number; ignoredDuplicate: number;
  oversizedFrames: number;
}>;
const validId = (value: number): boolean => Number.isSafeInteger(value) && value >= 0 && value <= 0x7fffffff;
const validTime = (value: number): boolean => Number.isSafeInteger(value) && value >= 0;

/** Limit traversal even for cyclic, deeply nested, or non-JSON synthetic inputs.
 * The future wire decoder must separately enforce JSON byte/depth limits before parsing.
 */
function bounded(value: unknown): boolean {
  const stack: { value: unknown; depth: number }[] = [{ value, depth: 0 }];
  const seen = new Set<object>();
  let nodes = 0;
  while (stack.length) {
    const next = stack.pop()!;
    if (++nodes > MAX_NODES || next.depth > MAX_DEPTH) return false;
    const item = next.value;
    if (typeof item === "string" && item.length > MAX_LIST_FRAME_BYTES) return false;
    if (item !== null && typeof item === "object") {
      if (seen.has(item)) return false;
      seen.add(item);
      if (Object.getPrototypeOf(item) !== Object.prototype && !Array.isArray(item) &&
          Object.getPrototypeOf(item) !== null) return false;
      const keys = Object.keys(item);
      if (keys.length > MAX_NODES) return false;
      for (const key of keys) {
        if (key.length > MAX_LIST_FRAME_BYTES) return false;
        const descriptor = Object.getOwnPropertyDescriptor(item, key);
        if (!descriptor || !("value" in descriptor)) return false;
        stack.push({ value: descriptor.value, depth: next.depth + 1 });
      }
    } else if (typeof item === "number" && !Number.isFinite(item)) return false;
    else if (typeof item === "function" || typeof item === "symbol" || typeof item === "bigint") return false;
  }
  return true;
}
function classify(reply: Reply): { shape: Shape; count: number | null } {
  try { if (!bounded(reply)) return { shape: "malformed", count: null }; }
  catch { return { shape: "malformed", count: null }; }
  if (reply.errorCode !== undefined) return { shape: "rpc-rejected", count: null };
  if (Array.isArray(reply.result) && reply.result.length === 1 && reply.result[0] === "ok")
    return { shape: "ack-ok", count: null };
  const flags = parseMultiMapsFlags(reply.result);
  if (flags) return { shape: "plugin-list-shape", count: flags.length };
  return { shape: reply.result !== null && typeof reply.result === "object" ? "other-structured" : "malformed", count: null };
}

/** Correlates a single reserved ID over two independently validated transports.
 * Observations are processed in supplied arrival order, not sorted by timestamps.
 * A list is diagnostic only: it cannot establish active-map binding or live pose.
 */
export function classifySplitMapList(input: SplitListInput): SplitListAssessment {
  let lanAck = false, lanList = false, cloudAck = false, cloudList = false;
  let itemCount: number | null = null;
  let ignoredUnbound = 0, ignoredLate = 0, ignoredDuplicate = 0, oversizedFrames = 0;
  let rejected = false, malformed = false;
  let bound = false;
  let seenLan = false, seenCloudAck = false, seenCloudList = false;
  const ready = input.setupReady === true && validId(input.requestId) &&
    typeof input.incomingTopic === "string" && input.incomingTopic.length > 0 &&
    input.incomingTopic.length <= 512 && validTime(input.sentAt) &&
    input.sentAt <= Number.MAX_SAFE_INTEGER - WINDOW_MS &&
    Array.isArray(input.observations) && input.observations.length <= MAX_LIST_FRAMES;
  if (ready) {
    const deadline = input.sentAt + WINDOW_MS;
    for (const event of input.observations) {
      if (!validTime(event.at) || event.at < input.sentAt || event.at > deadline) {
        ignoredLate++; continue;
      }
      let source: "lan" | "cloud";
      if (event.source === "lan") {
        if (!event.correlated || !event.reply || event.reply.requestId !== input.requestId) {
          ignoredUnbound++; continue;
        }
        if (seenLan) { ignoredDuplicate++; continue; }
        seenLan = true; source = "lan";
      } else if (event.source === "cloud") {
        if (!Number.isSafeInteger(event.frameBytes) || event.frameBytes < 23 ||
            event.frameBytes > MAX_LIST_FRAME_BYTES) {
          oversizedFrames++; continue;
        }
        if (event.topic !== input.incomingTopic || event.protocol !== 102 ||
            event.frameValid !== true || event.outerDecrypted !== true ||
            !event.reply || event.reply.requestId !== input.requestId) {
          ignoredUnbound++; continue;
        }
        source = "cloud";
      } else { ignoredUnbound++; continue; }
      bound = true;
      const { shape, count } = classify(event.reply);
      if (source === "cloud") {
        if (shape === "ack-ok") {
          if (seenCloudAck || seenCloudList) { ignoredDuplicate++; continue; }
          seenCloudAck = true;
        } else if (shape === "plugin-list-shape") {
          if (seenCloudList) { ignoredDuplicate++; continue; }
          seenCloudList = true;
        }
      }
      if (shape === "ack-ok") source === "lan" ? lanAck = true : cloudAck = true;
      else if (shape === "plugin-list-shape") {
        if (source === "lan") lanList = true;
        else cloudList = true;
        if (itemCount === null) itemCount = count;
      } else if (shape === "rpc-rejected") rejected = true;
      else malformed = true;
    }
  }
  const outcome: Outcome = !ready ? "setup_failed" : rejected ? "RPC_rejected" :
    malformed ? "correlated_malformed" : lanList ? "LAN_list" : cloudList ?
    (lanAck ? "LAN_ACK + cloud_list" : "cloud_list") : lanAck || cloudAck ? "ACK_only_timeout" :
    ignoredUnbound || oversizedFrames ? "cloud_unbound" : bound ? "correlated_malformed" : "no_correlated_response";
  return { status: SPLIT_LIST_RESEARCH_STATUS, outcome, lanAck, lanList, cloudAck, cloudList,
    itemCount, ignoredUnbound, ignoredLate, ignoredDuplicate, oversizedFrames };
}
