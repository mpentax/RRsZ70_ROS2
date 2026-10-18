/** Offline split-list orchestration research. Injection is mandatory: no real adapters,
 * credential loading, CLI entry point, publish-capable cloud command or service route.
 * A future live path requires separate authorization and a real synchronous local ID API.
 */
import { parseMultiMapsFlags } from "../../adapters/vendor/src/active-map-evidence.ts";

const METHOD = "get_multi_maps_list" as const;
const MAX_FRAMES = 128;
const MAX_FRAME_BYTES = 65558;
const MAX_JSON_DEPTH = 24;
const MAX_JSON_NODES = 4096;
const WINDOW_MS = 12_000;
const SETUP_MS = 4_000;

type Reply = { id: number; result?: unknown; rejected?: boolean };
export type CloudInspection = { kind: "unrelated" | "invalid" } | { kind: "bound"; reply: Reply };
export interface ResearchCloud {
  readonly connected: boolean;
  subscribe(topic: string): Promise<boolean>; // resolves true only for selected-topic SUBACK
  listen(onFrame: (topic: string, frame: Uint8Array) => void, onDisconnect: () => void): () => void;
  close(): void;
}
export interface ResearchLocal {
  connect(): Promise<void>;
  /** Must synchronously reserve and write exactly one L01 RPC, then return the
   * same reserved ID and its independently correlated result. Never reconnect/replay. */
  sendOnce(method: typeof METHOD, params: readonly []): { requestId: number; result: Promise<unknown> };
  close(): void;
}
export interface ResearchInputs {
  topic: string;
  cloud: ResearchCloud;
  local: ResearchLocal;
  /** Fake-only fixture classifier: tests may inject an independent V1/102 validator.
   * It must check CRC, decrypted payload and inner ID before returning `bound`. */
  inspect(frame: Uint8Array): CloudInspection;
  windowMs?: number;
  setupMs?: number;
}
export type ResearchOutcome = "LAN_ACK + cloud_list" | "cloud_list_without_LAN_ACK" | "LAN_list" | "ACK_only_timeout" |
  "cloud_unbound" | "RPC_rejected" | "setup_failed" | "no_correlated_response" |
  "correlated_malformed";
export interface ResearchSummary {
  outcome: ResearchOutcome;
  lanAck: boolean;
  lanList: boolean;
  cloudAck: boolean;
  cloudList: boolean;
  itemCount?: number;
}
function listCount(raw: unknown): number | undefined {
  // Only the count leaves this module. Never include flags, names, keys or hashes.
  return parseMultiMapsFlags(raw)?.length;
}
function category(reply: unknown): "ack" | "list" | "rejected" | "malformed" {
  if (Array.isArray(reply) && reply.length === 1 && reply[0] === "ok") return "ack";
  if (typeof reply === "object" && reply !== null && !Array.isArray(reply) &&
      (reply as { rejected?: unknown }).rejected === true) return "rejected";
  if (listCount(reply) !== undefined) return "list";
  return "malformed";
}
function finiteInteger(id: unknown): id is number {
  return typeof id === "number" && Number.isSafeInteger(id) && id > 0 && id <= 0x7fffffff;
}
/** Treat injected values as untrusted even in the fake harness: no getters,
 * cycles, shared subtrees, non-JSON primitives, or unbounded total work. */
function boundedShape(value: unknown): boolean {
  const stack: { value: unknown; depth: number }[] = [{ value, depth: 0 }];
  const visited = new WeakSet<object>();
  let nodes = 0, chars = 0;
  try {
    while (stack.length) {
      const next = stack.pop()!;
      if (++nodes > MAX_JSON_NODES || next.depth > MAX_JSON_DEPTH) return false;
      const item = next.value;
      if (item === null || typeof item === "boolean") continue;
      if (typeof item === "string") { if ((chars += item.length) > MAX_FRAME_BYTES) return false; continue; }
      if (typeof item === "number") { if (!Number.isFinite(item)) return false; continue; }
      if (typeof item !== "object" || visited.has(item)) return false;
      visited.add(item);
      if (!Array.isArray(item) && Object.getPrototypeOf(item) !== Object.prototype &&
          Object.getPrototypeOf(item) !== null) return false;
      const keys = Object.keys(item);
      if (keys.length > MAX_JSON_NODES || (Array.isArray(item) && item.length > MAX_JSON_NODES)) return false;
      for (const key of keys) {
        if (key.length > 256 || (chars += key.length) > MAX_FRAME_BYTES) return false;
        const descriptor = Object.getOwnPropertyDescriptor(item, key);
        if (!descriptor || !("value" in descriptor)) return false;
        stack.push({ value: descriptor.value, depth: next.depth + 1 });
      }
    }
    return true;
  } catch { return false; }
}

/** A deliberately non-exported live bridge: this function accepts only supplied
 * transports. Its timer covers setup as well as the post-send observation window.
 * On ambiguity it closes both peers and NEVER sends again. */
export async function researchSplitMapList(input: ResearchInputs): Promise<ResearchSummary> {
  let lanAck = false, lanList = false, cloudAck = false, cloudList = false;
  let itemCount: number | undefined;
  let unbound = false, malformed = false, rejected = false, sent = false;
  let requestId: number | undefined, end = 0;
  let frames = 0, closed = false, disconnected = false;
  let wake = () => {};
  let unsubscribe = () => {};
  const cloud = input.cloud, local = input.local;
  const finish = (outcome: ResearchOutcome): ResearchSummary => ({ outcome, lanAck, lanList, cloudAck, cloudList,
    ...(itemCount === undefined ? {} : { itemCount }) });
  const stop = () => {
    if (closed) return;
    closed = true; wake();
    // A failing injected cleanup hook must not leave the other peer open or
    // replace a sanitized outcome with an exception from a fake adapter.
    for (const close of [unsubscribe, () => local.close(), () => cloud.close()]) {
      try { close(); } catch { /* continue releasing the remaining peer */ }
    }
  };
  const wait = (ms: number) => new Promise<void>((resolve) => {
    const timer = setTimeout(resolve, ms);
    wake = () => { clearTimeout(timer); resolve(); };
  });
  const setupMs = Math.min(SETUP_MS, Math.max(1, input.setupMs ?? SETUP_MS));
  const windowMs = Math.min(WINDOW_MS, Math.max(1, input.windowMs ?? WINDOW_MS));
  const setupEnd = Date.now() + setupMs;
  const until = <T>(promise: Promise<T>, deadline: number): Promise<T> => new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error("deadline")), Math.max(0, deadline - Date.now()));
    promise.then((value) => { clearTimeout(timer); resolve(value); }, (error: unknown) => { clearTimeout(timer); reject(error); });
  });
  try {
    if (!input.topic || input.topic.length > 256 || !cloud.connected) return finish("setup_failed");
    const onFrame = (topic: string, frame: Uint8Array) => {
      if (closed || disconnected || topic !== input.topic) return;
      // A callback before sendOnce returns cannot be proven to follow the write.
      // In particular, never replay a buffered pre-send frame after learning the ID.
      if (!sent || requestId === undefined || Date.now() >= end) return;
      if (++frames > MAX_FRAMES || frame.byteLength > MAX_FRAME_BYTES) { malformed = true; wake(); return; }
      let inspected: CloudInspection;
      try { inspected = input.inspect(frame); } catch { unbound = true; wake(); return; }
      if (inspected.kind !== "bound") { if (inspected.kind === "invalid") unbound = true; wake(); return; }
      const { reply } = inspected;
      if (!boundedShape(reply) || !reply || typeof reply !== "object" || !finiteInteger(reply.id)) {
        malformed = true; wake(); return;
      }
      if (reply.id !== requestId) { unbound = true; wake(); return; }
      const kind = reply.rejected === true ? "rejected" : category(reply.result);
      if (kind === "ack") cloudAck = true;
      else if (kind === "list") { cloudList = true; itemCount = listCount(reply.result); }
      else if (kind === "rejected") rejected = true;
      else malformed = true;
      wake();
    };
    unsubscribe = cloud.listen(onFrame, () => { disconnected = true; wake(); });
    if (!await until(cloud.subscribe(input.topic), setupEnd) || !cloud.connected || disconnected) return finish("setup_failed");
    await until(local.connect(), setupEnd);
    if (closed || !cloud.connected || disconnected || Date.now() >= setupEnd) return finish("setup_failed");
    // The adapter contract reserves the ID synchronously, before any queued
    // response callback can run; listening and SUBACK precede this single send.
    const read = local.sendOnce(METHOD, []);
    sent = true;
    if (!finiteInteger(read.requestId)) { malformed = true; return finish("correlated_malformed"); }
    requestId = read.requestId;
    end = Date.now() + windowMs;
    // Do not await LAN indefinitely: a cloud list may arrive before its ACK.
    void read.result.then((result) => {
      if (closed || disconnected || Date.now() >= end) return;
      const kind = boundedShape(result) ? category(result) : "malformed";
      if (kind === "ack") lanAck = true;
      else if (kind === "list") { lanList = true; itemCount = listCount(result); }
      else if (kind === "rejected") rejected = true;
      else malformed = true;
      wake();
    }, () => { if (!closed && !disconnected && Date.now() < end) { rejected = true; wake(); } });
    while (!closed && !disconnected && !rejected && !malformed && !lanList && !(cloudList && lanAck) && Date.now() < end) {
      await wait(Math.max(1, end - Date.now()));
    }
    if (disconnected || !cloud.connected) return { outcome: "no_correlated_response", lanAck: false,
      lanList: false, cloudAck: false, cloudList: false };
    if (rejected) return finish("RPC_rejected");
    if (malformed) return finish("correlated_malformed");
    if (lanList) return finish("LAN_list");
    if (cloudList) return finish(lanAck ? "LAN_ACK + cloud_list" : "cloud_list_without_LAN_ACK");
    if (unbound) return finish("cloud_unbound");
    if (lanAck || cloudAck) return finish("ACK_only_timeout");
    return finish("no_correlated_response");
  } catch {
    return finish(sent ? "no_correlated_response" : "setup_failed");
  } finally { stop(); }
}
