import { localProtocolCodec } from "../../adapters/vendor/src/local.ts";
import { MAX_LIST_FRAME_BYTES } from "./map-list-split-correlation-research.ts";

/** Offline protocol-102 research only. No socket, credential loading, RPC or logging.
 * A valid V1 frame and local-key outer layer bind a JSON RPC ID, not the
 * method, map namespace, active floor or freshness of a returned list.
 */
export type ListFrameAssessment =
  | Readonly<{ kind: "unrelated" | "malformed" | "opaque" }>
  | Readonly<{ kind: "bound"; reply: Readonly<{ requestId: number; result?: unknown; errorCode?: number }> }>;
const OTHER = { kind: "unrelated" } as const;
const MALFORMED = { kind: "malformed" } as const;
const OPAQUE = { kind: "opaque" } as const;
const UTF8 = new TextDecoder("utf-8", { fatal: true });
const record = (value: unknown): value is Record<string, unknown> =>
  value !== null && typeof value === "object" && !Array.isArray(value);
const idValid = (id: number): boolean => Number.isSafeInteger(id) && id >= 0 && id <= 0x7fffffff;

/** Structural scan before JSON.parse. Reject excessive nesting and strings,
 * including escaped quote/backslash sequences; JSON.parse checks syntax later.
 */
function boundedJson(text: string): boolean {
  let depth = 0, nodes = 0, inString = false, escape = false, stringLength = 0;
  for (const char of text) {
    if (inString) {
      if (++stringLength > MAX_LIST_FRAME_BYTES) return false;
      if (escape) escape = false;
      else if (char === "\\") escape = true;
      else if (char === '"') inString = false;
    } else if (char === '"') { inString = true; stringLength = 0; }
    else if (char === "{" || char === "[") { if (++depth > 24 || ++nodes > 4096) return false; }
    else if (char === "}" || char === "]") { if (--depth < 0) return false; }
    else if (char === "," || char === ":") { if (++nodes > 4096) return false; }
  }
  return depth === 0 && !inString;
}

/** Supply only the selected-device MQTT topic's frame, and the exact ID
 * synchronously reserved by an independent L01 request. Topic validation is a
 * caller obligation; this deliberately has no topic, network or CLI surface.
 */
export function inspectCloudMapListFrame(frame: Buffer, localKey: string, expectedRequestId: number): ListFrameAssessment {
  if (!Buffer.isBuffer(frame) || frame.length < 23 || frame.length > MAX_LIST_FRAME_BYTES ||
      frame.toString("latin1", 0, 3) !== "1.0" ||
      frame.readUInt16BE(17) + 23 !== frame.length ||
      localProtocolCodec.crc32(frame.subarray(0, -4)) !== frame.readUInt32BE(frame.length - 4)) return MALFORMED;
  if (frame.readUInt16BE(15) !== 102) return OTHER;
  if (!idValid(expectedRequestId) || Buffer.byteLength(localKey) !== 16) return MALFORMED;
  if (!frame.readUInt16BE(17) || frame.readUInt16BE(17) % 16) return MALFORMED;
  let plain: Buffer;
  try {
    plain = localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
      version: "1.0", localKey, timestamp: frame.readUInt32BE(11),
      sequence: frame.readUInt32BE(3), random: frame.readUInt32BE(7), connectNonce: 0, ackNonce: 0,
    });
  } catch { return OPAQUE; }
  try {
    const text = UTF8.decode(plain);
    if (!boundedJson(text)) return OPAQUE;
    const outer: unknown = JSON.parse(text);
    if (!record(outer) || !record(outer.dps) || typeof outer.dps["102"] !== "string" ||
        Buffer.byteLength(outer.dps["102"]) > MAX_LIST_FRAME_BYTES || !boundedJson(outer.dps["102"])) return OPAQUE;
    const inner: unknown = JSON.parse(outer.dps["102"]);
    if (!record(inner) || !idValid(inner.id as number)) return OPAQUE;
    if (inner.id !== expectedRequestId) return OTHER;
    if (inner.error !== undefined && inner.error !== null) {
      const code = record(inner.error) && typeof inner.error.code === "number" && Number.isSafeInteger(inner.error.code)
        ? inner.error.code : -1;
      return { kind: "bound", reply: { requestId: inner.id, errorCode: code } };
    }
    if (!Object.hasOwn(inner, "result")) return OPAQUE;
    return { kind: "bound", reply: { requestId: inner.id, result: inner.result } };
  } catch { return OPAQUE; }
}
