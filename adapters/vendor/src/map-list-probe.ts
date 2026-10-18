/** Opt-in, one-shot L01 map-list diagnostic. No map IDs, geometry, or pose leave this process. */
import { RpcError } from "./robot.ts";
import type { RobotClient } from "./robot.ts";
import { selectedLocalProtocol, type SavedDevice } from "./credentials.ts";
import { parseMultiMapsFlags } from "./active-map-evidence.ts";

export const MAP_LIST_METHOD = "get_multi_maps_list";
export type MapListOutcome = {
  supported: boolean;
  method: typeof MAP_LIST_METHOD;
  transport: "local-L01";
  response: "structured-list" | "empty-list" | "request-rejected" | "no-correlated-response" | "unrecognized-shape" | "preflight-failed";
  itemCount?: number;
  // Counts syntactically valid plugin-normalized identifiers, never their values.
  normalizedFlagCount?: number;
  reason?: "live-safety-preflight-unavailable";
};

/** Recognize only documented list container candidates; do not log unknown keys/values. */
export function summarizeMapList(raw: unknown): MapListOutcome {
  const base = { method: MAP_LIST_METHOD, transport: "local-L01" } as const;
  // Hermes v96 #11957 reads result.map_info.map and #11958 normalizes
  // mapFlag. Accept no other guessed wrapper or unbound ACK as a list.
  const flags = parseMultiMapsFlags(raw);
  if (!flags) return { ...base, supported: false, response: "unrecognized-shape" };
  const normalizedFlagCount = flags.length;
  const list = flags;
  return { ...base, supported: true, response: list.length ? "structured-list" : "empty-list",
    itemCount: list.length, normalizedFlagCount };
}

/** Injected transport for offline/fake tests only. This is NOT a live safety preflight.
 * No default RobotClient is supplied: callers must not mistake this helper for
 * permission to query the robot. The CLI deliberately never calls it.
 */
export async function probeMapList(device: SavedDevice, makeClient: (options: {
  host: string; localKey: string; protocol: "L01";
}) => Pick<RobotClient, "connect" | "request" | "close">): Promise<MapListOutcome> {
  const base = { method: MAP_LIST_METHOD, transport: "local-L01" } as const;
  // Map transport must be explicitly selected, not inferred from cloud metadata.
  if (device.localProtocol !== "L01" || device.model !== "roborock.vacuum.a179" || selectedLocalProtocol(device) !== "L01") {
    return { ...base, supported: false, response: "preflight-failed" };
  }
  let client: ReturnType<typeof makeClient> | undefined;
  try {
    client = makeClient({ host: device.host, localKey: device.localKey, protocol: "L01" });
    await client.connect();
    // RobotClient decrypts L01 and resolves only an in-flight matching RPC id;
    // unsolicited frames and other IDs cannot satisfy this request.
    return summarizeMapList(await client.request(MAP_LIST_METHOD, []));
  } catch (error) {
    return { ...base, supported: false, response: error instanceof RpcError ? "request-rejected" : "no-correlated-response" };
  } finally {
    client?.close();
  }
}

// One prior live read returned an unrecognized shape. There is no trusted, fresh
// selected-device/idle/arm/chassis/camera-continuity preflight for this CLI.
// Do not load credentials, connect, or retry on ANY CLI invocation or argument.
if (process.argv[1] && import.meta.url === new URL(`file://${process.argv[1]}`).href) {
  console.log(JSON.stringify({ method: MAP_LIST_METHOD, transport: "local-L01",
    supported: false, response: "preflight-failed",
    reason: "live-safety-preflight-unavailable" } satisfies MapListOutcome));
  process.exitCode = 1;
}
