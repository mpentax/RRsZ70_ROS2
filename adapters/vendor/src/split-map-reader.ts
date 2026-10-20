/** Read the V1 LiDAR map via local L01 request and correlated MQTT 301 reply. */
import { createHash, randomUUID } from "node:crypto";
import { connectAsync } from "mqtt";
import { summarizeV1Map } from "./private-map-cache.ts";
import { RobotClient } from "./robot.ts";
import { createBlobSecurity, decodeCloudMapResponse, inspectMapFrame, MAX_CLOUD_MAP_FRAME_BYTES } from "./photo-protocol.ts";
import { cloudControlCredentials, mqttOptions } from "./cloud-control.ts";
import type { SavedDevice } from "./secret-store.ts";
import type { CameraAccount } from "./cloud.ts";
const TIMEOUT_MS = 12_000;
export async function readSplitMap(device: SavedDevice, account: CameraAccount): Promise<{
 raw: Buffer; bytes: number; sha256: string; mapIndex: number; mapSequence: number; receivedAt: number;
 geometry: ReturnType<typeof summarizeV1Map>;
}> {
 if (device.model !== "roborock.vacuum.a179") throw new Error("Unsupported map device.");
 const credentials = cloudControlCredentials(account, device);
  const auth = mqttOptions(credentials);
  const security = createBlobSecurity(credentials.accountKey);
  const client = await connectAsync(credentials.broker, {
    username: auth.username, password: auth.password,
    clientId: `roboclaw-map-split-${randomUUID()}`,
    protocolVersion: 4, clean: true, reconnectPeriod: 0, queueQoSZero: false,
    resubscribe: false, connectTimeout: 4000, keepalive: 20,
  });
  const robot = new RobotClient({ host: device.host, localKey: device.localKey, protocol: "L01" });
  let reason: string = "no-map-frame";
  let requestId: number | null = null;
  let receivedFrames = 0;
  let opaqueFrames = 0;
  let undecodableFrames = 0;
  let map: Buffer | null = null;
  let receivedAt = 0;
  let changed: () => void = () => {};
  let active = true;
  const wake = () => changed();
  client.on("error", wake);
  client.on("close", wake);
  client.on("message", (topic, frame) => {
    if (!active || topic !== auth.incoming || requestId === null) return;
    if (++receivedFrames > 128) { reason = "setup-failed"; client.end(true); return; }
    if (frame.length > MAX_CLOUD_MAP_FRAME_BYTES) { reason = "setup-failed"; client.end(true); return; }
    const classification = inspectMapFrame(frame, credentials.localKey, security, requestId);
    if (classification === "opaque-301") {
      // MQTT has no request binding for opaque 301 bytes. They may be an
      // unrelated camera or device blob; never report them as map evidence.
      if (reason === "no-map-frame") reason = "opaque-301-unbound";
      opaqueFrames++;
    } else if (classification === "correlated-v1") {
      try {
        const decoded = decodeCloudMapResponse(frame, credentials.localKey, security, requestId);
        if (decoded) { map = decoded.data; receivedAt = Date.now(); }
      } catch { reason = "correlated-map-undecodable"; undecodableFrames++; }
    }
    wake();
  });
  try {
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("setup-failed")), 4000);
      client.subscribe(auth.incoming, { qos: 0 }, (error, granted) => {
        clearTimeout(timer);
        if (error || granted?.length !== 1 || granted[0]?.qos === 128) reject(new Error("setup-failed"));
        else resolve();
      });
    });
    if (!client.connected) throw new Error("setup-failed");
    await robot.connect();
    // The read reserves its ID synchronously before returning the promise.
    // Listen during the LAN ACK so an early cloud response is not discarded.
    const read = robot.requestMapWithSecurity(security);
    requestId = read.requestId;
    const deadline = Date.now() + TIMEOUT_MS;
    let response: { id: number; response: unknown };
    try { response = await read.result; }
    catch { throw new Error("local-not-acknowledged"); }
    if (!Array.isArray(response.response) || response.response.length !== 1 || response.response[0] !== "ok") {
      throw new Error("local-not-acknowledged");
    }
    // A local ACK is not a map. Wait only for a correlated 301, once.
    while (!map && client.connected && Date.now() < deadline) {
      await new Promise<void>((resolve) => {
        const timer = setTimeout(resolve, Math.max(1, deadline - Date.now()));
        changed = () => { clearTimeout(timer); resolve(); };
      });
      changed = () => {};
    }
    if (!map) {
      if (undecodableFrames > 0) throw new Error("correlated-map-undecodable");
      if (opaqueFrames > 0) {
        // Timed presence is diagnostic only; no request-id/nonce binding.
        throw new Error(`opaque-301-unbound:${opaqueFrames}`);
      }
      throw new Error(reason);
    }
    const bytes: Buffer = map;
    let geometry: ReturnType<typeof summarizeV1Map>;
    try { geometry = summarizeV1Map(bytes); }
    catch { throw new Error("invalid-map-format"); }
    return {
      bytes: bytes.length, sha256: createHash("sha256").update(bytes).digest("hex"),
      mapIndex: bytes.readUInt32LE(12), mapSequence: bytes.readUInt32LE(16), receivedAt,
      geometry, raw: bytes };
  } finally {
    active = false; robot.close(); client.end(true);
  }
}
