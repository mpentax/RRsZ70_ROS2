import { SecretStore, type PersistentCamera } from "./secret-store.ts";
import type { SavedDevice } from "./credentials.ts";

const MAX_RESPONSE_BYTES = 1024 * 1024;

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function migrationOrigin(value: string): URL {
  const origin = new URL(value);
  if (origin.protocol !== "http:" || !["127.0.0.1", "[::1]"].includes(origin.hostname) ||
    !["1984", "57001"].includes(origin.port) || origin.username || origin.password ||
    origin.pathname !== "/" || origin.search || origin.hash) {
    throw new Error("Invalid migration origin.");
  }
  return origin;
}

async function responseJson(response: Response): Promise<unknown> {
  if (!response.ok || !response.body) throw new Error("Cannot read migration source.");
  const reader = response.body.getReader();
  const chunks: Uint8Array[] = [];
  let size = 0;
  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      size += value.byteLength;
      if (size > MAX_RESPONSE_BYTES) throw new Error("Migration source is too large.");
      chunks.push(value);
    }
  } finally {
    try { await reader.cancel(); }
    finally { reader.releaseLock(); }
  }
  return JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(Buffer.concat(chunks)));
}

function cameraFromSource(source: string, device: SavedDevice): PersistentCamera {
  const url = new URL(source);
  if (url.protocol !== "roborock:" || url.username || url.password || url.hash ||
    (url.pathname !== "" && url.pathname !== "/")) {
    throw new Error("Invalid migration source.");
  }
  const fields = ["u", "s", "k", "did", "key", "pin"];
  if (Array.from(url.searchParams.keys()).length !== fields.length ||
    fields.some((field) => url.searchParams.getAll(field).length !== 1)) {
    throw new Error("Invalid migration source.");
  }
  if (url.searchParams.get("did") !== device.duid || url.searchParams.get("key") !== device.localKey) {
    throw new Error("Migration device does not match.");
  }
  return {
    account: {
      user: url.searchParams.get("u")!,
      secret: url.searchParams.get("s")!,
      key: url.searchParams.get("k")!,
      mqttUrl: `ssl://${url.host}`,
    },
    pin: url.searchParams.get("pin")!,
  };
}

/**
 * Explicit one-shot migration only; importing this module performs no I/O.
 * Reads the selected device and the loopback z70 producer in memory. Never logs
 * credentials, writes a source URL, or contacts the cloud. Persistence provides
 * filesystem permissions, not encryption.
 */
export async function migrateState(
  directory = process.env.ROBOCLAW_STATE_DIR ?? ".local",
  go2rtcOrigin = "http://127.0.0.1:1984",
): Promise<{ migrated: true; pinCached: true }> {
  try {
    const origin = migrationOrigin(go2rtcOrigin);
    const store = new SecretStore(directory);
    const device = await store.loadDevice();
    if (!device) throw new Error("Missing selected device.");
    if (await store.loadCamera()) throw new Error("Camera state already exists.");
    const endpoint = new URL("/api/streams?src=z70", origin);
    const response = await fetch(endpoint, {
      redirect: "error",
      signal: AbortSignal.timeout(5000),
      headers: { Accept: "application/json" },
    });
    const payload = await responseJson(response);
    if (!record(payload) || !Array.isArray(payload.producers)) throw new Error("Missing producer.");
    const sources = payload.producers.flatMap((producer: unknown): string[] =>
      record(producer) && typeof producer.url === "string" && producer.url.startsWith("roborock:")
        ? [producer.url] : []);
    if (sources.length !== 1) throw new Error("Ambiguous migration source.");
    const camera = cameraFromSource(sources[0]!, device);
    await store.saveCamera(camera);
    return { migrated: true, pinCached: true };
  } catch {
    throw new Error("Private state migration failed; verify the selected device, loopback driver, and private state files.");
  }
}

// Usage: node src/migrate-state.ts [--go2rtc-origin http://127.0.0.1:57001]
// The state directory comes only from ROBOCLAW_STATE_DIR (default .local).
if (import.meta.main) {
  try {
    const args = process.argv.slice(2);
    if (args.length !== 0 && (args.length !== 2 || args[0] !== "--go2rtc-origin")) {
      throw new Error("Invalid migration arguments.");
    }
    const result = await migrateState(undefined, args[1]);
    process.stdout.write(`${JSON.stringify(result)}\n`);
  } catch {
    process.stderr.write("Private state migration failed.\n");
    process.exitCode = 1;
  }
}
