import type { CameraAccount, DeviceCredentials } from "./cloud.ts";
import type { ActiveCamera } from "./camera-selection.ts";
import { localServiceOrigin } from "./local-origin.ts";

export const GO2RTC_ORIGIN = localServiceOrigin(process.env.GO2RTC_ORIGIN ?? "http://127.0.0.1:57001", "go2rtc");
export const GO2RTC_STREAM = "z70";
export const GO2RTC_CAMERA_STREAMS: Record<ActiveCamera, string> = {
  front: `${GO2RTC_STREAM}_front_jpeg`,
  gripper: `${GO2RTC_STREAM}_gripper_jpeg`,
};

class DriverError extends Error {
  readonly upstreamStatus?: number;
  constructor(message: string, upstreamStatus?: number) { super(message); this.upstreamStatus = upstreamStatus; }
}

/** Fixed diagnostics only: never forward credential-bearing upstream text or URLs. */
export function publicCameraFailure(error: unknown): Record<string, unknown> {
  if (!(error instanceof DriverError)) return {};
  const reasons: Record<string, string> = {
    "go2rtc did not return a camera image.": "no_camera_image",
    "go2rtc returned no complete JPEG image.": "invalid_jpeg",
    "go2rtc camera image exceeds the size limit.": "image_too_large",
    "go2rtc image request failed or timed out.": "request_failed",
  };
  const cameraFailure = reasons[error.message];
  if (!cameraFailure) return {};
  return { cameraFailure, ...(Number.isInteger(error.upstreamStatus) && error.upstreamStatus! >= 100 && error.upstreamStatus! <= 599
    ? { upstreamStatus: error.upstreamStatus } : {}) };
}

async function responseText(response: Response, limit = 128 * 1024): Promise<string> {
  if (!response.body) return "";
  const reader = response.body.getReader();
  const chunks: Uint8Array[] = [];
  let size = 0;
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      size += value.byteLength;
      if (size > limit) {
        await reader.cancel();
        throw new DriverError("go2rtc returned an oversized response.");
      }
      chunks.push(value);
    }
  } finally { reader.releaseLock(); }
  return Buffer.concat(chunks).toString("utf8");
}

export function cameraSource(account: CameraAccount, device: DeviceCredentials, pin: string): string {
  if (!/^[1-9]{4,12}$/.test(pin)) throw new Error("Enter the existing pattern as grid digits 1-9.");
  let broker: URL;
  try { broker = new URL(account.mqttUrl); }
  catch { throw new Error("Invalid Roborock camera broker."); }
  if (broker.protocol !== "ssl:" || !broker.hostname.endsWith(".roborock.com") ||
    !broker.port || broker.username || broker.password || broker.search || broker.hash ||
    (broker.pathname !== "" && broker.pathname !== "/")) {
    throw new Error("Untrusted Roborock camera broker.");
  }
  if (!account.user || !account.secret || !account.key || !device.duid || device.localKey.length !== 16) {
    throw new Error("Incomplete camera credentials.");
  }
  const source = new URL(`roborock://${broker.host}`);
  source.search = new URLSearchParams({
    u: account.user, s: account.secret, k: account.key,
    did: device.duid, key: device.localKey, pin,
  }).toString();
  return source.href;
}

export class Go2Rtc {
  readonly #origin: string;
  #warm?: { controller: AbortController; ready: Promise<void> };
  #idleTimer?: ReturnType<typeof setTimeout>;
  #captures = 0;
  #session = new AbortController();

  #activeSignal(): AbortSignal {
    this.#session.signal.throwIfAborted();
    return this.#session.signal;
  }

  async setEnabled(enabled: boolean): Promise<void> {
    if (enabled) {
      if (this.#session.signal.aborted) this.#session = new AbortController();
      return;
    }
    // Abort both the owned warm consumer and outstanding configuration/snapshots.
    // close() alone is intentionally reusable on a lens switch; session stop isn't.
    this.#session.abort();
    this.close();
    await this.unload();
  }

  // Pre-warm both resolution-specific MJPEG receivers (front 1600x1200 and
  // gripper 1600x896) against the shared z70 WebRTC producer so switching
  // cameras never requires restarting FFmpeg or go2rtc.
  async #keepWarm(): Promise<void> {
    clearTimeout(this.#idleTimer);
    if (!this.#warm) {
      const controller = new AbortController();
      const hold = { controller, ready: Promise.resolve() };
      this.#warm = hold;
      const streams = [GO2RTC_CAMERA_STREAMS.front, GO2RTC_CAMERA_STREAMS.gripper];
      let remaining = streams.length;
      hold.ready = (async () => {
        const startup = setTimeout(() => controller.abort(), 30_000);
        startup.unref();
        try {
          for (const stream of streams) {
            const response = await fetch(`${this.#origin}/api/stream.mjpeg?src=${stream}`, {
              redirect: "error", signal: controller.signal,
            });
            if (!response.ok || !response.body || !response.headers.get("content-type")?.startsWith("multipart/x-mixed-replace")) {
              await response.body?.cancel();
              throw new DriverError("go2rtc did not return a camera image.", response.status);
            }
            const reader = response.body.getReader();
            // Drain without buffering; snapshots read the next decoded JPEG from the active camera stream.
            void (async () => {
              try { while (!(await reader.read()).done) { /* discard media */ } }
              catch { /* retry only on the next frame request, never a login loop */ }
              finally {
                reader.releaseLock();
                if (--remaining === 0 && this.#warm === hold) this.#warm = undefined;
              }
            })();
          }
          clearTimeout(startup);
        } catch (error) {
          controller.abort();
          if (this.#warm === hold) this.#warm = undefined;
          // No implicit second preview attempt via frame.jpeg after failed startup.
          throw error instanceof DriverError ? error : new DriverError("go2rtc image request failed or timed out.");
        } finally { clearTimeout(startup); }
      })();
    }
    await this.#warm?.ready;
  }

  close(): void {
    clearTimeout(this.#idleTimer);
    this.#warm?.controller.abort();
    this.#warm = undefined;
  }

  constructor(origin = GO2RTC_ORIGIN) {
    this.#origin = localServiceOrigin(origin, "go2rtc");
  }

  async acquisitionDiagnostics(): Promise<Record<string, unknown>> {
    // This read does not start preview. Retained evidence survives producer cleanup.
    const outcome = ["unknown", "ok", "error", "eof", "closed", "timeout", "rejected", "rpc_error"];
    const candidate = ["unknown", "host", "srflx", "prflx", "relay"];
    const enums: Record<string, string[]> = {
      stage: ["none", "unknown", "parse_url", "tcp", "tls", "mqtt", "subscribe", "pin", "status", "stop_preview", "start_preview", "turn", "create_api", "create_peer", "create_offer", "send_offer", "get_answer", "set_answer", "wait_connected", "connected"],
      failure: ["none", "invalid_url", "error", "eof", "closed", "timeout", "mqtt_rejected", "pin_rejected", "rpc_error", "camera_not_ready", "answer_timeout", "connect_failed"],
      vendorReason: ["none", "unknown", "permission_denied", "busy", "invalid_params", "unsupported", "not_allowed", "camera_disabled"],
      mqtt: outcome, pin: [...outcome, "skipped"], turn: outcome,
      pcState: ["unknown", "new", "connecting", "connected", "disconnected", "failed", "closed"],
      iceState: ["unknown", "new", "checking", "connected", "completed", "disconnected", "failed", "closed"],
      dtlsState: ["unknown", "new", "connecting", "connected", "closed", "failed"],
      localCandidateType: candidate, remoteCandidateType: candidate, candidateProtocol: ["unknown", "udp", "tcp"],
      readerExit: ["none", "unknown", "eof", "timeout", "closed", "read_error", "rtp_error", "unmapped_track", "track_error"],
    };
    try {
      const response = await fetch(`${this.#origin}/api/roborock/diagnostic`, {
        redirect: "error", signal: AbortSignal.timeout(3000),
      });
      if (!response.ok) { await response.body?.cancel(); return { available: false }; }
      const value: unknown = JSON.parse(await responseText(response));
      if (!value || typeof value !== "object" || Array.isArray(value)) return { available: false };
      const row = value as Record<string, unknown>;
      if (typeof row.available !== "boolean") return { available: false };
      if (typeof row.vendorCode !== "number" || !Number.isSafeInteger(row.vendorCode)) return { available: false };
      const result: Record<string, unknown> = { available: row.available, vendorCode: row.vendorCode };
      for (const key of ["attempt", "elapsedMs", "videoTracks", "videoPackets", "videoBytes"]) {
        if (typeof row[key] !== "number" || !Number.isSafeInteger(row[key]) || (row[key] as number) < 0) return { available: false };
        result[key] = row[key];
      }
      for (const [key, allowed] of Object.entries(enums)) {
        if (typeof row[key] !== "string" || !allowed.includes(row[key] as string)) return { available: false };
        result[key] = row[key];
      }
      return result;
    } catch { return { available: false }; }
  }

  async streamDiagnostics(): Promise<Record<string, { available: boolean; producers?: Record<string, unknown>[]; consumers?: Record<string, unknown>[] }>> {
    const result: Record<string, { available: boolean; producers?: Record<string, unknown>[]; consumers?: Record<string, unknown>[] }> = {};
    // Closed numeric fields only: stream JSON contains credential-bearing URLs.
    const counters = (value: unknown, children = true): Record<string, unknown> => {
      const out: Record<string, unknown> = {};
      if (!value || typeof value !== "object" || Array.isArray(value)) return out;
      const row = value as Record<string, unknown>;
      for (const key of ["bytes_recv", "bytes_send", "bytes", "packets"]) {
        if (typeof row[key] === "number" && Number.isSafeInteger(row[key]) && (row[key] as number) >= 0) out[key] = row[key];
      }
      if (children) for (const key of ["receivers", "senders"]) {
        if (Array.isArray(row[key])) out[key] = row[key].slice(0, 16).map(v => counters(v, false));
      }
      return out;
    };
    for (const name of [GO2RTC_STREAM, GO2RTC_CAMERA_STREAMS.front, GO2RTC_CAMERA_STREAMS.gripper]) {
      try {
        const response = await fetch(`${this.#origin}/api/streams?src=${name}`, {
          redirect: "error", signal: AbortSignal.timeout(3000),
        });
        if (!response.ok) { await response.body?.cancel(); result[name] = { available: false }; continue; }
        const value = JSON.parse(await responseText(response)) as Record<string, unknown> | null;
        if (!value || !Array.isArray(value.producers) || (value.consumers != null && !Array.isArray(value.consumers))) {
          result[name] = { available: false }; continue;
        }
        result[name] = { available: true, producers: value.producers.slice(0, 16).map(v => counters(v)),
          consumers: ((value.consumers ?? []) as unknown[]).slice(0, 16).map(v => counters(v)) };
      } catch { result[name] = { available: false }; }
    }
    return result;
  }

  async configurationMatches(account: CameraAccount, device: DeviceCredentials, pin: string) {
    try {
      const response = await fetch(`${this.#origin}/api/streams?src=${GO2RTC_STREAM}`, {
        redirect: "error", signal: AbortSignal.timeout(3000),
      });
      if (!response.ok) { await response.body?.cancel(); return { available: false }; }
      const value: unknown = JSON.parse(await responseText(response));
      if (!value || typeof value !== "object" || !("producers" in value) || !Array.isArray(value.producers) || value.producers.length !== 1)
        return { available: false };
      const producer = value.producers[0];
      if (!producer || typeof producer.url !== "string" || !producer.url.startsWith("roborock://")) return { available: false };
      const actual = new URL(producer.url), expected = new URL(cameraSource(account, device, pin));
      const same = (key: string) => actual.searchParams.getAll(key).length === 1 && actual.searchParams.get(key) === expected.searchParams.get(key);
      return { available: true, brokerMatches: actual.host === expected.host,
        accountMatches: ["u", "s", "k"].every(same), deviceMatches: ["did", "key"].every(same), pinMatches: same("pin") };
    } catch { return { available: false }; }
  }

  async preloaded(): Promise<boolean> {
    try {
      const response = await fetch(`${this.#origin}/api/preload`, { redirect: "error", signal: AbortSignal.timeout(3000) });
      if (!response.ok) { await response.body?.cancel(); return false; }
      const value: unknown = JSON.parse(await responseText(response));
      return !!value && typeof value === "object" && GO2RTC_STREAM in value;
    } catch { return false; }
  }

  async preload(videoOnly = true): Promise<void> {
    const session = this.#activeSignal();
    const endpoint = new URL("/api/preload", this.#origin);
    endpoint.search = new URLSearchParams({ src: GO2RTC_STREAM, ...(videoOnly ? { video: "" } : {}) }).toString();
    try {
      const response = await fetch(endpoint, { method: "PUT", redirect: "error", signal: AbortSignal.any([session, AbortSignal.timeout(35_000)]) });
      await response.body?.cancel();
      if (!response.ok && !await this.preloaded()) {
        // A read-only config makes go2rtc return 500 after adding the
        // in-memory preload. Only fail if the consumer was not installed.
        throw new Error("go2rtc rejected preload.");
      }
    } catch { throw new DriverError("go2rtc could not retain the robot video stream."); }
  }

  async unload(): Promise<void> {
    const endpoint = new URL("/api/preload", this.#origin);
    endpoint.search = new URLSearchParams({ src: GO2RTC_STREAM }).toString();
    try {
      const response = await fetch(endpoint, { method: "DELETE", redirect: "error", signal: AbortSignal.timeout(5000) });
      await response.body?.cancel();
      if (!response.ok && response.status !== 404) throw new Error("go2rtc rejected preload removal.");
    } catch { throw new DriverError("go2rtc could not release the retained robot video stream."); }
  }

  async frame(camera: ActiveCamera = "front"): Promise<Buffer> {
    const session = this.#activeSignal();
    this.#captures++;
    try {
      await this.#keepWarm();
      session.throwIfAborted();
      const response = await fetch(`${this.#origin}/api/frame.jpeg?src=${GO2RTC_CAMERA_STREAMS[camera]}`, {
        redirect: "error", signal: AbortSignal.any([session, AbortSignal.timeout(30_000)]),
      });
      if (!response.ok || response.headers.get("content-type")?.split(";")[0] !== "image/jpeg" || !response.body) {
        await response.body?.cancel();
        throw new DriverError("go2rtc did not return a camera image.", response.status);
      }
      const reader = response.body.getReader();
      let size = 0;
      const chunks: Uint8Array[] = [];
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          size += value.byteLength;
          if (size > 5 * 1024 * 1024) {
            await reader.cancel();
            throw new DriverError("go2rtc camera image exceeds the size limit.");
          }
          chunks.push(value);
        }
      } finally { reader.releaseLock(); }
      const image = Buffer.concat(chunks);
      if (image.length < 4 || image.readUInt16BE(0) !== 0xffd8 || image.readUInt16BE(image.length - 2) !== 0xffd9) {
        throw new DriverError("go2rtc returned no complete JPEG image.");
      }
      session.throwIfAborted();
      return image;
    } catch (error) {
      if (error instanceof DriverError) throw error;
      throw new DriverError("go2rtc image request failed or timed out.");
    } finally {
      if (--this.#captures === 0 && !session.aborted) {
        this.#idleTimer = setTimeout(() => this.close(), 10_000);
        this.#idleTimer.unref();
      }
    }
  }

  async offer(sdp: string): Promise<{ type: "answer"; sdp: string }> {
    const session = this.#activeSignal();
    if (!sdp.startsWith("v=0") || sdp.length > 128 * 1024) throw new Error("Invalid browser offer.");
    try {
      const response = await fetch(`${this.#origin}/api/webrtc?src=${GO2RTC_STREAM}`, {
        method: "POST", redirect: "error", signal: AbortSignal.any([session, AbortSignal.timeout(35_000)]),
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ type: "offer", sdp }),
      });
      const text = await responseText(response);
      if (!response.ok) {
        // Only classify fixed known phrases; upstream errors can contain credential URLs.
        if (text.includes("wrong pin")) throw new DriverError("The driver reports a rejected camera PIN.");
        if (text.includes("can't get device SDP")) throw new DriverError("go2rtc authenticated but could not obtain the robot's WebRTC answer.");
        if (/CONNACK|not authorized|bad user|connection refused/i.test(text)) {
          throw new DriverError("go2rtc could not authenticate or connect to the Roborock MQTT broker.");
        }
        throw new DriverError(`go2rtc camera startup failed (HTTP ${response.status}).`);
      }
      const value: unknown = JSON.parse(text);
      if (!value || typeof value !== "object" || !("type" in value) || value.type !== "answer" ||
        !("sdp" in value) || typeof value.sdp !== "string" || !value.sdp.startsWith("v=0")) {
        throw new DriverError("go2rtc returned an invalid answer.");
      }
      return { type: "answer", sdp: value.sdp };
    } catch (error) {
      if (error instanceof DriverError) throw error;
      throw new DriverError("go2rtc camera exchange failed or timed out; no image was received.");
    }
  }

  async configure(account: CameraAccount, device: DeviceCredentials, pin: string): Promise<void> {
    const session = this.#activeSignal();
    const source = cameraSource(account, device, pin);
    // PATCH updates the in-memory source. PUT would persist the credential URL to YAML.
    const endpoint = new URL("/api/streams", this.#origin);
    endpoint.search = new URLSearchParams({ name: GO2RTC_STREAM, src: source }).toString();
    try {
      const response = await fetch(endpoint, {
        method: "PATCH", redirect: "error", signal: AbortSignal.any([session, AbortSignal.timeout(5000)]),
      });
      await response.body?.cancel();
      if (!response.ok) throw new Error("Driver rejected camera configuration.");
    } catch {
      // A fetch error may contain the credential-bearing URL. Never retain its cause.
      throw new Error("Could not configure local go2rtc. Confirm its service is running.");
    }
  }
}
