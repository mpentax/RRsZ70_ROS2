import { setTimeout as delay } from "node:timers/promises";
import { ServiceHttpError } from "./service-config.ts";
import { RpcError } from "./robot.ts";
import { TransportError } from "./rpc-outcome.ts";
import { jpegImageDimensions, CAMERA_NAMES } from "@rrsz70/http-contracts";

export { CAMERA_NAMES } from "@rrsz70/http-contracts";
export type CameraName = typeof CAMERA_NAMES[number];
export type ActiveCamera = "front" | "gripper";
export function requestedCamera(value: unknown): ActiveCamera | undefined {
  if (value === undefined) return undefined;
  if (typeof value !== "string" || !CAMERA_NAMES.includes(value as CameraName))
    throw new ServiceHttpError(400, "Unknown camera name.");
  if (value !== "front" && value !== "gripper")
    throw new ServiceHttpError(422, "Unsupported camera selector; only front and gripper cameras are supported.");
  return value;
}
export function cameraFromDimensions({ width, height }: { width: number; height: number }): ActiveCamera | undefined {
  if (width !== 1600) return undefined;
  return height === 896 ? "gripper" : height === 1200 ? "front" : undefined;
}

/** One shared physical camera mode, including switches by the official app. Never restores front. */
export class CameraSelector {
  #tail: Promise<unknown> = Promise.resolve();
  #cached?: { camera: ActiveCamera; at: number };
  readonly rpc: (method: string, params: unknown, timeoutMs?: number) => Promise<unknown>;
  /** Optional hook invoked after a confirmed camera switch. */
  readonly onSwitched?: (camera: ActiveCamera) => Promise<void> | void;
  constructor(rpc: (method: string, params: unknown, timeoutMs?: number) => Promise<unknown>, onSwitched?: (camera: ActiveCamera) => Promise<void> | void) { this.rpc = rpc; this.onSwitched = onSwitched; }
  async #selected(refresh = false): Promise<ActiveCamera> {
    if (!refresh && this.#cached && Date.now() - this.#cached.at < 1000) return this.#cached.camera;
    const response = await this.rpc("get_camera_status", [], 500);
    const value = Array.isArray(response) ? response[0] : response;
    const status = typeof value === "number" ? value : (value as { camera_status?: unknown } | undefined)?.camera_status;
    if (typeof status !== "number" || !Number.isSafeInteger(status) || status < 0)
      throw new ServiceHttpError(502, "Camera selection feedback unavailable.");
    const camera = (status & 0x8000) ? "gripper" : "front";
    this.#cached = { camera, at: Date.now() };
    return camera;
  }
  #enqueue<T>(operation: () => Promise<T>): Promise<T> {
    const job = this.#tail.then(operation);
    this.#tail = job.catch(() => {});
    return job;
  }
  async #select(camera: ActiveCamera): Promise<ActiveCamera> {
    if (await this.#selected(true) === camera) return camera;
    this.#cached = undefined;
    try { await this.rpc("app_change_camera", { flag: camera === "gripper" ? 1 : 0 }, 500); }
    catch (error) {
      if (error instanceof RpcError) throw new ServiceHttpError(409, "Robot refused camera selection; gripper needs pickup mode (app_pick_up_enter_exit {flag:2}) with the arm out.");
      // This firmware switches without replying. Only the expected reply timeout is normal.
      if (!(error instanceof TransportError && /timed out|timeout/i.test(error.message))) throw error;
    }
    const deadline = Date.now() + 3000;
    let active: ActiveCamera | undefined;
    do {
      active = await this.#selected(true);
      if (active === camera) { await this.onSwitched?.(camera); return camera; }
      await delay(150);
    } while (Date.now() < deadline);
    throw new ServiceHttpError(409, `Camera selection not confirmed: requested ${camera}, active ${active ?? "unknown"}. Gripper needs pickup mode (app_pick_up_enter_exit {flag:2}) with the arm out.`);
  }
  async #warmAndSelect(camera: ActiveCamera, warm: (camera: ActiveCamera) => Promise<unknown>): Promise<ActiveCamera> {
    const current = await this.#selected(true);
    if (current !== camera) await warm(current); // switch inside an existing preview with both receivers pre-warmed
    return this.#select(camera);
  }
  select(camera: ActiveCamera, warm: (camera: ActiveCamera) => Promise<unknown>): Promise<ActiveCamera> {
    return this.#enqueue(() => this.#warmAndSelect(camera, warm));
  }
  run(camera: ActiveCamera | undefined, capture: (camera: ActiveCamera) => Promise<Buffer>): Promise<{ frame: Buffer; camera: ActiveCamera; sizeCamera?: ActiveCamera }> {
    return this.#enqueue(async () => {
      if (camera !== undefined) await this.#warmAndSelect(camera, capture);
      const deadline = Date.now() + 6000;
      do {
        let active = await this.#selected();
        const frame = await capture(active);
        // Each go2rtc transcoder gates on decoded H264 input dimensions (`select=eq(iw,1600)*eq(ih,1200|896)`)
        // upstream of `scale`, so pre-switch frames from the other lens are dropped before encoding.
        const sizeCamera = cameraFromDimensions(jpegImageDimensions(frame));
        if (sizeCamera !== undefined && sizeCamera !== active) active = await this.#selected(true);
        if (sizeCamera === undefined || sizeCamera === active) return { frame, camera: active, sizeCamera };
        // Drain stale decoded frames; don't relabel pixels or send another selector.
        await delay(100);
      } while (Date.now() < deadline);
      throw new ServiceHttpError(502, "Camera frame dimensions disagree with live selection feedback; retry the frame read.");
    });
  }
}
