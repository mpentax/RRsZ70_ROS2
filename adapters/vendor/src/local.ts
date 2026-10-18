/*
 * Protocol/crypto adapted from copystring/ioBroker.roborock:
 * src/lib/{localApi,cryptoEngine,messageParser}.ts (retrieved 2026-09-24).
 * Also cross-checked against Python-roborock/python-roborock:
 * roborock/protocol.py, protocols/v1_protocol.py, devices/transport/local_channel.py.
 *
 * MIT License
 * Copyright (c) 2025-2026 copystring <copystring@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
import { createCipheriv, createDecipheriv, createHash, randomInt } from "node:crypto";
import { isIP, Socket } from "node:net";

type Version = "1.0" | "L01";
type StatusResult = { protocol: string; status: unknown };
type CryptoContext = {
  version: Version;
  localKey: string;
  timestamp: number;
  sequence: number;
  random: number;
  connectNonce: number;
  ackNonce: number;
};

const SALT = "TXdfu$jyZ#TZHsg4";
const PORT = 58867;
const MAX_FRAME = 19 + 0xffff + 4;
const MAX_RECEIVED = 512 * 1024;
const TOTAL_TIMEOUT_MS = 12_000;
const HELLO_TIMEOUT_MS = 2_500;
const REQUEST_TIMEOUT_MS = 5_000;

class LocalProtocolError extends Error {}
class HelloTimeoutError extends LocalProtocolError {}

function crc32(data: Buffer): number {
  let crc = 0xffffffff;
  for (const byte of data) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function words(...values: number[]): Buffer {
  const result = Buffer.alloc(values.length * 4);
  values.forEach((value, index) => result.writeUInt32BE(value, index * 4));
  return result;
}

function keyMaterial(context: CryptoContext): string {
  const hex = context.timestamp.toString(16).padStart(8, "0");
  return [5, 6, 3, 7, 1, 2, 0, 4].map((index) => hex[index]).join("") + context.localKey + SALT;
}

function gcmParameters(context: CryptoContext): { key: Buffer; iv: Buffer; aad: Buffer } {
  return {
    key: createHash("sha256").update(keyMaterial(context)).digest(),
    iv: createHash("sha256").update(words(context.sequence, context.random, context.timestamp)).digest().subarray(0, 12),
    aad: words(context.sequence, context.connectNonce, context.ackNonce, context.random, context.timestamp),
  };
}

function encryptPayload(payload: Buffer, context: CryptoContext): Buffer {
  if (context.version === "1.0") {
    const key = createHash("md5").update(keyMaterial(context)).digest();
    const cipher = createCipheriv("aes-128-ecb", key, null);
    return Buffer.concat([cipher.update(payload), cipher.final()]);
  }
  const { key, iv, aad } = gcmParameters(context);
  const cipher = createCipheriv("aes-256-gcm", key, iv);
  cipher.setAAD(aad);
  return Buffer.concat([cipher.update(payload), cipher.final(), cipher.getAuthTag()]);
}

function decryptPayload(payload: Buffer, context: CryptoContext): Buffer {
  if (context.version === "1.0") {
    if (!payload.length || payload.length % 16 !== 0) {
      throw new LocalProtocolError("Invalid V1 encrypted payload length");
    }
    const key = createHash("md5").update(keyMaterial(context)).digest();
    const decipher = createDecipheriv("aes-128-ecb", key, null);
    return Buffer.concat([decipher.update(payload), decipher.final()]);
  }
  if (payload.length < 16) throw new LocalProtocolError("Invalid L01 encrypted payload length");
  const { key, iv, aad } = gcmParameters(context);
  const decipher = createDecipheriv("aes-256-gcm", key, iv);
  decipher.setAAD(aad);
  decipher.setAuthTag(payload.subarray(-16));
  return Buffer.concat([decipher.update(payload.subarray(0, -16)), decipher.final()]);
}

/** @internal Pure codec functions for offline interoperability tests; no network or RPC API. */
export const localProtocolCodec = Object.freeze({ crc32, encryptPayload, decryptPayload });

function prefix(frame: Buffer): Buffer {
  return Buffer.concat([words(frame.length), frame]);
}

function header(version: Version, sequence: number, random: number, timestamp: number, type: number): Buffer {
  const frame = Buffer.alloc(17);
  frame.write(version, 0, "ascii");
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(random, 7);
  frame.writeUInt32BE(timestamp, 11);
  frame.writeUInt16BE(type, 15);
  return frame;
}

function statusRequest(context: CryptoContext, requestId: number): Buffer {
  const payload = encryptPayload(Buffer.from(JSON.stringify({
    dps: { "101": JSON.stringify({ id: requestId, method: "get_status", params: [] }) },
    t: context.timestamp,
  })), context);
  const length = Buffer.alloc(2);
  length.writeUInt16BE(payload.length);
  const body = Buffer.concat([
    header(context.version, context.sequence, context.random, context.timestamp, 4), length, payload,
  ]);
  return prefix(Buffer.concat([body, words(crc32(body))]));
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function parseJson(text: string): unknown {
  // JSON errors can include plaintext excerpts, so never propagate their messages.
  try {
    return JSON.parse(text);
  } catch (error: unknown) {
    if (error instanceof SyntaxError) throw new LocalProtocolError("Invalid local response JSON");
    throw error;
  }
}

function rpcResponse(payload: Buffer, requestId: number): { status: unknown } | undefined {
  const outer = parseJson(new TextDecoder("utf-8", { fatal: true }).decode(payload));
  if (!isRecord(outer)) throw new LocalProtocolError("Invalid local response envelope");
  const dps = isRecord(outer.dps) ? outer.dps : undefined;
  let inner: unknown = dps ? (dps["102"] ?? dps["101"] ?? dps) : outer;
  if (typeof inner === "string") inner = parseJson(inner);
  if (!isRecord(inner)) throw new LocalProtocolError("Invalid local RPC response");
  // Unsolicited data-point updates share PUBLISH with RPC responses.
  if (!Object.hasOwn(inner, "id") && dps && !Object.hasOwn(dps, "102") && !Object.hasOwn(dps, "101")) {
    return undefined;
  }
  if (inner.id !== requestId) throw new LocalProtocolError("Local response request ID mismatch");
  if (inner.error !== undefined && inner.error !== null) {
    throw new LocalProtocolError("Robot rejected the read-only get_status request");
  }
  if (!Object.hasOwn(inner, "result") || inner.result === null || inner.result === undefined ||
      (!isRecord(inner.result) && !Array.isArray(inner.result))) {
    throw new LocalProtocolError("Local get_status returned no status object");
  }
  return { status: inner.result };
}

function attempt(host: string, localKey: string, version: Version, deadline: number): Promise<StatusResult> {
  return new Promise((resolve, reject) => {
    const socket = new Socket();
    const connectNonce = randomInt(1_000, 1_001_000);
    const requestId = randomInt(10_000, 32_768);
    let ackNonce: number | undefined;
    let connected = false;
    let settled = false;
    let completing = false;
    let received = 0;
    let frameCount = 0;
    let buffered: Buffer = Buffer.alloc(0);
    let phaseTimer: ReturnType<typeof setTimeout> | undefined;
    const deadlineTimer = setTimeout(() => fail(new LocalProtocolError("Local status deadline exceeded")),
      Math.max(1, deadline - performance.now()));

    function cleanup(): void {
      clearTimeout(deadlineTimer);
      clearTimeout(phaseTimer);
      socket.removeAllListeners("data");
      socket.removeAllListeners("connect");
      socket.destroy();
      buffered = Buffer.alloc(0);
    }

    function fail(error: Error): void {
      if (settled) return;
      settled = true;
      cleanup();
      reject(error);
    }

    function succeed(result: StatusResult): void {
      if (settled) return;
      settled = true;
      cleanup();
      resolve(result);
    }

    function handleFrame(frame: Buffer): void {
      if (++frameCount > 128) throw new LocalProtocolError("Local frame count limit exceeded");
      if (frame.toString("latin1", 0, 3) !== version) {
        throw new LocalProtocolError("Local protocol version mismatch");
      }
      const sequence = frame.readUInt32BE(3);
      const random = frame.readUInt32BE(7);
      const timestamp = frame.readUInt32BE(11);
      const type = frame.readUInt16BE(15);
      if (ackNonce === undefined) {
        if (type !== 1 || sequence !== 0 || (frame.length !== 17 && frame.length !== 21)) {
          throw new LocalProtocolError("Invalid local session acknowledgement");
        }
        if (frame.length === 21 && frame.readUInt32BE(17) !== 0) {
          throw new LocalProtocolError("Robot rejected the local session");
        }
        ackNonce = random;
        clearTimeout(phaseTimer);
        phaseTimer = setTimeout(() => fail(new LocalProtocolError("Local get_status response timed out")),
          REQUEST_TIMEOUT_MS);
        socket.write(statusRequest({
          version, localKey, connectNonce, ackNonce, sequence: 1,
          random: randomInt(1_000, 1_001_000), timestamp: Math.floor(Date.now() / 1000),
        }, requestId));
        return;
      }
      if (type === 5) {
        if ((frame.length !== 17 && frame.length !== 21) || sequence !== 1 ||
            (frame.length === 21 && frame.readUInt32BE(17) !== 0)) {
          throw new LocalProtocolError("Invalid local publish acknowledgement");
        }
        return;
      }
      if (type !== 4 || frame.length < 23 || sequence < 1 || sequence > 0xffff) {
        throw new LocalProtocolError("Unexpected local session frame");
      }
      const payloadLength = frame.readUInt16BE(17);
      if (frame.length !== payloadLength + 23) throw new LocalProtocolError("Local payload length mismatch");
      if (crc32(frame.subarray(0, -4)) !== frame.readUInt32BE(frame.length - 4)) {
        throw new LocalProtocolError("Local frame CRC32 mismatch");
      }
      const payload = decryptPayload(frame.subarray(19, -4), {
        version, localKey, sequence, random, timestamp, connectNonce, ackNonce,
      });
      const response = rpcResponse(payload, requestId);
      const acknowledgement = prefix(header(version, sequence, 0, 0, 5));
      if (response) {
        completing = true;
        socket.write(acknowledgement, (error) => {
          if (error) fail(new LocalProtocolError("Local publish acknowledgement failed"));
          else succeed({ protocol: version, status: response.status });
        });
      } else {
        socket.write(acknowledgement);
      }
    }

    socket.on("connect", () => {
      connected = true;
      clearTimeout(phaseTimer);
      phaseTimer = setTimeout(() => fail(new HelloTimeoutError("Local protocol handshake timed out")),
        HELLO_TIMEOUT_MS);
      // Current app-style CONNECT has no encrypted payload, payload length, or CRC.
      socket.write(prefix(Buffer.concat([header(version, 0, connectNonce, 0, 0), words(10)])));
    });
    socket.on("data", (chunk: Buffer) => {
      if (settled || completing) return;
      // Only the event boundary catches protocol/crypto failures; all reject, never recover as success.
      try {
        received += chunk.length;
        if (received > MAX_RECEIVED) throw new LocalProtocolError("Local receive byte limit exceeded");
        buffered = Buffer.concat([buffered, chunk]);
        while (!settled && !completing && buffered.length >= 4) {
          const length = buffered.readUInt32BE(0);
          if (length < 17 || length > MAX_FRAME) throw new LocalProtocolError("Invalid local TCP frame length");
          if (buffered.length < length + 4) break;
          const frame = buffered.subarray(4, length + 4);
          buffered = buffered.subarray(length + 4);
          handleFrame(frame);
        }
      } catch (error: unknown) {
        fail(error instanceof LocalProtocolError ? error :
          new LocalProtocolError("Local response could not be decrypted or decoded; check the local key"));
      }
    });
    socket.on("error", () => fail(new LocalProtocolError("Local TCP connection failed")));
    socket.on("close", () => {
      if (!settled) fail(connected && ackNonce === undefined
        ? new HelloTimeoutError("Robot closed the connection during local protocol negotiation")
        : new LocalProtocolError("Local TCP connection closed before status was received"));
    });
    phaseTimer = setTimeout(() => fail(new LocalProtocolError("Local TCP connection timed out")), 3_000);
    socket.connect({ host, port: PORT });
  });
}

/**
 * Read only get_status over a direct local TCP socket. Never discovers devices,
 * opens cloud/MQTT connections, or sends movement/configuration commands.
 * V1 uses legacy AES-ECB/CRC (not authenticated encryption); L01 authenticates
 * payloads and session nonces using AES-GCM.
 */
export async function readLocalStatus(host: string, localKey: string): Promise<StatusResult> {
  if (isIP(host) === 0) throw new LocalProtocolError("A literal robot IP address is required");
  if (typeof localKey !== "string" || Buffer.byteLength(localKey, "utf8") !== 16) {
    throw new LocalProtocolError("A 16-byte Roborock local key is required");
  }
  const deadline = performance.now() + TOTAL_TIMEOUT_MS;
  try {
    return await attempt(host, localKey, "1.0", deadline);
  } catch (error: unknown) {
    // Only an unanswered/closed hello permits trying the other local protocol.
    // Never downgrade/retry after a key, CRC, session, or RPC validation failure.
    if (!(error instanceof HelloTimeoutError)) throw error;
  }
  return attempt(host, localKey, "L01", deadline);
}
