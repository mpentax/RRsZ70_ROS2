/*
 * App-style TCP framing and L01/1.0 crypto adapted from copystring/ioBroker.roborock
 * src/lib/{localApi,cryptoEngine,messageParser}.ts (retrieved 2026-09-24).
 * Copyright (c) 2025-2026 copystring <copystring@gmail.com>, MIT License (see THIRD_PARTY_NOTICES).
 */
import { createCipheriv, createDecipheriv, createHash, randomInt } from "node:crypto";
import { isIP, Socket } from "node:net";
import { defaultRpcTimeoutMs, PRIORITY_STOPS, TransportError } from "./rpc-outcome.ts";

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

class LocalProtocolError extends Error {}

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

export const localProtocolCodec = Object.freeze({ crc32, encryptPayload, decryptPayload });

export interface RobotDiagnostic {
  at: number;
  kind: string;
  method?: string;
  durationMs?: number;
  message?: string;
}

export class RpcError extends Error {
  readonly code: number | undefined;

  constructor(code?: number) {
    super("Robot rejected the RPC request");
    this.name = "RpcError";
    this.code = code;
  }
}

type Version = "1.0" | "L01";
type Connection = {
  state: "disconnected" | "connecting" | "connected";
  protocol: string;
  stage?: "tcp-connect" | "handshake" | "ready";
  lastFailure?: "tcp-connect" | "handshake" | "session";
  connectedAt: number | null;
  lastReceivedAt: number | null;
};
type Pending = {
  responseTimeoutMs: number;
  id: number;
  method: string;
  params: string;
  started: number;
  timer: ReturnType<typeof setTimeout>;
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
  completing: boolean;
  dispatch: "not_sent" | "unknown";
  security?: { endpoint: string; nonce: Buffer };
};
type Session = {
  socket: Socket;
  connectNonce: number;
  ackNonce?: number;
  sequence: number;
  buffered: Buffer;
  frames: number;
  bytes: number;
  windowStarted: number;
  promise: Promise<void>;
  resolve: () => void;
  reject: (error: Error) => void;
  phaseTimer?: ReturnType<typeof setTimeout>;
  partialTimer?: ReturnType<typeof setTimeout>;
  pingTimer?: ReturnType<typeof setTimeout>;
  pongTimer?: ReturnType<typeof setTimeout>;
};

const MAX_FRAME = 23 + 0xffff;
const MAX_PENDING = 32;
const MAX_INFLIGHT = 4;

class ProtocolError extends Error {}

function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function word(value: number): Buffer {
  const bytes = Buffer.alloc(4);
  bytes.writeUInt32BE(value);
  return bytes;
}

function prefix(frame: Buffer): Buffer {
  return Buffer.concat([word(frame.length), frame]);
}

function header(version: Version, type: number, sequence = 0, random = 0, timestamp = 0): Buffer {
  const frame = Buffer.alloc(17);
  frame.write(version);
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(random, 7);
  frame.writeUInt32BE(timestamp, 11);
  frame.writeUInt16BE(type, 15);
  return frame;
}

function json(text: string): unknown {
  try {
    return JSON.parse(text);
  } catch {
    throw new ProtocolError("Invalid local response JSON");
  }
}

/** Generic vendor RPC; disconnected requests connect before dispatch, never replay.
 * close() is terminal. Create a new client to reopen an explicitly closed client.
 * Results may contain sensitive network/camera data: callers must filter them.
 */
export class RobotClient {
  #host: string;
  #localKey: string;
  #protocol: Version;
  #onDiagnostic?: (event: RobotDiagnostic) => void;
  #connection: Connection;
  #session?: Session;
  #closed = false;

  #nextId = randomInt(10_000, 32_768);
  #queued: Pending[] = [];
  #inflight = new Map<number, Pending>();

  constructor(options: {
    host: string;
    localKey: string;
    protocol?: Version;
    onDiagnostic?: (event: RobotDiagnostic) => void;

  }) {
    if (typeof options.host !== "string" || !isIP(options.host)) {
      throw new Error("A literal robot IP address is required");
    }
    if (typeof options.localKey !== "string" || Buffer.byteLength(options.localKey) !== 16) {
      throw new Error("A 16-byte Roborock local key is required");
    }
    if (options.protocol !== undefined && options.protocol !== "1.0" && options.protocol !== "L01") {
      throw new Error("Unsupported local protocol");
    }
    this.#host = options.host;
    this.#localKey = options.localKey;
    this.#protocol = options.protocol ?? "1.0";
    this.#onDiagnostic = options.onDiagnostic;


    this.#connection = {
      state: "disconnected", protocol: this.#protocol, connectedAt: null, lastReceivedAt: null,
    };
  }

  get connection(): Connection {
    return { ...this.#connection };
  }

  connect(): Promise<void> {
    if (this.#closed) return Promise.reject(new Error("Robot client is closed"));
    if (this.#session) return this.#session.promise;
    const { promise, resolve, reject } = Promise.withResolvers<void>();
    const session: Session = {
      socket: new Socket(), connectNonce: randomInt(1_000, 1_001_000), sequence: 0,
      buffered: Buffer.alloc(0), frames: 0, bytes: 0, windowStarted: performance.now(),
      promise, resolve, reject,
    };
    this.#session = session;
    this.#connection = {
      state: "connecting", stage: "tcp-connect", protocol: this.#protocol, connectedAt: null, lastReceivedAt: null,
    };
    session.phaseTimer = setTimeout(() => this.#fail(session, new Error("Local TCP connection timed out")), 3_000);
    session.socket.on("connect", () => {
      if (this.#session !== session) return;
      this.#connection.stage = "handshake";
      clearTimeout(session.phaseTimer);
      session.phaseTimer = setTimeout(() => this.#fail(session, new Error("Local handshake timed out")), 2_500);
      this.#write(session, prefix(Buffer.concat([
        header(this.#protocol, 0, 0, session.connectNonce), word(10),
      ])));
    });
    session.socket.on("data", (chunk: Buffer) => this.#receive(session, chunk));
    session.socket.on("error", () => this.#fail(session, new Error("Local TCP connection failed")));
    session.socket.on("end", () => this.#fail(session, new Error("Local TCP connection ended")));
    session.socket.on("close", () => {
      this.#fail(session, new Error("Local TCP connection closed"));
      session.socket.removeAllListeners();
    });
    try {
      session.socket.connect({ host: this.#host, port: 58867 });
    } catch {
      this.#fail(session, new Error("Local TCP connection failed"));
    }
    this.#diagnostic({ kind: "connecting" });
    return promise;
  }

  async request(method: string, params: unknown = []): Promise<unknown> {
    return this.rpc(method, params);
  }

  rpc(method: string, params: unknown = [], timeoutMs: number = defaultRpcTimeoutMs(method)): Promise<unknown> {
    if (typeof method !== "string" || !/^[A-Za-z][A-Za-z0-9_]{0,127}$/.test(method))
      return Promise.reject(new TransportError("Invalid RPC method", "not_sent"));
    if (!Number.isInteger(timeoutMs) || timeoutMs < 1 || timeoutMs > 120000)
      return Promise.reject(new TransportError("Invalid RPC timeout", "not_sent"));
    return this.#enqueue(method, params, timeoutMs);
  }

  /** One explicit L01 map read. A local ["ok"] is only an ACK; the binary
   * response may arrive on MQTT. No automatic retry or generic security RPC.
   */
  requestMapWithSecurity(security: { endpoint: string; nonce: Buffer }):
    { requestId: number; result: Promise<{ id: number; response: unknown }> } {
    if (this.#protocol !== "L01" || this.#connection.state !== "connected" ||
      !/^[A-Za-z0-9+/]{8}$/.test(security.endpoint) || !Buffer.isBuffer(security.nonce) || security.nonce.length !== 16) {
      throw new Error("Connected L01 session and valid map security are required.");
    }
    // Allocate the request ID synchronously. No other read can interleave before
    // #enqueue reserves it; the transport sequence remains independent.
    const id = this.#nextId;
    const result = this.#enqueue("get_map_v1", [], 10_000,
      { endpoint: security.endpoint, nonce: Buffer.from(security.nonce) })
      .then((response) => ({ id, response }));
    return { requestId: id, result };
  }

  async #enqueue(method: string, params: unknown, responseTimeoutMs = 5000,
    security?: { endpoint: string; nonce: Buffer }): Promise<unknown> {
    if (this.#closed) throw new TransportError("Robot client is closed", "not_sent");
    const priority = PRIORITY_STOPS.has(method);
    if (priority) {
      // A stop invalidates queued actuator intent before reserving its own
      // transport slot. Already-written requests remain uncertain, never replayed.
      const cancelled = this.#queued.filter(p => !/^(get_|app_get_)/.test(p.method) && !PRIORITY_STOPS.has(p.method));
      this.#queued = this.#queued.filter(p => !cancelled.includes(p));
      for (const pending of cancelled) this.#settle(pending, new TransportError('Command cancelled by priority stop', 'not_sent', 'cancelled'));
    }
    if (this.#queued.length + this.#inflight.size >= MAX_PENDING + (priority ? 4 : 0)) {
      throw new TransportError("Local request queue is full", "not_sent");
    }
    let encoded: string | undefined;
    try {
      encoded = JSON.stringify(params);
    } catch {
      throw new TransportError("RPC parameters must be JSON serializable", "not_sent");
    }
    if (encoded === undefined || Buffer.byteLength(encoded) > 32_768) {
      throw new TransportError("Invalid or oversized RPC parameters", "not_sent");
    }
    // Freeze the serialized parameters before queueing; callers cannot mutate queued commands.
    const { promise, resolve, reject } = Promise.withResolvers<unknown>();
    const id = this.#nextId++;
    if (this.#nextId > 0x7fffffff) this.#nextId = 1;
    const dropQueued = (error: Error) => {
      const index = this.#queued.indexOf(pending);
      if (index !== -1) {
        this.#queued.splice(index, 1);
        this.#settle(pending, error);
      }
    };
    const pending: Pending = {
      id, method, params: encoded, started: performance.now(), resolve, reject, completing: false, responseTimeoutMs,
      security, dispatch: "not_sent",
      timer: setTimeout(() => dropQueued(new Error("Local queued request timed out")), Math.min(15_000, responseTimeoutMs)),
    };
    this.#queued.push(pending); // #flush selects the first priority stop, preserving stop/end order.
    // Connection failure rejects the queue in #fail. No request is retried.
    void this.connect().then(() => this.#flush(), (error: unknown) => {
      dropQueued(error instanceof Error ? error : new Error("Local connection failed"));
    });
    return promise;
  }

  close(): void {
    this.#closed = true;
    if (this.#session) this.#fail(this.#session, new Error("Robot client is closed"));
  }

  #diagnostic(event: Omit<RobotDiagnostic, "at">): void {
    try {
      this.#onDiagnostic?.({ at: Date.now(), ...event });
    } catch {
      // An observer must not turn a successful write into an apparent failed request.
      process.emitWarning("Robot diagnostic observer failed");
    }
  }

  #fail(session: Session, error: Error): void {
    if (this.#session !== session) return;
    this.#session = undefined;
    this.#connection.lastFailure = this.#connection.stage === "tcp-connect" ? "tcp-connect" : this.#connection.stage === "handshake" ? "handshake" : "session";
    this.#connection.state = "disconnected";
    this.#connection.connectedAt = null;
    for (const timer of [session.phaseTimer, session.partialTimer, session.pingTimer, session.pongTimer]) {
      clearTimeout(timer);
    }
    session.socket.removeAllListeners("data");
    session.socket.removeAllListeners("connect");
    session.socket.removeAllListeners("end");
    // Keep error/close handlers until destroy completes, to absorb late socket events.
    session.socket.destroy();
    session.buffered = Buffer.alloc(0);
    session.reject(error);
    const pending = [...this.#queued, ...this.#inflight.values()];
    this.#queued = [];
    this.#inflight.clear();
    for (const request of pending) this.#settle(request, error);
    this.#diagnostic({ kind: "disconnected", message: error.message });
  }

  #settle(pending: Pending, error?: Error, value?: unknown): void {
    clearTimeout(pending.timer);
    if (error) pending.reject(error instanceof RpcError || error instanceof TransportError ? error : new TransportError(error.message, pending.dispatch));
    else pending.resolve(value);
    this.#diagnostic({
      kind: error ? "request-error" : "response", method: pending.method,
      durationMs: Math.round(performance.now() - pending.started),
      ...(error ? { message: error.message } : {}),
    });
  }

  #write(session: Session, bytes: Buffer, done?: () => void): void {
    if (this.#session !== session) return;
    if (session.socket.writableLength + bytes.length > 256 * 1024) {
      this.#fail(session, new Error("Local write buffer limit exceeded"));
      return;
    }
    try {
      session.socket.write(bytes, (error) => {
        if (this.#session !== session) return;
        if (error) this.#fail(session, new Error("Local TCP write failed"));
        else done?.();
      });
    } catch {
      this.#fail(session, new Error("Local TCP write failed"));
    }
  }

  #flush(): void {
    const session = this.#session;
    if (!session || session.ackNonce === undefined) return;
    while (this.#session === session && this.#queued.length) {
      const priority = this.#queued.findIndex(p => PRIORITY_STOPS.has(p.method));
      if (this.#inflight.size >= MAX_INFLIGHT && (priority < 0 || this.#inflight.size >= MAX_INFLIGHT+4)) break;
      const pending = this.#queued.splice(priority >= 0 ? priority : 0,1)[0];
      clearTimeout(pending.timer);
      const remaining = pending.responseTimeoutMs - (performance.now()-pending.started);
      if (remaining <= 0) { this.#settle(pending,new Error('Local queued request timed out')); continue; }
      this.#inflight.set(pending.id, pending);
      pending.timer = setTimeout(() => {
        // A slow reply fails only this request (outcome unknown, never replayed).
        // The session stays up; a late reply for this id is ACKed and ignored.
        // A dead link is still detected by the keepalive.
        if (this.#inflight.get(pending.id) !== pending) return;
        this.#inflight.delete(pending.id);
        this.#settle(pending, new Error("Local RPC response timed out"));
        this.#flush();
      }, remaining);
      const sequence = session.sequence = session.sequence % 0xffff + 1;
      const random = randomInt(1_000, 1_001_000);
      const timestamp = Math.floor(Date.now() / 1000);
      try {
        const inner = `{"id":${pending.id},"method":${JSON.stringify(pending.method)},"params":${pending.params}${pending.security
          ? `,"security":{"endpoint":${JSON.stringify(pending.security.endpoint)},"nonce":"${pending.security.nonce.toString("hex").toUpperCase()}"}` : ""}}`;
        const payload = localProtocolCodec.encryptPayload(
          Buffer.from(JSON.stringify({ dps: { "101": inner }, t: timestamp })),
          { version: this.#protocol, localKey: this.#localKey, sequence, random, timestamp,
            connectNonce: session.connectNonce, ackNonce: session.ackNonce },
        );
        if (payload.length > 0xffff) throw new ProtocolError("RPC frame exceeds payload limit");
        const length = Buffer.alloc(2);
        length.writeUInt16BE(payload.length);
        const body = Buffer.concat([header(this.#protocol, 4, sequence, random, timestamp), length, payload]);
        pending.dispatch = "unknown"; // Socket handoff cannot prove delivery to the robot.
        this.#write(session, prefix(Buffer.concat([body, word(localProtocolCodec.crc32(body))])));
      } catch (error: unknown) {
        this.#fail(session, error instanceof ProtocolError ? error : new Error("Local RPC encoding failed"));
      }
    }
  }

  #heartbeat(session: Session): void {
    session.pingTimer = setTimeout(() => {
      if (this.#session !== session) return;
      session.pongTimer = setTimeout(() => this.#fail(session, new Error("Local keepalive timed out")), 10_000);
      // Current app PINGREQ: zero IDs/nonces/timestamp, type 2, no payload length or CRC.
      this.#write(session, prefix(header(this.#protocol, 2)));
    }, 8_000);
  }

  #receive(session: Session, chunk: Buffer): void {
    if (this.#session !== session) return;
    try {
      if (performance.now() - session.windowStarted >= 10_000) {
        session.windowStarted = performance.now();
        session.frames = session.bytes = 0;
      }
      session.bytes += chunk.length;
      if (session.bytes > 1024 * 1024) throw new ProtocolError("Local receive rate limit exceeded");
      session.buffered = Buffer.concat([session.buffered, chunk]);
      while (this.#session === session && session.buffered.length >= 4) {
        const length = session.buffered.readUInt32BE(0);
        if (length < 17 || length > MAX_FRAME) throw new ProtocolError("Invalid local TCP frame length");
        if (session.buffered.length < length + 4) break;
        if (++session.frames > 512) throw new ProtocolError("Local frame rate limit exceeded");
        const frame = session.buffered.subarray(4, length + 4);
        session.buffered = session.buffered.subarray(length + 4);
        clearTimeout(session.partialTimer);
        session.partialTimer = undefined;
        this.#frame(session, frame);
      }
      if (this.#session === session && session.buffered.length && !session.partialTimer) {
        session.partialTimer = setTimeout(
          () => this.#fail(session, new Error("Local partial frame timed out")), 5_000,
        );
      }
    } catch (error: unknown) {
      this.#fail(session, error instanceof ProtocolError ? error :
        new Error("Local response could not be decrypted or decoded"));
    }
  }

  #frame(session: Session, frame: Buffer): void {
    if (frame.toString("latin1", 0, 3) !== this.#protocol) {
      throw new ProtocolError("Local protocol version mismatch");
    }
    const type = frame.readUInt16BE(15);
    const sequence = frame.readUInt32BE(3);
    const random = frame.readUInt32BE(7);
    const timestamp = frame.readUInt32BE(11);
    if (session.ackNonce === undefined) {
      if (type !== 1 || sequence !== 0 || (frame.length !== 17 && frame.length !== 21)) {
        throw new ProtocolError("Invalid local session acknowledgement");
      }
      if (frame.length === 21 && frame.readUInt32BE(17) !== 0) {
        throw new ProtocolError("Robot rejected the local session");
      }
      session.ackNonce = random;
      clearTimeout(session.phaseTimer);
      this.#connection.stage = "ready";
      this.#connection.state = "connected";
      this.#connection.connectedAt = this.#connection.lastReceivedAt = Date.now();
      this.#heartbeat(session);
      session.resolve();
      this.#diagnostic({ kind: "connected" });
      this.#flush();
      return;
    }
    if (type === 2 || type === 3 || type === 5) {
      if ((frame.length !== 17 && frame.length !== 21) ||
          (frame.length === 21 && frame.readUInt32BE(17) !== 0) ||
          ((type === 2 || type === 3) && sequence !== 0)) {
        throw new ProtocolError("Invalid local control frame");
      }
      this.#connection.lastReceivedAt = Date.now();
      if (type === 2) this.#write(session, prefix(header(this.#protocol, 3)));
      if (type === 3 && session.pongTimer) {
        clearTimeout(session.pongTimer);
        session.pongTimer = undefined;
        this.#diagnostic({ kind: "pong" });
        this.#heartbeat(session);
      }
      return;
    }
    // The robot's sequence is a full uint32 that keeps counting across sessions (seen past 70000);
    // only our outgoing sequence wraps at 16 bits. A frame type we don't use is ignored, not fatal:
    // tearing the session down would turn every in-flight command into an unknown outcome.
    if (![4, 6, 7].includes(type) || frame.length < 23) {
      this.#connection.lastReceivedAt = Date.now();
      this.#diagnostic({ kind: "ignored-frame", message: `type ${type}, ${frame.length} bytes` });
      return;
    }
    if (frame.readUInt16BE(17) + 23 !== frame.length) throw new ProtocolError("Local payload length mismatch");
    if (localProtocolCodec.crc32(frame.subarray(0, -4)) !== frame.readUInt32BE(frame.length - 4)) {
      throw new ProtocolError("Local frame CRC32 mismatch");
    }
    const payload = localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
      version: this.#protocol, localKey: this.#localKey, sequence, random, timestamp,
      connectNonce: session.connectNonce, ackNonce: session.ackNonce,
    });
    this.#connection.lastReceivedAt = Date.now();
    const outer = json(new TextDecoder("utf-8", { fatal: true }).decode(payload));
    if (!record(outer)) throw new ProtocolError("Invalid local response envelope");
    const dps = record(outer.dps) ? outer.dps : undefined;
    let inner: unknown = dps ? (dps["102"] ?? dps["101"] ?? dps) : outer;
    if (typeof inner === "string") inner = json(inner);
    const pending = record(inner) && typeof inner.id === "number" ? this.#inflight.get(inner.id) : undefined;
    if (!pending || pending.completing) {
      this.#write(session, prefix(header(this.#protocol, 5, sequence)));
      this.#diagnostic({ kind: "unsolicited" });
      return;
    }
    if (!record(inner)) throw new ProtocolError("Invalid local RPC response");
    let error: Error | undefined;
    if (inner.error !== undefined && inner.error !== null) {
      const code = record(inner.error) && typeof inner.error.code === "number" && Number.isSafeInteger(inner.error.code)
        ? inner.error.code : undefined;
      error = new RpcError(code);
    } else if (!Object.hasOwn(inner, "result")) {
      error = new ProtocolError("Local RPC response has no result");
    }
    pending.completing = true;
    // Received correlated evidence survives a subsequent outbound transport-ACK failure.
    this.#inflight.delete(pending.id);
    this.#settle(pending, error, inner.result);
    this.#write(session, prefix(header(this.#protocol, 5, sequence)), () => this.#flush());
  }
}
