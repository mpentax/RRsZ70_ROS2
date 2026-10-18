import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { createServer, Socket } from "node:net";
import { test } from "node:test";
import type { TestContext } from "node:test";
import { localProtocolCodec, readLocalStatus } from "./local.ts";

const KEY = "b8Hj5mFk3QzT7rLp";
const ACK_NONCE = 485592656;
type Version = "1.0" | "L01";
type Peer = {
  socket: Socket;
  closed: Promise<void>;
  version: Version;
  connectNonce: number;
  requestId: number;
  requests: number;
  acknowledgements: number[];
};
type FrameHandler = (frame: Buffer, peer: Peer) => void;

function u32(value: number): Buffer {
  const bytes = Buffer.alloc(4);
  bytes.writeUInt32BE(value);
  return bytes;
}

function wrap(frame: Buffer): Buffer {
  return Buffer.concat([u32(frame.length), frame]);
}

function control(version: Version, type: number, sequence = 0, random = 0, payload?: Buffer): Buffer {
  const frame = Buffer.alloc(17);
  frame.write(version);
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(random, 7);
  frame.writeUInt16BE(type, 15);
  return wrap(Buffer.concat([frame, payload ?? Buffer.alloc(0)]));
}

function hello(peer: Peer): Buffer {
  return control(peer.version, 1, 0, ACK_NONCE, u32(0));
}

function response(peer: Peer, inner: unknown = { id: peer.requestId, result: [{ battery: 81, state: 8 }] },
  options: { key?: string; ackNonce?: number; version?: Version; sequence?: number; outer?: unknown } = {}): Buffer {
  const version = options.version ?? peer.version;
  const sequence = options.sequence ?? 42;
  const timestamp = 1753606905;
  const random = 304251;
  const payload = localProtocolCodec.encryptPayload(
    Buffer.from(JSON.stringify(options.outer ?? { dps: { "102": JSON.stringify(inner) }, t: timestamp })),
    { version, sequence, random, timestamp, localKey: options.key ?? KEY,
      connectNonce: peer.connectNonce, ackNonce: options.ackNonce ?? ACK_NONCE },
  );
  const frame = Buffer.alloc(19 + payload.length + 4);
  frame.write(version);
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(random, 7);
  frame.writeUInt32BE(timestamp, 11);
  frame.writeUInt16BE(4, 15);
  frame.writeUInt16BE(payload.length, 17);
  payload.copy(frame, 19);
  frame.writeUInt32BE(localProtocolCodec.crc32(frame.subarray(0, -4)), frame.length - 4);
  return wrap(frame);
}

function acceptRequest(frame: Buffer, peer: Peer): void {
  assert.equal(frame.readUInt16BE(15), 4);
  assert.equal(frame.readUInt32BE(3), 1);
  assert.equal(frame.length, frame.readUInt16BE(17) + 23);
  assert.equal(frame.readUInt32BE(frame.length - 4), localProtocolCodec.crc32(frame.subarray(0, -4)));
  const payload = localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
    version: peer.version, localKey: KEY, timestamp: frame.readUInt32BE(11),
    sequence: frame.readUInt32BE(3), random: frame.readUInt32BE(7),
    connectNonce: peer.connectNonce, ackNonce: ACK_NONCE,
  });
  const outer: unknown = JSON.parse(payload.toString());
  assert.ok(typeof outer === "object" && outer !== null && "dps" in outer);
  assert.ok(typeof outer.dps === "object" && outer.dps !== null && "101" in outer.dps);
  assert.equal(typeof outer.dps["101"], "string");
  const inner: unknown = JSON.parse(String(outer.dps["101"]));
  assert.ok(typeof inner === "object" && inner !== null && "id" in inner);
  assert.equal(typeof inner.id, "number");
  assert.ok(typeof inner.id === "number");
  peer.requestId = inner.id;
  peer.requests++;
  assert.deepEqual(inner, { id: peer.requestId, method: "get_status", params: [] });
}

async function robot(t: TestContext, handler: FrameHandler): Promise<{ peers: Peer[]; read: () => Promise<{ protocol: string; status: unknown }> }> {
  const peers: Peer[] = [];
  const server = createServer((socket) => {
    const peer: Peer = {
      socket, closed: new Promise<void>((resolve) => socket.once("close", resolve)),
      version: "1.0", connectNonce: 0, requestId: 0, requests: 0, acknowledgements: [],
    };
    peers.push(peer);
    let pending: Buffer = Buffer.alloc(0);
    socket.on("data", (chunk: Buffer) => {
      pending = Buffer.concat([pending, chunk]);
      while (pending.length >= 4 && pending.length >= pending.readUInt32BE(0) + 4) {
        const length = pending.readUInt32BE(0);
        const frame = pending.subarray(4, length + 4);
        pending = pending.subarray(length + 4);
        const type = frame.readUInt16BE(15);
        if (type === 0) {
          assert.equal(frame.length, 21);
          assert.equal(frame.readUInt32BE(3), 0);
          assert.equal(frame.readUInt32BE(11), 0);
          assert.equal(frame.readUInt32BE(17), 10);
          const version = frame.toString("ascii", 0, 3);
          assert.ok(version === "1.0" || version === "L01");
          peer.version = version;
          peer.connectNonce = frame.readUInt32BE(7);
          assert.ok(peer.connectNonce > 0);
        } else if (type === 4) {
          acceptRequest(frame, peer);
        } else {
          assert.equal(type, 5, "The transport may send only CONNECT, get_status PUBLISH, and PUBACK");
          peer.acknowledgements.push(frame.readUInt32BE(3));
        }
        handler(frame, peer);
      }
    });
  });
  await new Promise<void>((resolve, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", resolve);
  });
  const address = server.address();
  assert.ok(address && typeof address !== "string");
  const originalConnect = Socket.prototype.connect;
  t.mock.method(Socket.prototype, "connect", function(this: Socket, options: unknown) {
    assert.deepEqual(options, { host: "127.0.0.1", port: 58867 });
    return originalConnect.bind(this)({ host: "127.0.0.1", port: address.port });
  });
  t.after(async () => {
    for (const peer of peers) peer.socket.destroy();
    await new Promise<void>((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
  });
  return { peers, read: () => readLocalStatus("127.0.0.1", KEY) };
}

test("CRC32 matches the standard check vector", () => {
  assert.equal(localProtocolCodec.crc32(Buffer.from("123456789")), 0xcbf43926);
  assert.equal(localProtocolCodec.crc32(Buffer.alloc(0)), 0);
});

test("L01 matches upstream cryptoEngine.test.ts official vector", () => {
  // Fixture from copystring/ioBroker.roborock, under the MIT notice in local.ts.
  const context = {
    version: "L01" as const, localKey: KEY, timestamp: 1753606905, sequence: 1,
    random: 304251, connectNonce: 893563, ackNonce: ACK_NONCE,
  };
  const plain = Buffer.from('{"dps":{"101":"{\\"id\\":1806,\\"method\\":\\"get_prop\\",\\"params\\":[\\"get_status\\"]}"},"t":1753606905}');
  const encrypted = Buffer.from([
    253,96,200,218,202,28,202,230,127,96,119,71,123,250,157,55,24,154,56,215,91,60,74,144,124,36,53,211,193,70,238,132,216,249,149,151,
    227,225,87,26,1,89,97,206,170,77,100,188,54,149,250,224,36,195,65,103,55,215,113,80,52,29,226,156,173,47,149,191,175,83,35,88,241,
    43,191,248,159,20,15,239,91,30,226,132,195,171,254,59,131,165,119,145,10,114,5,109,171,77,90,117,177,130,209,160,203,161,69,227,
    228,80,243,146,116,67,
  ]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), encrypted);
  assert.deepEqual(localProtocolCodec.decryptPayload(encrypted, context), plain);
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, ackNonce: ACK_NONCE + 1 }));
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, connectNonce: context.connectNonce + 1 }));
});

test("V1 follows the independent upstream MD5 timestamp permutation and AES-ECB specification", () => {
  const context = {
    version: "1.0" as const, localKey: KEY, timestamp: 0x12345678,
    sequence: 1, random: 2, connectNonce: 3, ackNonce: 4,
  };
  const key = createHash("md5").update("67482315" + KEY + "TXdfu$jyZ#TZHsg4").digest();
  const cipher = createCipheriv("aes-128-ecb", key, null);
  const plain = Buffer.from("read-only status");
  const expected = Buffer.concat([cipher.update(plain), cipher.final()]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), expected);
  assert.deepEqual(localProtocolCodec.decryptPayload(expected, context), plain);
});

for (const version of ["1.0", "L01"] as const) {
  test(`${version}: authenticates get_status with fragmented hello and coalesced ACK/status frames`, { timeout: 2_000 }, async (t) => {
    const fixture = await robot(t, (frame, peer) => {
      const type = frame.readUInt16BE(15);
      if (type === 0) {
        if (peer.version !== version) { peer.socket.end(); return; }
        const acknowledgement = hello(peer);
        peer.socket.write(acknowledgement.subarray(0, 2));
        setImmediate(() => peer.socket.write(acknowledgement.subarray(2)));
      }
      if (type === 4) {
        const bytes = Buffer.concat([control(version, 5, 1), response(peer)]);
        peer.socket.write(bytes.subarray(0, 8));
        setImmediate(() => peer.socket.write(bytes.subarray(8, 31)));
        setImmediate(() => peer.socket.write(bytes.subarray(31)));
      }
    });
    assert.deepEqual(await fixture.read(), { protocol: version, status: [{ battery: 81, state: 8 }] });
    assert.equal(fixture.peers.reduce((sum, peer) => sum + peer.requests, 0), 1);
    await Promise.all(fixture.peers.map((peer) => peer.closed));
    assert.ok(fixture.peers.every((peer) => peer.socket.destroyed));
    assert.deepEqual(fixture.peers.at(-1)?.acknowledgements, [42]);
  });
}

test("ignores authenticated unsolicited datapoints, not mismatched RPC responses", async (t) => {
  const fixture = await robot(t, (frame, peer) => {
    if (frame.readUInt16BE(15) === 0) peer.socket.write(hello(peer));
    if (frame.readUInt16BE(15) === 4) {
      peer.socket.write(Buffer.concat([
        response(peer, undefined, { outer: { dps: { "122": 81 } }, sequence: 41 }),
        response(peer),
      ]));
    }
  });
  assert.equal((await fixture.read()).protocol, "1.0");
});

const badResponses: { name: string; make: (peer: Peer) => Buffer; message: RegExp }[] = [
  { name: "CRC mismatch", make: (peer) => { const bytes = response(peer); bytes[bytes.length - 1] ^= 1; return bytes; }, message: /CRC32/ },
  { name: "wrong RPC ID", make: (peer) => response(peer, { id: peer.requestId + 1, result: [{}] }), message: /request ID/ },
  { name: "error response", make: (peer) => response(peer, { id: peer.requestId, error: { message: "SECRET" } }), message: /rejected/ },
  { name: "missing result", make: (peer) => response(peer, { id: peer.requestId }), message: /no status/ },
  { name: "scalar result", make: (peer) => response(peer, { id: peer.requestId, result: "unknown_method" }), message: /no status/ },
  { name: "oversized prefix", make: () => u32(0xffffffff), message: /frame length/ },
  { name: "undersized prefix", make: () => u32(16), message: /frame length/ },
  { name: "payload length mismatch", make: (peer) => { const bytes = response(peer); bytes.writeUInt16BE(0, 21); return bytes; }, message: /payload length/ },
  { name: "version change", make: (peer) => response(peer, undefined, { version: "L01" }), message: /version mismatch/ },
  { name: "duplicate CONNACK", make: (peer) => hello(peer), message: /Unexpected/ },
  { name: "wrong PUBACK ID", make: (peer) => control(peer.version, 5, 7), message: /publish acknowledgement/ },
];
for (const scenario of badResponses) {
  test(`rejects ${scenario.name} without retrying or exposing payload secrets`, async (t) => {
    const fixture = await robot(t, (frame, peer) => {
      if (frame.readUInt16BE(15) === 0) peer.socket.write(hello(peer));
      if (frame.readUInt16BE(15) === 4) peer.socket.write(scenario.make(peer));
    });
    await assert.rejects(fixture.read(), (error: unknown) => {
      assert.ok(error instanceof Error);
      assert.match(error.message, scenario.message);
      assert.ok(!error.message.includes(KEY) && !error.message.includes("SECRET"));
      return true;
    });
    assert.equal(fixture.peers.length, 1);
  });
}

for (const nonce of ["ackNonce", "connectNonce"] as const) {
  test(`L01 rejects authentication under another session ${nonce}`, async (t) => {
    const fixture = await robot(t, (frame, peer) => {
      if (frame.readUInt16BE(15) === 0) {
        if (peer.version === "1.0") peer.socket.end();
        else peer.socket.write(hello(peer));
      }
      if (frame.readUInt16BE(15) === 4) {
        const changed = nonce === "connectNonce" ? { ...peer, connectNonce: peer.connectNonce + 1 } : peer;
        peer.socket.write(response(changed, undefined, { ackNonce: nonce === "ackNonce" ? ACK_NONCE + 1 : ACK_NONCE }));
      }
    });
    await assert.rejects(fixture.read(), /could not be decrypted/);
    assert.equal(fixture.peers.length, 2);
  });
}

for (const scenario of ["sequence", "return code", "length"] as const) {
  test(`rejects invalid CONNACK ${scenario} before sending an RPC`, async (t) => {
    const fixture = await robot(t, (frame, peer) => {
      if (frame.readUInt16BE(15) === 0) {
        peer.socket.write(control(peer.version, 1, scenario === "sequence" ? 1 : 0, ACK_NONCE,
          scenario === "length" ? Buffer.alloc(2) : u32(scenario === "return code" ? 1 : 0)));
      }
    });
    await assert.rejects(fixture.read(), /acknowledgement|rejected/);
    assert.equal(fixture.peers[0].requests, 0);
    assert.equal(fixture.peers.length, 1);
  });
}

test("bounds silent hello attempts and closes both sockets without sending an RPC", async (t) => {
  const fixture = await robot(t, (frame) => {
    if (frame.readUInt16BE(15) === 0) t.mock.timers.tick(2_500);
  });
  t.mock.timers.enable({ apis: ["setTimeout"] });
  await assert.rejects(fixture.read(), /handshake timed out/);
  assert.equal(fixture.peers.length, 2);
  assert.ok(fixture.peers.every((peer) => peer.requests === 0));
});

test("bounds an incomplete response even when bytes keep arriving", async (t) => {
  const fixture = await robot(t, (frame, peer) => {
    if (frame.readUInt16BE(15) === 0) peer.socket.write(hello(peer));
    if (frame.readUInt16BE(15) === 4) {
      peer.socket.write(response(peer).subarray(0, 10));
      t.mock.timers.tick(5_000);
    }
  });
  t.mock.timers.enable({ apis: ["setTimeout"] });
  await assert.rejects(fixture.read(), /timed out/);
  assert.equal(fixture.peers.length, 1);
});

test("validates literal host and local key before any network connection", async (t) => {
  t.mock.method(Socket.prototype, "connect", () => { throw new Error("Unexpected network activity"); });
  await assert.rejects(readLocalStatus("robot.example", KEY), /literal robot IP/);
  await assert.rejects(readLocalStatus("127.0.0.1", ""), /16-byte/);
  await assert.rejects(readLocalStatus("127.0.0.1", "short"), /16-byte/);
});

test("bounds connection establishment and destroys the pending socket", async (t) => {
  let pending: Socket | undefined;
  t.mock.method(Socket.prototype, "connect", function(this: Socket) {
    pending = this;
    return this;
  });
  t.mock.timers.enable({ apis: ["setTimeout"] });
  const result = readLocalStatus("127.0.0.1", KEY);
  t.mock.timers.tick(3_000);
  await assert.rejects(result, /connection timed out/);
  assert.ok(pending?.destroyed);
});

test("a connection error is terminal and never retries another protocol", async (t) => {
  let calls = 0;
  t.mock.method(Socket.prototype, "connect", function(this: Socket) {
    calls++;
    queueMicrotask(() => this.emit("error", new Error("SECRET")));
    return this;
  });
  await assert.rejects(readLocalStatus("127.0.0.1", KEY), { message: "Local TCP connection failed" });
  assert.equal(calls, 1);
});

test("rejects a frame flood under the per-session limit", async (t) => {
  const fixture = await robot(t, (frame, peer) => {
    if (frame.readUInt16BE(15) === 0) peer.socket.write(hello(peer));
    if (frame.readUInt16BE(15) === 4) {
      peer.socket.write(Buffer.concat(Array.from({ length: 129 }, () => control(peer.version, 5, 1))));
    }
  });
  await assert.rejects(fixture.read(), /frame count limit/);
});

test("rejects a truncated response when the socket closes", async (t) => {
  const fixture = await robot(t, (frame, peer) => {
    if (frame.readUInt16BE(15) === 0) peer.socket.write(hello(peer));
    if (frame.readUInt16BE(15) === 4) peer.socket.end(response(peer).subarray(0, 10));
  });
  await assert.rejects(fixture.read(), /closed before status/);
  assert.equal(fixture.peers.length, 1);
});
