import assert from "node:assert/strict";
import { Socket } from "node:net";
import { test, type TestContext } from "node:test";
import { localProtocolCodec, RobotClient, RpcError, type RobotDiagnostic } from "./robot.ts";
import { TransportError } from "./rpc-outcome.ts";

const KEY = "b8Hj5mFk3QzT7rLp";
const ACK_NONCE = 485592656;
type Version = "1.0" | "L01";
type Rpc = { id: number; method: string; params: unknown; sequence: number };
type Peer = {
  socket: Socket;
  nonce: number;
  version: Version;
  frames: Buffer[];
  rpcs: Rpc[];
  send: (bytes: Buffer) => void;
};

function word(value: number): Buffer {
  const bytes = Buffer.alloc(4);
  bytes.writeUInt32BE(value);
  return bytes;
}

function wrap(frame: Buffer): Buffer {
  return Buffer.concat([word(frame.length), frame]);
}

function control(peer: Peer, type: number, sequence = 0, payload?: Buffer): Buffer {
  const frame = Buffer.alloc(17);
  frame.write(peer.version);
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(type === 1 ? ACK_NONCE : 0, 7);
  frame.writeUInt16BE(type, 15);
  return wrap(Buffer.concat([frame, payload ?? Buffer.alloc(0)]));
}

function reply(peer: Peer, inner: unknown, sequence = 42, options: { outer?: unknown; key?: string; type?: number } = {}): Buffer {
  const timestamp = 1753606905;
  const random = 304251;
  const payload = localProtocolCodec.encryptPayload(
    Buffer.from(JSON.stringify(options.outer ?? { dps: { "102": JSON.stringify(inner) }, t: timestamp })),
    { version: peer.version, localKey: options.key ?? KEY, sequence, timestamp, random,
      connectNonce: peer.nonce, ackNonce: ACK_NONCE },
  );
  const frame = Buffer.alloc(23 + payload.length);
  frame.write(peer.version);
  frame.writeUInt32BE(sequence, 3);
  frame.writeUInt32BE(random, 7);
  frame.writeUInt32BE(timestamp, 11);
  frame.writeUInt16BE(options.type ?? 4, 15);
  frame.writeUInt16BE(payload.length, 17);
  payload.copy(frame, 19);
  frame.writeUInt32BE(localProtocolCodec.crc32(frame.subarray(0, -4)), frame.length - 4);
  return wrap(frame);
}

function accept(peer: Peer, frame: Buffer): void {
  const type = frame.readUInt16BE(15);
  if (type === 0) {
    assert.equal(frame.length, 21);
    assert.equal(frame.readUInt32BE(3), 0);
    assert.equal(frame.readUInt32BE(11), 0);
    assert.equal(frame.readUInt32BE(17), 10);
    const version = frame.toString("ascii", 0, 3);
    assert.ok(version === "1.0" || version === "L01");
    peer.version = version;
    peer.nonce = frame.readUInt32BE(7);
    assert.ok(peer.nonce > 0);
  } else if (type === 4) {
    assert.equal(frame.length, frame.readUInt16BE(17) + 23);
    assert.equal(localProtocolCodec.crc32(frame.subarray(0, -4)), frame.readUInt32BE(frame.length - 4));
    const payload = localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
      version: peer.version, localKey: KEY, timestamp: frame.readUInt32BE(11),
      sequence: frame.readUInt32BE(3), random: frame.readUInt32BE(7),
      connectNonce: peer.nonce, ackNonce: ACK_NONCE,
    });
    const outer = JSON.parse(payload.toString());
    const rpc = JSON.parse(outer.dps["101"]);
    assert.equal(typeof rpc.id, "number");
    peer.rpcs.push({ ...rpc, sequence: frame.readUInt32BE(3) });
  } else {
    assert.ok([2, 3, 5].includes(type));
    assert.equal(frame.length, 17);
    assert.equal(frame.readUInt32BE(7), 0);
    assert.equal(frame.readUInt32BE(11), 0);
    if (type !== 5) assert.equal(frame.readUInt32BE(3), 0);
  }
}

function fixture(t: TestContext, handler: (peer: Peer, type: number) => void,
  options: { version?: Version; silentConnect?: boolean; diagnostic?: (event: RobotDiagnostic) => void } = {}) {
  const peers: Peer[] = [];
  const diagnostics: RobotDiagnostic[] = [];
  const client = new RobotClient({
    host: "127.0.0.1", localKey: KEY, protocol: options.version ?? "L01",
    onDiagnostic: (event) => { diagnostics.push(event); options.diagnostic?.(event); },
  });
  t.mock.method(Socket.prototype, "connect", function(this: Socket, address: unknown) {
    assert.deepEqual(address, { host: "127.0.0.1", port: 58867 });
    const peer: Peer = {
      socket: this, nonce: 0, version: options.version ?? "L01", frames: [], rpcs: [],
      send: (bytes) => { if (!this.destroyed) this.emit("data", bytes); },
    };
    peers.push(peer);
    t.mock.method(this, "write", (bytes: Buffer, callback?: (error?: Error) => void) => {
      assert.equal(bytes.readUInt32BE(0), bytes.length - 4);
      const frame = Buffer.from(bytes.subarray(4));
      peer.frames.push(frame);
      accept(peer, frame);
      queueMicrotask(() => {
        if (!this.destroyed) handler(peer, frame.readUInt16BE(15));
        callback?.();
      });
      return true;
    });
    if (!options.silentConnect) queueMicrotask(() => this.emit("connect"));
    return this;
  });
  t.after(() => { client.close(); for (const peer of peers) peer.socket.destroy(); });
  return { client, peers, diagnostics };
}

async function drain(): Promise<void> {
  await new Promise<void>((resolve) => setImmediate(resolve));
}

test("L01 map read binds an explicit nonce and endpoint to its RPC ID without retry", async (t) => {
  const nonce = Buffer.from("0123456789abcdef");
  const security = { endpoint: "AbCdEf12", nonce };
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) peer.send(reply(peer, { id: peer.rpcs.at(-1)!.id, result: ["ok"] }));
  });
  assert.throws(() => client.requestMapWithSecurity(security), /Connected L01 session/);
  assert.equal(peers.length, 0);
  await client.connect();
  const read = client.requestMapWithSecurity(security);
  const result = await read.result;
  assert.equal(read.requestId, result.id);
  assert.deepEqual(result.response, ["ok"]);
  assert.equal(peers.length, 1);
  assert.equal(peers[0].rpcs.length, 1);
  assert.equal(peers[0].rpcs[0].id, result.id);
  assert.equal(peers[0].rpcs[0].method, "get_map_v1");
  assert.deepEqual(peers[0].rpcs[0].params, []);
  assert.deepEqual(Reflect.get(peers[0].rpcs[0], "security"), {
    endpoint: security.endpoint, nonce: nonce.toString("hex").toUpperCase(),
  });
  nonce.fill(0);
  assert.throws(() => client.requestMapWithSecurity({ ...security, endpoint: "invalid" }), /valid map security/);
  assert.equal(peers[0].rpcs.length, 1);
});

function handshake(peer: Peer, type: number): void {
  if (type === 0) peer.send(control(peer, 1, 0, word(0)));
}

for (const stopMethod of ['app_rc_stop','stop_goto_target']) {
test(`${stopMethod} bypasses four hanging reads and cancels queued actuator intent`, async t => {
  let stopReplied=false;
  const {client,peers}=fixture(t,(peer,type)=>{
    handshake(peer,type);
    const stop=peer.rpcs.find(r=>r.method===stopMethod);
    if (stop && !stopReplied) { stopReplied=true;peer.send(reply(peer,{id:stop.id,result:['ok']})); }
  });
  await client.connect();
  const reads=Array.from({length:4},(_,i)=>client.rpc('get_hanging_'+i,[],10000).catch(e=>e));
  await drain();assert.equal(peers[0].rpcs.length,4);
  const queued=client.rpc('app_arm_move',{m:[1,0,0]},10000).catch(e=>e);
  await drain();assert.equal(peers[0].rpcs.length,4);
  assert.deepEqual(await client.rpc(stopMethod,[],1000),['ok']);
  const cancellation=await queued;
  assert.ok(cancellation instanceof TransportError);
  assert.equal(cancellation.dispatch,'not_sent');assert.equal(cancellation.reason,'cancelled');
  assert.deepEqual(peers[0].rpcs.map(r=>r.method),['get_hanging_0','get_hanging_1','get_hanging_2','get_hanging_3',stopMethod]);
  client.close();await Promise.all(reads);
});
}

for (const version of ["1.0", "L01"] as const) {
  test(`${version}: shared handshake, independent IDs and sequences, out-of-order replies`, async (t) => {
    let replied = false;
    const { client, peers } = fixture(t, (peer, type) => {
      handshake(peer, type);
      if (type === 4 && peer.rpcs.length === 3 && !replied) {
        replied = true;
        const responses = [...peer.rpcs].reverse().map((rpc, i) =>
          reply(peer, { id: rpc.id, result: { method: rpc.method } }, 100 + i));
        peer.send(Buffer.concat(responses));
      }
    }, { version });
    const a = client.connect();
    const b = client.connect();
    assert.equal(a, b);
    assert.equal(client.connection.state, "connecting");
    const results = await Promise.all([
      client.request("get_status"), client.request("get_camera_status"),
      client.request("app_get_arm_joints_data"),
    ]);
    await Promise.all([a, b]);
    assert.deepEqual(results, [
      { method: "get_status" }, { method: "get_camera_status" }, { method: "app_get_arm_joints_data" },
    ]);
    assert.equal(peers.length, 1);
    assert.deepEqual(peers[0].rpcs.map((rpc) => rpc.sequence), [1, 2, 3]);
    assert.equal(new Set(peers[0].rpcs.map((rpc) => rpc.id)).size, 3);
    assert.ok(peers[0].rpcs.every((rpc) => rpc.id !== rpc.sequence));
    assert.deepEqual(peers[0].frames.filter((frame) => frame.readUInt16BE(15) === 5)
      .map((frame) => frame.readUInt32BE(3)), [100, 101, 102]);
    assert.equal(client.connection.state, "connected");
    assert.ok(client.connection.connectedAt && client.connection.lastReceivedAt);
    const snapshot = client.connection;
    snapshot.state = "disconnected";
    assert.equal(client.connection.state, "connected");
  });
}

test("fragments length prefix/hello/payload and ACKs broadcasts and unrelated replies", async (t) => {
  const { client, peers, diagnostics } = fixture(t, (peer, type) => {
    if (type === 0) {
      const bytes = control(peer, 1, 0, word(0));
      peer.send(bytes.subarray(0, 2));
      peer.send(bytes.subarray(2, 12));
      peer.send(bytes.subarray(12));
    }
    if (type === 4) {
      const bytes = Buffer.concat([
        control(peer, 5, 65000),
        reply(peer, null, 31, { outer: { dps: { "122": 81 } } }),
        reply(peer, { id: 900000, error: { message: "SECRET" } }, 32),
        reply(peer, { id: peer.rpcs[0].id, result: [{ battery: 81 }] }, 33),
      ]);
      for (let index = 0; index < bytes.length; index += 7) peer.send(bytes.subarray(index, index + 7));
    }
  });
  assert.deepEqual(await client.request("get_status"), [{ battery: 81 }]);
  assert.deepEqual(peers[0].frames.filter((frame) => frame.readUInt16BE(15) === 5)
    .map((frame) => frame.readUInt32BE(3)), [31, 32, 33]);
  assert.equal(diagnostics.filter((event) => event.kind === "unsolicited").length, 2);
  assert.ok(!JSON.stringify(diagnostics).includes("SECRET"));
});

test("app-style keepalive uses PINGREQ/PINGRESP and responds to peer PINGREQ", async (t) => {
  const { client, peers, diagnostics } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 2) peer.send(control(peer, 3));
  });
  t.mock.timers.enable({ apis: ["setTimeout"] });
  await client.connect();
  t.mock.timers.tick(8_000);
  await drain();
  assert.equal(client.connection.state, "connected");
  assert.equal(diagnostics.filter((event) => event.kind === "pong").length, 1);
  t.mock.timers.tick(8_000);
  await drain();
  assert.equal(diagnostics.filter((event) => event.kind === "pong").length, 2);
  peers[0].send(control(peers[0], 2));
  assert.equal(peers[0].frames.at(-1)?.readUInt16BE(15), 3);
});

test("missing PONG disconnects without automatic reconnect", async (t) => {
  const { client, peers, diagnostics } = fixture(t, handshake);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  await client.connect();
  t.mock.timers.tick(8_000);
  await drain();
  t.mock.timers.tick(10_000);
  assert.equal(client.connection.state, "disconnected");
  assert.ok(peers[0].socket.destroyed);
  assert.equal(peers.length, 1);
  assert.match(diagnostics.at(-1)?.message ?? "", /keepalive/);
});

test("bounded queue/inflight; disconnect rejects all and only an explicit new request reconnects", async (t) => {
  let answer = false;
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4 && answer) {
      for (const rpc of peer.rpcs) peer.send(reply(peer, { id: rpc.id, result: "ok" }));
    }
  });
  const requests = Array.from({ length: 32 }, () => client.request("get_status"));
  const settled = Promise.allSettled(requests);
  await assert.rejects(client.request("get_status"), /queue is full/);
  await drain();
  assert.equal(peers.length, 1);
  assert.equal(peers[0].rpcs.length, 4);
  const oldIds = peers[0].rpcs.map((rpc) => rpc.id);
  peers[0].socket.emit("error", new Error("SECRET KEY"));
  const outcomes = await settled;
  assert.ok(outcomes.every((outcome) => outcome.status === "rejected"));
  assert.equal(client.connection.state, "disconnected");
  await drain();
  assert.equal(peers.length, 1);
  answer = true;
  assert.equal(await client.request("get_camera_status"), "ok");
  assert.equal(peers.length, 2);
  assert.equal(peers[1].rpcs.length, 1);
  assert.equal(peers[1].rpcs[0].method, "get_camera_status");
  assert.ok(!oldIds.includes(peers[1].rpcs[0].id));
  assert.equal(peers[0].socket.listenerCount("data"), 0);
  assert.equal(peers[0].socket.listenerCount("error"), 0);
});

test("queue advances only as in-flight replies finish", async (t) => {
  const { client, peers } = fixture(t, handshake);
  const requests = Array.from({ length: 6 }, () => client.request("get_status"));
  await drain();
  const peer = peers[0];
  assert.equal(peer.rpcs.length, 4);
  peer.send(reply(peer, { id: peer.rpcs[0].id, result: 0 }));
  await drain();
  assert.equal(peer.rpcs.length, 5);
  peer.send(reply(peer, { id: peer.rpcs[1].id, result: 1 }));
  await drain();
  assert.equal(peer.rpcs.length, 6);
  for (let i = 2; i < 6; i++) peer.send(reply(peer, { id: peer.rpcs[i].id, result: i }));
  assert.deepEqual(await Promise.all(requests), [0, 1, 2, 3, 4, 5]);
});

test("a slow reply fails only its own request; the session survives and nothing is replayed", async (t) => {
  const { client, peers } = fixture(t, handshake);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  const slow = client.rpc("app_charge");                 // 5 s default
  const arm = client.rpc("app_arm_out_compartment");     // 30 s default
  const slowResult = assert.rejects(slow, (e: unknown) => /timed out/.test(String(e)) &&
    typeof e === "object" && e !== null && "dispatch" in e && e.dispatch === "unknown");
  await drain();
  const peer = peers[0];
  t.mock.timers.tick(5_000);
  await slowResult;
  assert.equal(client.connection.state, "connected");
  // late reply for the timed-out request is ignored; the arm command is still pending
  peer.send(reply(peer, { id: peer.rpcs[0].id, result: "late" }));
  peer.send(reply(peer, { id: peer.rpcs[1].id, result: "arm-done" }));
  assert.equal(await arm, "arm-done");
  const next = client.request("get_status");
  await drain();
  peer.send(reply(peer, { id: peer.rpcs[2].id, result: "ok" }));
  assert.equal(await next, "ok");
  assert.equal(peers.length, 1);
  assert.deepEqual(peer.rpcs.map(r => r.method), ["app_charge", "app_arm_out_compartment", "get_status"]);
});

test("queued work has its own deadline even as earlier requests keep completing", async (t) => {
  const { client, peers } = fixture(t, handshake);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  const promises = Array.from({ length: 32 }, () => client.rpc("get_status",[],15000));
  const results = Promise.allSettled(promises);
  await drain();
  const peer = peers[0];
  for (let batch = 0; batch < 3; batch++) {
    t.mock.timers.tick(4_000);
    for (const rpc of peer.rpcs.slice(batch * 4, batch * 4 + 4)) {
      peer.send(reply(peer, { id: rpc.id, result: "ok" }));
    }
    peer.send(control(peer, 3));
    await drain();
  }
  assert.equal(peer.rpcs.length, 16);
  t.mock.timers.tick(3_000);
  for (const rpc of peer.rpcs.slice(12)) peer.send(reply(peer, { id: rpc.id, result: "ok" }));
  const outcomes = await results;
  assert.equal(outcomes.filter((outcome) => outcome.status === "fulfilled").length, 16);
  assert.ok(outcomes.slice(16).every((outcome) =>
    outcome.status === "rejected" && /queued request timed out/.test(String(outcome.reason))));
  assert.equal(client.connection.state, "connected");
});

test("failed outbound ACK preserves the already-received vendor result", async (t) => {
  const { client, peers } = fixture(t, handshake);
  const result = client.request("get_status");
  await drain();
  const peer = peers[0];
  t.mock.method(peer.socket, "write", (bytes: Buffer, callback?: (error?: Error) => void) => {
    assert.equal(bytes.readUInt16BE(19), 5);
    queueMicrotask(() => callback?.(new Error("SECRET write error")));
    return false;
  });
  peer.send(reply(peer, { id: peer.rpcs[0].id, result: "ok" }));
  assert.equal(await result, "ok");
  await drain();
  assert.equal(client.connection.state, "disconnected");
  assert.equal(peers.length, 1);
});

test("EOF with a partial response rejects the uncertain request, without replay", async (t) => {
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) {
      peer.send(reply(peer, { id: peer.rpcs[0].id, result: "ok" }).subarray(0, 10));
      peer.socket.emit("end");
    }
  });
  await assert.rejects(client.request("get_status"), /connection ended/);
  assert.equal(client.connection.state, "disconnected");
  assert.equal(peers.length, 1);
  assert.equal(peers[0].rpcs.length, 1);
});

test("RPC error codes are typed, messages sanitized, and sibling requests survive", async (t) => {
  const { client, peers, diagnostics } = fixture(t, handshake);
  const bad = client.request("get_status");
  const good = client.request("get_camera_status");
  const rejection = assert.rejects(bad, (error: unknown) => {
    assert.ok(error instanceof RpcError);
    assert.equal(error.code, -10002);
    assert.equal(error.message, "Robot rejected the RPC request");
    return true;
  });
  await drain();
  const peer = peers[0];
  peer.send(reply(peer, { id: peer.rpcs[0].id, error: { code: -10002, message: `SECRET ${KEY}` } }));
  peer.send(reply(peer, { id: peer.rpcs[1].id, result: { enabled: true } }));
  await rejection;
  assert.deepEqual(await good, { enabled: true });
  assert.equal(client.connection.state, "connected");
  assert.ok(!JSON.stringify(diagnostics).includes(KEY));
  assert.ok(!JSON.stringify(diagnostics).includes("SECRET"));
});

test("missing result rejects only that RPC with a sanitized schema error", async (t) => {
  const { client } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) peer.send(reply(peer, { id: peer.rpcs[0].id, sensitive: "SECRET" }));
  });
  await assert.rejects(client.request("get_status"), { message: "Local RPC response has no result" });
  assert.equal(client.connection.state, "connected");
});

for (const phase of ["TCP", "handshake"] as const) {
  test(`${phase} timeout is bounded, shared, and sends no RPC`, async (t) => {
    const { client, peers } = fixture(t, () => {}, { silentConnect: phase === "TCP" });
    t.mock.timers.enable({ apis: ["setTimeout"] });
    const a = client.connect();
    const b = client.connect();
    assert.equal(a, b);
    const result = assert.rejects(a, /timed out/);
    await drain();
    t.mock.timers.tick(phase === "TCP" ? 3_000 : 2_500);
    await result;
    assert.equal(peers[0].rpcs.length, 0);
    assert.ok(peers[0].socket.destroyed);
  });
}

test("partial idle frame has an absolute deadline, not renewed by trickled bytes", async (t) => {
  const { client, peers } = fixture(t, handshake);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  await client.connect();
  const bytes = reply(peers[0], { id: 123, result: "ok" });
  peers[0].send(bytes.subarray(0, 1));
  t.mock.timers.tick(4_000);
  peers[0].send(bytes.subarray(1, 8));
  t.mock.timers.tick(1_000);
  assert.equal(client.connection.state, "disconnected");
  assert.ok(peers[0].socket.destroyed);
});

const corruptions: { name: string; make: (peer: Peer) => Buffer; error: RegExp }[] = [
  { name: "oversize", make: () => word(0xffffffff), error: /frame length/ },
  { name: "undersize", make: () => word(16), error: /frame length/ },
  { name: "CRC", make: (peer) => {
    const bytes = reply(peer, { id: peer.rpcs[0].id, result: "SECRET" });
    bytes[bytes.length - 1] ^= 1;
    return bytes;
  }, error: /CRC32/ },
  { name: "wrong key", make: (peer) => reply(peer, {}, 42, { key: "0123456789012345" }), error: /decrypted/ },
  { name: "bad JSON", make: (peer) => reply(peer, null, 42, { outer: { dps: { "102": "SECRET invalid" } } }), error: /JSON/ },
  { name: "frame flood", make: (peer) => Buffer.concat(Array.from({ length: 513 }, () => control(peer, 5, 1))), error: /frame rate/ },
];
for (const corruption of corruptions) {
  test(`rejects ${corruption.name} with safe error and no retry`, async (t) => {
    const { client, peers, diagnostics } = fixture(t, (peer, type) => {
      handshake(peer, type);
      if (type === 4) peer.send(corruption.make(peer));
    });
    await assert.rejects(client.request("get_status"), corruption.error);
    assert.equal(peers.length, 1);
    assert.equal(client.connection.state, "disconnected");
    assert.ok(!JSON.stringify(diagnostics).includes(KEY));
    assert.ok(!JSON.stringify(diagnostics).includes("SECRET"));
  });
}

test("close rejects active and queued requests, cancels timers and prevents reopen", async (t) => {
  const { client, peers } = fixture(t, handshake);
  t.mock.timers.enable({ apis: ["setTimeout"] });
  const promises = Array.from({ length: 5 }, () => client.request("get_status"));
  const result = Promise.allSettled(promises);
  await drain();
  client.close();
  assert.ok((await result).every((entry) => entry.status === "rejected" && /closed/.test(String(entry.reason))));
  await assert.rejects(client.connect(), /closed/);
  await assert.rejects(client.request("get_status"), /closed/);
  t.mock.timers.tick(60_000);
  assert.equal(peers.length, 1);
  assert.equal(client.connection.state, "disconnected");
});

test("close during handshake rejects the shared connection promise and queued request", async (t) => {
  const { client, peers } = fixture(t, () => {});
  const connecting = assert.rejects(client.connect(), /closed/);
  const pending = assert.rejects(client.request("get_status"), /closed/);
  await drain();
  client.close();
  await Promise.all([connecting, pending]);
  assert.equal(peers[0].rpcs.length, 0);
  await drain();
  for (const event of ["connect", "data", "error", "close", "end"]) {
    assert.equal(peers[0].socket.listenerCount(event), 0);
  }
});

test("rejects unsafe/unknown methods and invalid params without connecting", async (t) => {
  const { client, peers } = fixture(t, handshake);
  for (const method of ["send_sdp_to_robot\napp_start", "", "x".repeat(129)]) {
    await assert.rejects(client.request(method), /Invalid RPC method/);
  }
  const circular: { self?: unknown } = {};
  circular.self = circular;
  for (const value of [circular, 1n, () => {}, "x".repeat(32_769)]) {
    await assert.rejects(client.request("get_status", value), /parameters/);
  }
  assert.equal(peers.length, 0);
});

test("camera params are captured before mutation and cannot replace the method", async (t) => {
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) peer.send(reply(peer, { id: peer.rpcs[0].id, result: null }));
  });
  const params = { sdp: "original", method: "app_start" };
  const result = client.request("send_sdp_to_robot", params);
  params.sdp = "mutated";
  assert.equal(await result, null);
  assert.equal(peers[0].rpcs[0].method, "send_sdp_to_robot");
  assert.deepEqual(peers[0].rpcs[0].params, { sdp: "original", method: "app_start" });
});

test("invalid credentials/host are rejected locally without dumping them", () => {
  assert.throws(() => new RobotClient({ host: "robot.example", localKey: KEY }), /literal robot IP/);
  assert.throws(() => new RobotClient({ host: "127.0.0.1", localKey: "SECRET" }), /16-byte/);
});


test("generic vendor writes queue normally and do not need supervision permission or known prior completion", async t => {
  const answered = new Set<number>();
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) for (const rpc of [...peer.rpcs]) if (!answered.has(rpc.id)) { answered.add(rpc.id); peer.send(reply(peer, { id: rpc.id, result: ["ok"] }, 100 + answered.size)); }
  });
  const methods = ["app_charge", "app_rc_move", "app_open_gripper", "vendor_new_method", "app_arm_out_compartment"];
  const values = await Promise.all(methods.map(method => client.rpc(method, { unrestricted: true })));
  assert.ok(values.every(v => JSON.stringify(v) === '["ok"]'));
  assert.deepEqual(peers[0].rpcs.map(r => r.method), methods);
});
test("request after an unknown-outcome timeout reuses the session without replay", async t => {
  const { client, peers } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4 && peer.rpcs.at(-1)!.method === "app_next") peer.send(reply(peer, { id: peer.rpcs.at(-1)!.id, result: ["ok"] }, 5));
  });
  await assert.rejects(client.rpc("app_first", [], 5), error => !!error && typeof error === "object" && "dispatch" in error && error.dispatch === "unknown");
  assert.deepEqual(await client.rpc("app_next"), ["ok"]);
  assert.equal(peers.length, 1);
  assert.deepEqual(peers.flatMap(p => p.rpcs.map(r => r.method)), ["app_first", "app_next"]);
});

test("robot sequences past 16 bits and unknown frame types keep the session and the command alive", async (t) => {
  // Real Z70 after a long afternoon: replies arrived with sequence 70592, which used to kill the session.
  const { client, peers, diagnostics } = fixture(t, (peer, type) => {
    handshake(peer, type);
    if (type === 4) {
      const id = peer.rpcs.at(-1)!.id;
      peer.send(Buffer.concat([reply(peer, { id: 999_999, result: ["push"] }, 70_591, { type: 9 }), reply(peer, { id, result: ["ok"] }, 70_592)]));
    }
  });
  assert.deepEqual(await client.rpc("app_rc_move", [{ velocity: 0, omega: 0.5, duration: 400, seqnum: 2 }]), ["ok"]);
  assert.deepEqual(await client.rpc("get_status", []), ["ok"]);
  assert.equal(peers.length, 1);
  assert.equal(client.connection.state, "connected");
  assert.ok(diagnostics.some((d) => d.kind === "ignored-frame"));
  assert.ok(!diagnostics.some((d) => d.kind === "disconnected"));
});
