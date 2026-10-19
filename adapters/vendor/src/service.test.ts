import { test } from "node:test";
import assert from "node:assert/strict";
import { createService, type ServiceRobot, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";
import { RpcError } from "./robot.ts";
import { TransportError } from "./rpc-outcome.ts";
const token = "t".repeat(43), key = "1234567890abcdef";
test("RC command observations include successful and uncertain outcomes without redispatch", async t => {
  const { post, get, calls } = await fixture(t, method => { if (method === 'app_rc_end') throw new TransportError('timeout', 'unknown'); return ['ok']; });
  const params=[{velocity:.1,omega:0,duration:400,seqnum:1}];
  await post('/api/driver/rpc',{method:'app_rc_move',params});
  await post('/api/driver/rpc',{method:'app_rc_end',params:[]});
  const recent=await get('/api/commands/recent');
  assert.equal(recent.commands.length,3);
  assert.deepEqual(recent.commands[0].params,params);
  assert.equal(recent.commands[0].dispatch,'sent');
  assert.equal(recent.commands[1].dispatch,'unknown');
  assert.ok(recent.commands[0].replyAt>=recent.commands[0].sentAt);
  assert.notEqual(recent.commands[0].id,recent.commands[1].id);
  await get('/api/commands/recent'); assert.equal(calls.length,3);
});

async function fixture(t: import("node:test").TestContext, behavior?: (method: string) => unknown) {
  const calls: { method: string; params: unknown }[] = [];
  const robot: ServiceRobot = { connection: { state: "connected", connectedAt: 1 }, close() {}, async request(method, params) {
    calls.push({ method, params }); return behavior ? behavior(method) : ["ok"];
  } };
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadDevice() { return { duid: "robot1", host: "127.0.0.1", localKey: key, name: "test", model: "a179", protocolVersion: "L01" }; },
    async saveDevice() {}, async loadCamera() { return null; }, async saveCamera() {} };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({ store, config, robotFactory: () => robot });
  await service.start(); t.after(() => service.stop());
  const address = service.server.address(); assert.ok(address && typeof address === "object");
  const origin = `http://127.0.0.1:${address.port}`;
  config.allowedHosts.push(`127.0.0.1:${address.port}`);
  config.publicOrigin = origin;
  async function post(path: string, body: unknown, authorized = true) {
    const res = await fetch(origin + path, { method: "POST", headers: { Host: "127.0.0.1:57000", "Content-Type": "application/json", ...(authorized ? { Authorization: `Bearer ${token}` } : {}) }, body: JSON.stringify(body) });
    return { status: res.status, body: await res.json() as Record<string, unknown> };
  }
  async function get(path: string) {
    const res = await fetch(origin + path, { headers: { Host: "127.0.0.1:57000", Authorization: `Bearer ${token}` } });
    return await res.json() as Record<string, any>;
  }
  return { calls, post, get, service };
}
test("canonical RPC authenticates, forwards arbitrary vendor methods and preserves parameters", async t => {
  const { calls, post } = await fixture(t);
  assert.equal((await post("/api/driver/rpc", { method: "app_charge" }, false)).status, 401);
  assert.equal(calls.length, 0);
  const reply = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ velocity: 8, omega: 6, duration: 9000, seqnum: 7 }] });
  assert.equal(reply.status, 200); assert.equal(reply.body.dispatch, "sent"); assert.equal(reply.body.physicalCompletion, "unverified");
  assert.deepEqual(calls, [{ method: "app_rc_move", params: [{ velocity: 8, omega: 6, duration: 9000, seqnum: 7 }] }]);
});
test("unknown timeout never latches later calls or retries the previous command", async t => {
  const { calls, post } = await fixture(t, method => { if (method === "app_first") throw new TransportError("timeout secret", "unknown"); return ["ok"]; });
  const first = await post("/api/driver/rpc", { method: "app_first" });
  assert.equal(first.status, 502); assert.equal(first.body.dispatch, "unknown");
  assert.equal((await post("/api/driver/rpc", { method: "app_next" })).status, 200);
  assert.deepEqual(calls.map(c => c.method), ["app_first", "app_next"]);
  assert.equal(JSON.stringify(first.body).includes("timeout secret"), false);
});
test("vendor rejection is correlated response evidence, not a lockout", async t => {
  const { calls, post } = await fixture(t, method => { if (method === "bad_mode") throw new RpcError(-7); return { localKey: key, echo: key, ok: true }; });
  const reply = await post("/api/driver/rpc", { method: "bad_mode" });
  assert.equal(reply.body.response, "vendor_error"); assert.equal(reply.body.vendorCode, -7); assert.equal(reply.body.dispatch, "sent");
  const next = await post("/api/driver/rpc", { method: "app_stop_grasp" });
  assert.equal(next.status, 200); assert.equal(JSON.stringify(next.body).includes(key), false); assert.equal(calls.length, 2);
});
test("malformed envelopes, wrong device and retired workflows do not dispatch", async t => {
  const { calls, post } = await fixture(t);
  for (const body of [{ method: "app_charge", supervisedPermit: {} }, { method: "bad\nmethod" }, { method: "app_charge", transport: "unknown" }, { method: "app_charge", timeoutMs: 0 }])
    assert.equal((await post("/api/driver/rpc", body)).status, 400);
  assert.equal((await post("/api/v1/devices/wrong/rpc", { method: "app_charge" })).status, 503);
  assert.equal((await post("/api/commands/recovery", {})).status, 410);
  assert.equal((await post("/api/docking/start", {})).status, 410);
  assert.equal(calls.length, 0);
});
test("legacy dock and arm stop aliases cannot dispatch outside ROS", async t => {
  const { calls, post } = await fixture(t);
  assert.equal((await post("/api/agent/chassis/dock", {})).status, 503);
  assert.equal((await post("/api/agent/arm/stop-grasp", {})).status, 503);
  assert.deepEqual(calls, []);
});

test("explicit null params pass unchanged rather than becoming an empty list", async t => {
  const { calls, post } = await fixture(t);
  assert.equal((await post("/api/driver/rpc", { method: "vendor_method", params: null })).status, 200);
  assert.deepEqual(calls, [{ method: "vendor_method", params: null }]);
});
test("service uses explicit local protocol independently of cloud inventory", async t => {
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadDevice() { return { duid: "robot1", host: "127.0.0.1", localKey: key, name: "test", model: "a179", protocolVersion: "1.0", localProtocol: "L01" }; },
    async saveDevice() {}, async loadCamera() { return null; }, async saveCamera() {} };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({ store, config });
  t.after(() => service.stop());
  await service.start();
  const address = service.server.address(); assert.ok(address && typeof address === "object");
  config.allowedHosts.push(`127.0.0.1:${address.port}`); config.publicOrigin = `http://127.0.0.1:${address.port}`;
  const response = await fetch(`http://127.0.0.1:${address.port}/api/state`, { headers: { Host: "127.0.0.1:57000", Authorization: `Bearer ${token}` } });
  const state = await response.json() as { connection: { protocol: string }; device: { cloudProtocolVersion: string; localProtocol: string } };
  assert.equal(state.connection.protocol, "L01"); assert.equal(state.device.cloudProtocolVersion, "1.0"); assert.equal(state.device.localProtocol, "L01");
});
test("retired command and estimator routes cannot bypass ROS authority", async t => {
  const { post, get, calls } = await fixture(t);
  for (const path of ['/api/v1/rpc','/api/v1/devices/robot1/rpc','/api/arm/anchor','/api/camera/select'])
    assert.equal((await post(path,{method:'app_arm_move',params:{m:[1,0,0]},anchor:'deployed',camera:'front'})).status,503);
  assert.equal((await get('/api/arm/estimate')).state,undefined);
  assert.equal(calls.length,0);
});
test("repeat-safe commands retry once; a move that may have run is never re-sent; a move that never left is", async t => {
  let failStop = true, failMove = 0;
  const { calls, post } = await fixture(t, method => {
    if (method === "app_rc_stop" && failStop) { failStop = false; throw new TransportError("session dropped", "unknown"); }
    if (method === "app_rc_move" && failMove === 1) { failMove = 2; throw new TransportError("session dropped", "unknown"); }
    if (method === "app_rc_move" && failMove === 3) { failMove = 4; throw new TransportError("no session", "not_sent"); }
    return ["ok"];
  });
  const stop = await post("/api/driver/rpc", { method: "app_rc_stop" });
  assert.equal(stop.status, 200); assert.equal(stop.body.retried, 1);
  failMove = 1;
  const ambiguous = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ omega: 0.5, duration: 400, seqnum: 1 }] });
  assert.equal(ambiguous.body.dispatch, "unknown");
  failMove = 3;
  const unsent = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ omega: 0.5, duration: 400, seqnum: 2 }] });
  assert.equal(unsent.status, 200); assert.equal(unsent.body.retried, 1);
  assert.deepEqual(calls.map(c => c.method), ["app_rc_stop", "app_rc_stop", "app_rc_move", "app_rc_move", "app_rc_move"]);
});

test('native goto is never replayed, even on not_sent; native stop is repeatable', async t => {
  const {calls,post}=await fixture(t,()=>{throw new TransportError('timeout','not_sent');});
  await post('/api/driver/rpc',{method:'app_goto_target',params:[24500,26000]});
  assert.deepEqual(calls.map(c=>c.method),['app_goto_target']);
  await post('/api/driver/rpc',{method:'stop_goto_target',params:[]});
  assert.deepEqual(calls.map(c=>c.method),['app_goto_target','stop_goto_target','stop_goto_target']);
});

test('stop-cancelled queued intent is NOT an unsent retry opportunity',async t=>{
  const {calls,post}=await fixture(t,()=>{throw new TransportError('Command cancelled by priority stop','not_sent','cancelled');});
  const response=await post('/api/driver/rpc',{method:'app_arm_move',params:{m:[1,0,0]}});
  assert.equal(response.status,502);assert.equal(response.body.transportReason,'cancelled');
  assert.equal(response.body.dispatch,'not_sent');assert.equal(calls.length,1);
});

test('private driver logical UUID survives the backend envelope without normalization',async t=>{
  const {post,calls}=await fixture(t);
  assert.equal((await post('/api/driver/rpc',{method:'get_status',commandId:'invalid-source-id'})).status,400);
  assert.equal(calls.length,0);
  const commandId='ABCD1234-1234-4234-8234-123456789ABC'; // synthetic UUID; preserve its literal spelling
  const reply=await post('/api/driver/rpc',{method:'app_arm_move',params:{m:[0,0,1]},commandId});
  assert.equal(reply.body.commandId,commandId);assert.equal(reply.body.requestId,commandId);
  assert.match(reply.body.attemptId as string,/^[0-9a-f-]{36}$/);assert.equal(calls.length,1);
});
