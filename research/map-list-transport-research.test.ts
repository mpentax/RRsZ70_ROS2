import assert from "node:assert/strict";
import { test } from "node:test";
import { researchSplitMapList, type ResearchCloud, type ResearchLocal, type CloudInspection } from "./legacy-ts/map-list-transport-research.ts";

class FakeCloud implements ResearchCloud {
  connected = true;
  events: string[] = [];
  callback?: (topic: string, frame: Uint8Array) => void;
  disconnect?: () => void;
  allowSub = true;
  subscribe(topic: string) { this.events.push(`sub:${topic}`); return Promise.resolve(this.allowSub); }
  listen(frame: (topic: string, bytes: Uint8Array) => void, disconnect: () => void) {
    this.events.push("listen"); this.callback = frame; this.disconnect = disconnect;
    return () => { this.events.push("unlisten"); this.callback = undefined; };
  }
  emit(topic: string, response: object) { this.callback?.(topic, Buffer.from(JSON.stringify(response))); }
  drop() { this.connected = false; this.disconnect?.(); }
  close() { this.events.push("cloud-close"); this.connected = false; }
}
class FakeLocal implements ResearchLocal {
  events: string[] = [];
  sends = 0;
  resolve?: (result: unknown) => void;
  reject?: (error: unknown) => void;
  onSend?: () => void;
  connect() { this.events.push("connect"); return Promise.resolve(); }
  sendOnce(method: "get_multi_maps_list", params: readonly []) {
    assert.equal(method, "get_multi_maps_list"); assert.deepEqual(params, []);
    this.events.push("send"); this.sends++; const result = new Promise<unknown>((resolve, reject) => {
      this.resolve = resolve; this.reject = reject;
    });
    this.onSend?.(); return { requestId: 41, result };
  }
  close() { this.events.push("local-close"); }
}
const topic = "selected/fixture";
function fixture() {
  const cloud = new FakeCloud(), local = new FakeLocal();
  const inspect = (frame: Uint8Array): CloudInspection => {
    try {
      const response = JSON.parse(Buffer.from(frame).toString()) as { id: number; result?: unknown; rejected?: boolean; valid?: boolean };
      if (response.valid === false) return { kind: "invalid" }; // fake CRC/decryption failure
      return { kind: "bound", reply: response };
    } catch { return { kind: "invalid" }; }
  };
  const run = (windowMs = 15) => researchSplitMapList({ cloud, local, topic, inspect, windowMs, setupMs: 25 });
  return { cloud, local, run };
}
const list = { map_info: { map: [{ mapFlag: 987654, mapName: "PRIVATE-NAME", geometry: "PRIVATE-GEOMETRY" }] } };

test("subscribe+listen before one synchronous reservation; LAN ACK then cloud list, redacted", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => { assert.deepEqual(cloud.events.slice(0, 2), ["listen", `sub:${topic}`]);
    local.resolve?.(["ok"]); queueMicrotask(() => cloud.emit(topic, { id: 41, result: list })); };
  const output = await run();
  assert.deepEqual(output, { outcome: "LAN_ACK + cloud_list", lanAck: true, lanList: false,
    cloudAck: false, cloudList: true, itemCount: 1 });
  assert.equal(local.sends, 1);
  assert.ok(!JSON.stringify(output).includes("PRIVATE")); assert.ok(!JSON.stringify(output).includes("987654"));
  assert.deepEqual(local.events, ["connect", "send", "local-close"]);
  assert.deepEqual(cloud.events, ["listen", `sub:${topic}`, "unlisten", "cloud-close"]);
});

test("cloud list before LAN ACK is observed, not discarded", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => { queueMicrotask(() => cloud.emit(topic, { id: 41, result: list }));
    queueMicrotask(() => local.resolve?.(["ok"])); };
  const outcome = await run(); assert.equal(outcome.outcome, "LAN_ACK + cloud_list");
  assert.equal(outcome.cloudList, true); assert.equal(outcome.lanAck, true); assert.equal(local.sends, 1);
});

test("wrong topic or ID, invalid fake CRC/decryption, unrelated ACK remain unbound", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => queueMicrotask(() => {
    cloud.emit("wrong/topic", { id: 41, result: list });
    cloud.emit(topic, { id: 42, result: list });
    cloud.emit(topic, { id: 43, result: ["ok"] });
    cloud.emit(topic, { id: 41, result: list, valid: false });
  });
  assert.equal((await run()).outcome, "cloud_unbound"); assert.equal(local.sends, 1);
});

test("bound cloud ACK then list does not terminate on ACK", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => { local.resolve?.(["ok"]); queueMicrotask(() => {
    cloud.emit(topic, { id: 41, result: ["ok"] }); cloud.emit(topic, { id: 41, result: list });
  }); };
  const outcome = await run(); assert.equal(outcome.outcome, "LAN_ACK + cloud_list");
  assert.equal(outcome.cloudAck, true);
});

test("LAN list, duplicate/oversize flags, malformed correlated payload, and RPC rejection", async () => {
  { const { local, run } = fixture(); local.onSend = () => local.resolve?.(list);
    assert.equal((await run()).outcome, "LAN_list"); }
  for (const bad of [{ map_info: { map: [{ mapFlag: 1 }, { mapFlag: 1 }] } },
    { map_info: { map: Array.from({ length: 65 }, (_, mapFlag) => ({ mapFlag })) } }]) {
    const { cloud, local, run } = fixture(); local.onSend = () => queueMicrotask(() => cloud.emit(topic, { id: 41, result: bad }));
    assert.equal((await run()).outcome, "correlated_malformed");
  }
  { const { cloud, local, run } = fixture(); local.onSend = () => queueMicrotask(() => cloud.emit(topic, { id: 41, rejected: true }));
    assert.equal((await run()).outcome, "RPC_rejected"); }
});

test("setup failed, ACK-only timeout, no response, disconnect and frame flood never replay", async () => {
  { const { cloud, local, run } = fixture(); cloud.allowSub = false;
    assert.equal((await run()).outcome, "setup_failed"); assert.equal(local.sends, 0); }
  { const { local, run } = fixture(); local.onSend = () => local.resolve?.(["ok"]);
    assert.equal((await run()).outcome, "ACK_only_timeout"); assert.equal(local.sends, 1); }
  { const { local, run } = fixture(); assert.equal((await run()).outcome, "no_correlated_response");
    assert.equal(local.sends, 1); }
  { const { cloud, local, run } = fixture(); local.onSend = () => cloud.drop();
    assert.equal((await run()).outcome, "no_correlated_response"); assert.equal(local.sends, 1); }
  { const { cloud, local, run } = fixture(); local.onSend = () => {
    queueMicrotask(() => { for (let n = 0; n < 129; n++) cloud.emit(topic, { id: n + 90, result: ["ok"] }); });
  }; assert.equal((await run()).outcome, "correlated_malformed"); assert.equal(local.sends, 1); }
});

test("pre-send and synchronous send callbacks have no authority even with the future request ID", async () => {
  const { cloud, local, run } = fixture();
  const originalSubscribe = cloud.subscribe.bind(cloud);
  cloud.subscribe = (selectedTopic) => {
    cloud.emit(selectedTopic, { id: 41, result: list }); // buffered before the write
    return originalSubscribe(selectedTopic);
  };
  local.onSend = () => {
    cloud.emit(topic, { id: 41, result: list }); // callback races synchronous ID reservation/write
    local.resolve?.(["ok"]);
  };
  assert.deepEqual(await run(), { outcome: "ACK_only_timeout", lanAck: true, lanList: false,
    cloudAck: false, cloudList: false });
  assert.equal(local.sends, 1);
});

test("post-send cloud list without LAN ACK is never labeled LAN_ACK", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => queueMicrotask(() => cloud.emit(topic, { id: 41, result: list }));
  assert.deepEqual(await run(), { outcome: "cloud_list_without_LAN_ACK", lanAck: false,
    lanList: false, cloudAck: false, cloudList: true, itemCount: 1 });
});

test("fake transport cannot promote accessor, non-JSON or over-budget lists", async () => {
  const getterList = { ...list, get trap() { throw new Error("fake getter ran"); } };
  const hugeList = { ...list, extra: Array.from({ length: 65 }, () => Array(65).fill(0)) };
  for (const candidate of [getterList, { ...list, extra: undefined },
    { ...list, extra: Number.NaN }, { ...list, extra: Array(70).fill("x".repeat(1000)) }, hugeList]) {
    const { local, run } = fixture(); local.onSend = () => local.resolve?.(candidate);
    assert.equal((await run()).outcome, "correlated_malformed");
  }
  const cloud = new FakeCloud(), local = new FakeLocal();
  local.onSend = () => queueMicrotask(() => cloud.emit(topic, { id: 41 }));
  const invalid = await researchSplitMapList({ cloud, local, topic, windowMs: 15, setupMs: 25,
    inspect: () => ({ kind: "bound", reply: { id: 41, result: getterList } }) });
  assert.equal(invalid.outcome, "correlated_malformed");
  assert.equal(local.sends, 1);
});

test("cleanup attempts both peers despite throwing fake hooks", async () => {
  const { cloud, local, run } = fixture();
  cloud.listen = () => () => { cloud.events.push("unlisten-throws"); throw new Error("fake unsubscribe"); };
  local.close = () => { local.events.push("local-close-throws"); throw new Error("fake local close"); };
  local.onSend = () => local.resolve?.(list);
  assert.equal((await run()).outcome, "LAN_list");
  assert.ok(local.events.includes("local-close-throws"));
  assert.ok(cloud.events.includes("cloud-close"));
});

test("immediate LAN list terminates without consuming the observation window", async () => {
  const { local, run } = fixture();
  local.onSend = () => local.resolve?.(list);
  const started = Date.now();
  assert.equal((await run(1000)).outcome, "LAN_list");
  assert.ok(Date.now() - started < 500, "LAN list should not wait a full second");
});

test("LAN promise rejection and explicitly rejected result never become a list", async () => {
  { const { local, run } = fixture(); local.onSend = () => local.reject?.(new Error("RPC rejected"));
    assert.equal((await run()).outcome, "RPC_rejected"); }
  { const { local, run } = fixture(); local.onSend = () => local.resolve?.({ ...list, rejected: true });
    assert.equal((await run()).outcome, "RPC_rejected"); }
});

test("disconnect invalidates prior cloud and late LAN evidence, even if a list was bound", async () => {
  const { cloud, local, run } = fixture();
  local.onSend = () => queueMicrotask(() => {
    cloud.emit(topic, { id: 41, result: list });
    cloud.drop();
    local.resolve?.(["ok"]);
    cloud.emit(topic, { id: 41, rejected: true });
  });
  assert.deepEqual(await run(), { outcome: "no_correlated_response", lanAck: false,
    lanList: false, cloudAck: false, cloudList: false });
  assert.equal(local.sends, 1);
});
