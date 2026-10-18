import assert from "node:assert/strict";
import { test } from "node:test";
import { classifySplitMapList, MAX_LIST_FRAME_BYTES, MAX_LIST_FRAMES } from "./legacy-ts/map-list-split-correlation-research.ts";

const requestId = 7281, incomingTopic = "fixture/selected-device/in";
const secret = { mapFlag: 493012, mapName: "PRIVATE_FLOOR_SENTINEL", geometry: "PRIVATE_GEOMETRY_SENTINEL" };
const list = { map_info: { map: [secret] }, unknown: "PRIVATE_UNKNOWN_SENTINEL" };
const lan = (result: unknown, at = 101) => ({ source: "lan" as const, at, correlated: true,
  reply: { requestId, result } });
const cloud = (result: unknown, at = 102) => ({ source: "cloud" as const, at,
  topic: incomingTopic, protocol: 102, frameBytes: 128, frameValid: true, outerDecrypted: true,
  reply: { requestId, result } });
const assess = (observations: Parameters<typeof classifySplitMapList>[0]["observations"]) =>
  classifySplitMapList({ requestId, incomingTopic, sentAt: 100, setupReady: true, observations });
const safe = (result: ReturnType<typeof assess>) => {
  const printed = JSON.stringify(result);
  for (const value of ["493012", "PRIVATE_FLOOR_SENTINEL", "PRIVATE_GEOMETRY_SENTINEL",
    "PRIVATE_UNKNOWN_SENTINEL", incomingTopic, String(requestId)]) assert.ok(!printed.includes(value));
  assert.equal(result.status, "UNVERIFIED_RESEARCH_ONLY");
};

test("LAN ACK followed by independently bound protocol-102 list; no IDs or private fields emitted", () => {
  const result = assess([lan(["ok"]), cloud(list)]);
  assert.equal(result.outcome, "LAN_ACK + cloud_list");
  assert.deepEqual([result.lanAck, result.lanList, result.cloudAck, result.cloudList, result.itemCount],
    [true, false, false, true, 1]);
  safe(result);
});
test("cloud list before LAN ACK remains bound; LAN typed list is independent evidence", () => {
  assert.equal(assess([cloud(list, 101), lan(["ok"], 102)]).outcome, "LAN_ACK + cloud_list");
  const result = assess([lan(list), cloud(["ok"])]);
  assert.equal(result.outcome, "LAN_list");
  assert.equal(result.cloudAck, true);
  assert.equal(result.itemCount, 1);
  safe(result);
});
test("cloud ACK is not a list, even with a LAN ACK; timeout is not unsupported", () => {
  const result = assess([cloud(["ok"]), lan(["ok"])]);
  assert.equal(result.outcome, "ACK_only_timeout");
  assert.equal(result.cloudAck, true);
  assert.equal(result.cloudList, false);
  assert.equal(result.itemCount, null);
  assert.equal(assess([]).outcome, "no_correlated_response");
});
test("wrong device topic, ID, protocol, CRC/outer decryption, and uncorrelated LAN cannot bind", () => {
  const unrelated = [
    { ...cloud(list), topic: "fixture/other-device/in" },
    { ...cloud(list), reply: { requestId: requestId + 1, result: list } },
    { ...cloud(list), protocol: 301 },
    { ...cloud(list), frameValid: false },
    { ...cloud(list), outerDecrypted: false },
    { ...lan(list), correlated: false },
    { ...lan(list), reply: { requestId: requestId + 1, result: list } },
  ];
  const result = assess(unrelated);
  assert.equal(result.outcome, "cloud_unbound");
  assert.equal(result.ignoredUnbound, unrelated.length);
  assert.equal(result.itemCount, null);
  safe(result);
  assert.equal(assess([cloud(["ok"], 101), unrelated[1], cloud(list, 103)]).outcome, "cloud_list");
});
test("oversized, undersized and invalid-size frames cannot bind; finite observation budget", () => {
  const result = assess([cloud(list, 101), { ...cloud(list, 102), frameBytes: MAX_LIST_FRAME_BYTES + 1 },
    { ...cloud(list, 103), frameBytes: 22 }]);
  assert.equal(result.outcome, "cloud_list");
  assert.equal(result.oversizedFrames, 2);
  assert.equal(assess([{ ...cloud(list), frameBytes: MAX_LIST_FRAME_BYTES + 1 }]).outcome, "cloud_unbound");
  assert.equal(assess(Array(MAX_LIST_FRAMES + 1).fill(cloud(list))).outcome, "setup_failed");
  safe(result);
});
test("duplicate and late replies are ignored; no post-window list upgrades an ACK", () => {
  const result = assess([lan(["ok"]), lan(list, 102), cloud(["ok"], 103), cloud(["ok"], 104),
    cloud(list, 105), cloud(list, 106), cloud(list, 12_101)]);
  assert.equal(result.outcome, "LAN_ACK + cloud_list");
  assert.equal(result.ignoredDuplicate, 3);
  assert.equal(result.ignoredLate, 1);
  assert.equal(result.itemCount, 1);
  assert.equal(assess([lan(["ok"]), cloud(list, 12_101)]).outcome, "ACK_only_timeout");
  safe(result);
});
test("rejections and correlated malformed or differently structured results stay categorical", () => {
  assert.equal(assess([{ ...lan(null), reply: { requestId, errorCode: 42 } }]).outcome, "RPC_rejected");
  assert.equal(assess([lan({ map_info: { map: [{ mapFlag: 4 }, { mapFlag: 4 }] } })]).outcome, "correlated_malformed");
  assert.equal(assess([cloud({ map_info: { map: [{ mapFlag: "4" }] } })]).outcome, "correlated_malformed");
  assert.equal(assess([lan({ unexpected: "PRIVATE_UNKNOWN_SENTINEL" })]).outcome, "correlated_malformed");
  // Conflicting bound replies must not promote a list even if its shape is valid.
  assert.equal(assess([cloud(list), { ...cloud(null, 103), reply: { requestId, errorCode: 42 } }]).outcome,
    "RPC_rejected");
  assert.equal(assess([cloud(list), cloud({ unexpected: true }, 103)]).outcome, "correlated_malformed");
  const cycle: { self?: unknown } = {}; cycle.self = cycle;
  assert.equal(assess([cloud(cycle)]).outcome, "correlated_malformed");
  let nested: unknown = 1;
  for (let i = 0; i < 26; i++) nested = [nested];
  assert.equal(assess([cloud(nested)]).outcome, "correlated_malformed");
  assert.equal(assess([cloud({ map_info: { map: Array.from({ length: 65 }, (_, i) => ({ mapFlag: i })) } })]).outcome,
    "correlated_malformed");
});
test("no setup or invalid request identity cannot be interpreted as a response", () => {
  assert.equal(classifySplitMapList({ requestId, incomingTopic, sentAt: 100, setupReady: false,
    observations: [cloud(list)] }).outcome, "setup_failed");
  assert.equal(classifySplitMapList({ requestId: -1, incomingTopic, sentAt: 100, setupReady: true,
    observations: [cloud(list)] }).outcome, "setup_failed");
  assert.equal(assess([{ ...cloud(list), at: 99 }]).outcome, "no_correlated_response");
  assert.equal(classifySplitMapList({ requestId: 0x80000000, incomingTopic, sentAt: 100,
    setupReady: true, observations: [{ ...cloud(list), reply: { requestId: 0x80000000, result: list } }] }).outcome,
  "setup_failed");
});
