import test from "node:test";
import assert from "node:assert/strict";
import { assessActiveMapEvidence, parseMultiMapsFlags, type CorrelatedMapInputs } from "./active-map-evidence.ts";

const input = (): CorrelatedMapInputs => ({
  selectedDeviceId: "synthetic-device",
  status: { deviceId: "synthetic-device", requestId: "status-1", observedAt: 100, response: [{ map_status: 13, is_locating: 0, pose: { x: 555 } }] },
  maps: { deviceId: "synthetic-device", requestId: "maps-2", observedAt: 101, response: [{ map_info: { map: [{ mapFlag: 3, name: "private-name" }, { mapFlag: 4 }] } }] },
  header: { deviceId: "synthetic-device", mapIndex: 3, mapSequence: 9, integrityVerified: true },
});

test("#11957/#11958 accept only bounded map_info.map flags, never names or extra fields", () => {
  assert.deepEqual(parseMultiMapsFlags(input().maps?.response), [3, 4]);
  assert.deepEqual(parseMultiMapsFlags({ map_info: { map: [] } }), []);
  for (const response of [null, [{ map_info: { map: [{ mapFlag: 1 }] } }, {}],
    { map: [{ mapFlag: 3 }] }, { map_info: { map: [{ id: 3 }] } },
    { map_info: { map: [{ mapFlag: "3" }] } }, { map_info: { map: [{ mapFlag: 3 }, { mapFlag: 3 }] } },
    { map_info: { map: [{ mapFlag: -1 }] } }, { map_info: { map: [{ mapFlag: 2 ** 32 }] } },
    { result: [{ map_info: { map: [{ mapFlag: 3 }] } }] }]) {
    assert.equal(parseMultiMapsFlags(response), null);
  }
});

test("same-device matching status/list/header numbers are diagnostic, never binding or pose", () => {
  const result = assessActiveMapEvidence(input());
  assert.deepEqual(result, {
    verdict: "UNKNOWN", reason: "namespace-unproven", statusCandidate: 3,
    listedMapFlags: [3, 4], pluginListMatch: true, v1MapIndex: 3,
    numericHeaderMatch: true, activeMapVerified: false, currentPoseVerified: false,
  });
  assert.equal(JSON.stringify(result).includes("private-name"), false);
  assert.equal(JSON.stringify(result).includes("555"), false);
  assert.deepEqual(assessActiveMapEvidence({ ...input(), header: { ...input().header!, mapIndex: 4 } }).numericHeaderMatch, false);
  assert.equal(assessActiveMapEvidence({ ...input(), maps: { ...input().maps!, response: { map_info: { map: [{ mapFlag: 4 }] } } } }).pluginListMatch, false);
});

test("missing correlation, device mismatch, failed integrity and reused request IDs fail closed", () => {
  assert.equal(assessActiveMapEvidence({ ...input(), maps: null }).reason, "uncorrelated");
  assert.equal(assessActiveMapEvidence({ ...input(), maps: { ...input().maps!, deviceId: "other" } }).reason, "uncorrelated");
  assert.equal(assessActiveMapEvidence({ ...input(), status: { ...input().status!, requestId: "maps-2" } }).reason, "uncorrelated");
  assert.equal(assessActiveMapEvidence({ ...input(), status: { ...input().status!, observedAt: NaN } }).reason, "uncorrelated");
  assert.equal(assessActiveMapEvidence({ ...input(), header: { ...input().header!, integrityVerified: false } }).v1MapIndex, null);
  assert.equal(assessActiveMapEvidence({ ...input(), header: { ...input().header!, deviceId: "other" } }).numericHeaderMatch, null);
});

test("malformed, locating and sentinel observations preserve UNKNOWN", () => {
  assert.equal(assessActiveMapEvidence({ ...input(), status: { ...input().status!, response: {} } }).reason, "invalid-status");
  assert.equal(assessActiveMapEvidence({ ...input(), maps: { ...input().maps!, response: ["ok"] } }).reason, "invalid-list");
  for (const map_status of [13, 141, -1]) {
    const result = assessActiveMapEvidence({ ...input(), status: { ...input().status!, response: [{ map_status, is_locating: 1 }] } });
    assert.equal(result.reason, "no-candidate");
    assert.equal(result.activeMapVerified, false);
  }
  assert.equal(assessActiveMapEvidence({ ...input(), maps: { ...input().maps!, response: { map_info: { map: [] } } } }).reason, "ambiguous-list");
});
