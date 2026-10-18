import test from "node:test";
import assert from "node:assert/strict";
import { parseMapStatusEvidence } from "./map-status-evidence.ts";

test("Hermes #8421 map_status candidate, bit-7 sentinel and locating gate remain unverified", () => {
  assert.deepEqual(parseMapStatusEvidence([{ map_status: 13, is_locating: 0, pose: { x: 123 } }]), {
    mapStatus: 13, mapStatusLowBits: 1, isLocating: false, pluginMapIdCandidate: 3,
    candidateUsable: true, interpretation: "unverified",
  });
  assert.equal(parseMapStatusEvidence([{ map_status: 141, is_locating: 0 }])?.pluginMapIdCandidate, -1);
  assert.equal(parseMapStatusEvidence([{ map_status: -1, is_locating: 0 }])?.candidateUsable, false);
  assert.equal(parseMapStatusEvidence([{ map_status: 13, is_locating: 1 }])?.pluginMapIdCandidate, -1);
  assert.equal(parseMapStatusEvidence([{ map_status: 13, is_locating: 1 }])?.candidateUsable, false);
  assert.equal(parseMapStatusEvidence([{ map_status: 13 }])?.pluginMapIdCandidate, null);
});

test("malformed, missing and potentially private fields are not exposed", () => {
  assert.equal(parseMapStatusEvidence([{ map_status: "13", is_locating: "0", position: [4, 8] }]), null);
  assert.equal(parseMapStatusEvidence([{ map_status: Infinity, is_locating: 2 }]), null);
  assert.equal(parseMapStatusEvidence([{ map_status: 2 ** 40, is_locating: 0 }])?.mapStatus, null);
  assert.equal(parseMapStatusEvidence([{ map_status: 1 }, { map_status: 2 }]), null);
  assert.equal(parseMapStatusEvidence(null), null);
});
