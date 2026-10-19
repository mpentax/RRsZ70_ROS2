import { test } from "node:test";
import assert from "node:assert/strict";
import { summarizeStatus } from "./credentials.ts";

test("status summary permits only selected scalar fields", () => {
  assert.deepEqual(summarizeStatus([{ battery: 90, state: 8, localKey: "secret", fw_ver: {} }]), {
    battery: 90, state: 8,
  });
  assert.deepEqual(summarizeStatus(null), {});
  assert.deepEqual(summarizeStatus([]), {});
});
