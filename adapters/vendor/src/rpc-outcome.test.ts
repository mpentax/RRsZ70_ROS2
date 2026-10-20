import assert from "node:assert/strict";
import { test } from "node:test";
import { numericTelemetry, publicRpcValue, summarizeStatus } from "./rpc-outcome.ts";
test("credential keys and embedded known credential values never survive public vendor results", () => {
  assert.deepEqual(publicRpcValue({ localKey: "hidden", nested: [{ password: "hidden", result: "secret-value" }], ok: true }, ["secret-value"]), { nested: [{ result: "[redacted]" }], ok: true });
});
test("deep vendor payloads are bounded without changing transport evidence", () => {
  let value: unknown = { leaf: true }; for (let i = 0; i < 1000; i++) value = { nested: value };
  const result = JSON.stringify(publicRpcValue(value)); assert.ok(result.length < 1024); assert.ok(result.includes("null"));
});

test("credential substrings in vendor property names are redacted", () => {
  assert.deepEqual(publicRpcValue({ "prefix-1234abcdef-suffix": "ok" }, ["1234abcdef"]), { "prefix-[redacted]-suffix": "ok" });
});

test("numeric telemetry removes secrets and strings with bounded nesting", () => {
  assert.deepEqual(numericTelemetry({
    m: [1, 2, 3], angle: 4.5, gripper: 2, token: 123, localKey: 42, text: "secret", nested: { ready: true },
  }), { m: [1, 2, 3], angle: 4.5, gripper: 2, nested: { ready: true } });
  assert.equal(numericTelemetry(Infinity), null);
});

test("transport sanitization bounds nesting and array size without inventing numbers", () => {
  assert.equal((numericTelemetry(Array.from({ length: 100 }, () => true)) as unknown[]).length, 64);
  assert.deepEqual(numericTelemetry([NaN, "ok", {}, false]), [null, null, {}, false]);
  assert.equal(numericTelemetry({ value: 1 }, 6), null);
});

test("status summary permits only selected scalar fields", () => {
  assert.deepEqual(summarizeStatus([{ battery: 90, state: 8, localKey: "secret", fw_ver: {} }]), {
    battery: 90, state: 8,
  });
  assert.deepEqual(summarizeStatus(null), {});
  assert.deepEqual(summarizeStatus([]), {});
});
