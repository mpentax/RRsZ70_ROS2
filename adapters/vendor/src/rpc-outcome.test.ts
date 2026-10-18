import assert from "node:assert/strict";
import { test } from "node:test";
import { publicRpcValue } from "./rpc-outcome.ts";
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
