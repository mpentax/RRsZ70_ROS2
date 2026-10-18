import { test } from "node:test";
import assert from "node:assert/strict";
import { numericTelemetry } from "./telemetry.ts";

test("numeric telemetry removes secrets and strings with bounded nesting", () => {
  assert.deepEqual(numericTelemetry({
    m: [1, 2, 3], angle: 4.5, gripper: 2, token: 123, localKey: 42, text: "secret", nested: { ready: true },
  }), { m: [1, 2, 3], angle: 4.5, gripper: 2, nested: { ready: true } });
  assert.equal(numericTelemetry(Infinity), null);
});

test("transport sanitization bounds nesting and array size without inventing numbers", () => {
  assert.equal((numericTelemetry(Array.from({length:100},()=>true)) as unknown[]).length,64);
  assert.deepEqual(numericTelemetry([NaN,"ok",{},false]),[null,null,{},false]);
  assert.equal(numericTelemetry({value:1},6),null);
});
