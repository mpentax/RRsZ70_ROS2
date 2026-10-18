import assert from "node:assert/strict";
import { test } from "node:test";
import { actionOutcome, outcomeMessage } from "../src/outcomes.ts";
import { ApiError, json } from "../src/api.ts";

test("legacy successful HTTP/state-only responses never imply robot acknowledgment", () => {
  for (const value of [{}, { stopped: true }, { acknowledged: false }, { state: { phase: "ready", active: false } }]) {
    assert.equal(actionOutcome(value), "unknown");
  }
  assert.equal(actionOutcome({ acknowledged: true }), "acknowledged");
  assert.equal(actionOutcome({ state: { busy: true } }), "in-progress");
  for (const kind of ["accepted", "acknowledged", "in-progress", "rejected", "failed", "unknown"] as const) {
    assert.equal(actionOutcome({ actionOutcome: { outcome: kind } }), kind);
    assert.doesNotMatch(outcomeMessage(kind), /physically verified|completed successfully/);
  }
});

test("HTTP errors distinguish explicit rejection from legacy ambiguous errors without retries", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => Response.json({ error: "Busy" }, { status: 409 }));
  await assert.rejects(json("/api/arm/action", { body: { action: "deploy" } }), (error: unknown) => {
    assert.ok(error instanceof ApiError); assert.equal(error.outcome, "unknown");
    assert.match(error.message, /may have executed/); return true;
  });
  fetch.mock.mockImplementation(async () => Response.json({ error: "Not allowed", actionOutcome: { outcome: "rejected" } }, { status: 403 }));
  await assert.rejects(json("/api/arm/action", { body: {} }), (error: unknown) => {
    assert.ok(error instanceof ApiError); assert.equal(error.outcome, "rejected"); return true;
  });
  assert.equal(fetch.mock.callCount(), 2);
});

test("network loss, malformed responses and post-dispatch aborts are ambiguous", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => { throw new TypeError("Network lost"); });
  await assert.rejects(json("/api/chassis/input", { body: {} }), /may have executed/);
  fetch.mock.mockImplementation(async () => new Response("{", { headers: { "Content-Type": "application/json" } }));
  await assert.rejects(json("/api/arm/action", { body: {} }), /may have executed/);
  fetch.mock.mockImplementation((_path, options) => new Promise((_resolve, reject) => {
    options?.signal?.addEventListener("abort", () => reject(new DOMException("Aborted", "AbortError")));
  }));
  const controller = new AbortController();
  const request = json("/api/arm/action", { body: {}, signal: controller.signal });
  controller.abort();
  await assert.rejects(request, /may have executed/);
  assert.equal(fetch.mock.callCount(), 3);
});

test("pre-dispatch abort does not send an actuation request", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => { assert.fail("must not send"); });
  const controller = new AbortController(); controller.abort();
  await assert.rejects(json("/api/arm/action", { body: {}, signal: controller.signal }), /before dispatch/);
  assert.equal(fetch.mock.callCount(), 0);
});
