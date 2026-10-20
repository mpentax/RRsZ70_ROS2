import assert from "node:assert/strict";
import { test } from "node:test";
import { actionOutcome, outcomeMessage } from "../src/outcomes.ts";
import { ApiError, json } from "../src/api.ts";

test("actionOutcome maps explicit outcome envelopes and state flags", () => {
  for (const value of [{}, { stopped: true }, { acknowledged: false }, { state: { phase: "ready", active: false } }]) {
    assert.equal(actionOutcome(value), "unknown");
  }
  assert.equal(actionOutcome({ acknowledged: true }), "acknowledged");
  assert.equal(actionOutcome({ outcome: { dispatch: 1, response: 2 } }), "acknowledged");
  assert.equal(actionOutcome({ outcome: { dispatch: 2, response: 1 } }), "rejected");
  assert.equal(actionOutcome({ outcome: { dispatch: 1, response: 4 } }), "failed");
  assert.equal(actionOutcome({ outcome: { dispatch: 1, response: 1 } }), "accepted");
  assert.equal(actionOutcome({ status: "accepted", kind: "pick" }), "accepted");
  assert.equal(actionOutcome({ status: "executing", kind: "pick" }), "in-progress");
  assert.equal(actionOutcome({ status: "succeeded", kind: "pick" }), "acknowledged");
  assert.equal(actionOutcome({ status: "failed", kind: "pick" }), "failed");
  assert.equal(actionOutcome({ error: "INVALID_INPUT", dispatch: "not_sent" }), "rejected");
  assert.equal(actionOutcome({ state: { busy: true } }), "in-progress");
  for (const kind of ["accepted", "acknowledged", "in-progress", "rejected", "failed", "unknown"] as const) {
    assert.equal(actionOutcome({ actionOutcome: { outcome: kind } }), kind);
    assert.ok(outcomeMessage(kind).length > 0);
  }
});

test("HTTP errors distinguish explicit rejection from ambiguous action errors without retries", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => Response.json({ error: "Busy" }, { status: 409 }));
  await assert.rejects(json("/api/ros/tasks", { body: { action: "deploy" }, action: true }), (error: unknown) => {
    assert.ok(error instanceof ApiError); assert.equal(error.outcome, "unknown");
    assert.match(error.message, /Outcome unknown\./); return true;
  });
  fetch.mock.mockImplementation(async () => Response.json({ error: "Not allowed", actionOutcome: { outcome: "rejected" } }, { status: 403 }));
  await assert.rejects(json("/api/ros/tasks", { body: {}, action: true }), (error: unknown) => {
    assert.ok(error instanceof ApiError); assert.equal(error.outcome, "rejected"); return true;
  });
  assert.equal(fetch.mock.callCount(), 2);
});

test("network loss, malformed responses and post-dispatch aborts are ambiguous", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => { throw new TypeError("Network lost"); });
  await assert.rejects(json("/api/ros/tasks", { body: {}, action: true }), /Outcome unknown\./);
  fetch.mock.mockImplementation(async () => new Response("{", { headers: { "Content-Type": "application/json" } }));
  await assert.rejects(json("/api/ros/tasks", { body: {}, action: true }), /Outcome unknown\./);
  fetch.mock.mockImplementation((_path, options) => new Promise((_resolve, reject) => {
    options?.signal?.addEventListener("abort", () => reject(new DOMException("Aborted", "AbortError")));
  }));
  const controller = new AbortController();
  const request = json("/api/ros/tasks", { body: {}, action: true, signal: controller.signal });
  controller.abort();
  await assert.rejects(request, /Outcome unknown\./);
  assert.equal(fetch.mock.callCount(), 3);
});

test("pre-dispatch abort does not send an actuation request", async (t) => {
  const fetch = t.mock.method(globalThis, "fetch", async () => { assert.fail("must not send"); });
  const controller = new AbortController(); controller.abort();
  await assert.rejects(json("/api/ros/tasks", { body: {}, action: true, signal: controller.signal }), /before dispatch/);
  assert.equal(fetch.mock.callCount(), 0);
});
