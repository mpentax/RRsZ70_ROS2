import assert from "node:assert/strict";
import { test } from "node:test";
import { actionOutcome, ApiError, deleteSession, initialSession, json, listGoals, outcomeMessage, robotRpc, snapshot, validateGoal } from "../src/api.ts";

test("JSON requests use same-origin credentials and POST JSON without CSRF or bearer headers", async (context) => {
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/status");
    assert.equal(options.credentials, "same-origin");
    assert.equal(options.method, "POST");
    assert.equal(options.body, "{}");
    assert.deepEqual(options.headers, { "Content-Type": "application/json" });
    assert.equal(options.cache, "no-store");
    assert.equal(options.redirect, "error");
    return Response.json({ ok: true });
  });
  assert.deepEqual(await json("/api/status", { body: {} }), { ok: true });
});

test("session sign-in sends a camera PIN in JSON only", async (context) => {
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/session");
    assert.equal(options.method, "POST");
    assert.equal(options.body, JSON.stringify({ pin: "123456" }));
    return Response.json({ authenticated: true });
  });
  assert.deepEqual(await initialSession("123456"), { authenticated: true });
});

test("existing session discovery is a credentialed GET without a PIN", async (context) => {
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/session");
    assert.equal(options.method, "GET");
    assert.equal(options.body, undefined);
    assert.equal(options.credentials, "same-origin");
    return Response.json({ authenticated: false });
  });
  assert.deepEqual(await initialSession(null), { authenticated: false });
});

test("one shared initialSession promise supports multiple subscribers without repeating requests", async (context) => {
  let respond: ((response: Response) => void) | undefined;
  const response = new Promise<Response>((resolve) => { respond = resolve; });
  const fetch = context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/session");
    assert.equal(options.method, "GET");
    return response;
  });
  const session = initialSession();
  const firstSubscriber = session.then((result) => result.authenticated);
  const secondSubscriber = session.then((result) => result.authenticated);
  assert.ok(respond);
  respond(Response.json({ authenticated: true }));
  assert.deepEqual(await Promise.all([firstSubscriber, secondSubscriber]), [true, true]);
  assert.equal((await session).authenticated, true);
  assert.equal(fetch.mock.callCount(), 1);
});

test("authenticated reload discovers the cookie session with a GET", async (context) => {
  const fetch = context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/session");
    assert.equal(options.method, "GET");
    assert.equal(options.credentials, "same-origin");
    assert.equal(options.body, undefined);
    return Response.json({ authenticated: true });
  });
  assert.deepEqual(await initialSession(), { authenticated: true });
  assert.equal(fetch.mock.callCount(), 1);
});

test("session errors cannot echo a camera PIN from a server response", async (context) => {
  context.mock.method(globalThis, "fetch", async () =>
    Response.json({ error: "Invalid PIN: 123456" }, { status: 401 }));
  await assert.rejects(initialSession("123456"), (error: unknown) => {
    assert.ok(error instanceof ApiError);
    assert.equal(error.status, 401);
    assert.doesNotMatch(error.message, /123456/);
    assert.match(error.message, /Check your camera PIN/);
    return true;
  });
});

test("setup session errors point to localhost or configured PIN without echoing server data", async context => {
  context.mock.method(globalThis, "fetch", async () =>
    Response.json({ error: "upstream-private-token 123456" }, { status: 403 }));
  await assert.rejects(json('/api/session', {body:{setup:true}}), (error:unknown) => {
    assert.ok(error instanceof ApiError);
    assert.match(error.message, /localhost/);
    assert.doesNotMatch(error.message, /upstream-private-token|123456/);
    assert.equal(error.evidence, undefined);
    return true;
  });
});

test("logout accepts an empty successful response", async (context) => {
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/session");
    assert.equal(options.method, "DELETE");
    assert.equal(options.credentials, "same-origin");
    return new Response(null, { status: 204 });
  });
  await deleteSession();
});

test("snapshots accept JPEG bytes and use an independent empty POST", async (context) => {
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    assert.equal(path, "/api/ros/frame");
    assert.equal(options.method, "POST");
    assert.equal(options.body, "{}");
    assert.equal(options.credentials, "same-origin");
    return new Response(new Uint8Array([255, 216, 255, 217]), { headers: { "Content-Type": "image/jpeg" } });
  });
  const result = await snapshot(new AbortController().signal);
  assert.equal(result.type, "image/jpeg");
  assert.equal(result.size, 4);
});

test("snapshot HTTP failures and JSON errors never masquerade as image blobs", async (context) => {
  const fetch = context.mock.method(globalThis, "fetch", async () =>
    Response.json({ error: "Camera unavailable" }, { status: 503 }));
  await assert.rejects(snapshot(new AbortController().signal), /Camera unavailable/);
  fetch.mock.mockImplementation(async () => Response.json({ error: "No current frame" }));
  await assert.rejects(snapshot(new AbortController().signal), /No current frame/);
  fetch.mock.mockImplementation(async () => new Response("not an image", { headers: { "Content-Type": "text/html" } }));
  await assert.rejects(snapshot(new AbortController().signal), /unexpected image format/);
  fetch.mock.mockImplementation(async () => new Response(new Blob([], { type: "image/jpeg" })));
  await assert.rejects(snapshot(new AbortController().signal), /empty image/);
});

test("JSON endpoints reject accidental HTML app fallback", async (context) => {
  context.mock.method(globalThis, "fetch", async () =>
    new Response("<html>app</html>", { headers: { "Content-Type": "text/html" } }));
  await assert.rejects(json("/api/state"), /instead of JSON/);
});

test("caller cancellation and timeout both abort the underlying fetch", async (context) => {
  context.mock.method(globalThis, "fetch", (_path: string, options: RequestInit) =>
    new Promise((_resolve, reject) => {
      options.signal?.addEventListener("abort", () => reject(new DOMException("Aborted", "AbortError")), { once: true });
    }));
  const controller = new AbortController();
  const pending = json("/api/state", { signal: controller.signal });
  controller.abort();
  await assert.rejects(pending, { name: "AbortError" });
  await assert.rejects(json("/api/ros/tasks", { body: { action: "close" }, action: true, timeout: 5 }),
    /Outcome unknown\./);
});

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

test("browser RPC uses canonical route, session cookie and unchanged raw outcome", async context => {
  const input = { method: "custom", params: [1, { x: true }], transport: "local" as const, timeoutMs: 1000 };
  const outcome = { requestId: "1", dispatch: "unknown", response: "transport_error", error: "timeout" };
  const paths: string[] = [];
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    paths.push(path); assert.equal(options.method, "POST"); assert.equal(options.credentials, "same-origin");
    assert.deepEqual(JSON.parse(String(options.body)), input); return Response.json(outcome);
  });
  assert.deepEqual(await robotRpc(input), outcome);
  assert.deepEqual(paths, ["/api/ros/rpc"]);
});

test("listGoals and validateGoal use canonical ROS goal endpoints", async context => {
  const calls: { path: string; method?: string; body?: unknown }[] = [];
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    calls.push({ path, method: options.method, body: options.body ? JSON.parse(String(options.body)) : undefined });
    if (path === "/api/ros/goals") return Response.json({ goals: [{ kind: "pick" }], context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }] });
    return Response.json({ valid: true, kind: "pick", simulated: { ok: true, result: { verified_hold: true } } });
  });
  const catalog = await listGoals();
  assert.equal(catalog.goals[0].kind, "pick");
  assert.equal(catalog.context_primitives[0].summary, "Read robot status.");
  const validated = await validateGoal({ kind: "pick", goal: { color: "yellow", dry_run: true }, simulate: true }) as { valid: boolean };
  assert.equal(validated.valid, true);
  assert.deepEqual(calls.map(c => [c.method, c.path]), [["GET", "/api/ros/goals"], ["POST", "/api/ros/goals/validate"]]);
});
