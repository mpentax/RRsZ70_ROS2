import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { registerHooks } from "node:module";
import { test } from "node:test";
import { Window, type HTMLInputElement } from "happy-dom";
import { act, createElement } from "react";
import { createRoot } from "react-dom/client";
import ts from "typescript";
import { actionOutcome, ApiError, deleteSession, initialSession, json, listGoals, outcomeMessage, robotRpc, SESSION_EXPIRED, snapshot, validateGoal } from "../src/api.ts";

registerHooks({
  load(url, context, next) {
    if (url.endsWith(".tsx")) return { format: "module", shortCircuit: true, source: ts.transpileModule(readFileSync(new URL(url), "utf8"), {
      fileName: new URL(url).pathname, compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2022 } }).outputText };
    return next(url, context);
  },
});

const { App, TaskList } = await import("../src/App.tsx");
const { SetupForm } = await import("../src/SetupForm.tsx");

function dom(t?: { after(fn: () => unknown): void }, url = "http://127.0.0.1:57000") {
  const window = new Window({ url });
  Object.assign(globalThis, { window, document: window.document, HTMLElement: window.HTMLElement, IS_REACT_ACT_ENVIRONMENT: true });
  const element = window.document.createElement("div");
  window.document.body.append(element);
  const root = createRoot(element as unknown as Element);
  const button = (text: string) => Array.from(element.querySelectorAll("button")).find(b => b.textContent === text);
  t?.after(async () => { await act(async () => root.unmount()); await window.happyDOM.close(); });
  return { window, element, root, button };
}

async function flush(ms = 10) {
  await act(async () => { await new Promise(resolve => setTimeout(resolve, ms)); });
}

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

test("actual task rows expose shared ID, numeric phase/result and cancel same ID", async t => {
  const { element, root } = dom(t); const id = "b9e0e188-122b-449d-966b-f97b2bb63981"; const ids: string[] = [];
  const task = { id, kind: "pick", status: "executing", phase: 3, feedback: { reason: "OBSERVING" }, result: null };
  await act(async () => root.render(createElement(TaskList, { tasks: [task], cancel: (id: string) => ids.push(id) })));
  assert.match(element.textContent!, new RegExp(id)); assert.match(element.textContent!, /phase 3/);
  await act(async () => element.querySelector("button")!.click()); assert.deepEqual(ids, [id]);
  await act(async () => root.render(createElement(TaskList, { tasks: [{ ...task, status: "cancelled", phase: 5, result: { outcome: { physical_completion: 0 } } }], cancel: (id: string) => ids.push(id) })));
  assert.equal(element.querySelector("button")!.disabled, true); assert.match(element.textContent!, /physical_completion/);
});

test("stop remains actionable while task start response stalls; expiry clears setup secrets", async t => {
  const { element, root, window, button } = dom(t); const paths: string[] = []; let release: ((r: Response) => void) | undefined;
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    paths.push(path);
    if (path === "/api/ros/tasks" && init?.method === "POST") return new Promise<Response>(resolve => { release = resolve; });
    if (path === "/api/ros/tasks") return Response.json({ tasks: [] });
    if (path === "/api/setup") return Response.json({ device: null, accountConfigured: false, pinConfigured: false });
    if (path === "/api/ros/stop") return Response.json({ cessation_observed: false, outcomes: [] });
    return Response.json({ available: false, state: null, camera: null, arm: null, calibration: null });
  });
  await act(async () => root.render(createElement(App, { session: Promise.resolve({ authenticated: true }) }))); await flush();
  await act(async () => button("Start Pick")!.form!.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  assert.ok(release); const stop = button("STOP ALL")!;
  assert.equal(stop.disabled, false); await act(async () => stop.click()); await flush(); assert.ok(paths.includes("/api/ros/stop"));
  await act(async () => window.dispatchEvent(new window.Event(SESSION_EXPIRED))); await flush();
  assert.equal(element.querySelector("h1")?.textContent, "RoboClaw"); assert.equal(element.querySelectorAll("input").length, 1);
  assert.match(element.textContent!, /Camera PIN/);
  assert.equal(element.querySelector("input")!.value, "");
  release(Response.json({ id: "b9e0e188-122b-449d-966b-f97b2bb63981", status: "accepted" })); await flush();
});

test("navigation and garment patrol buttons dispatch pin_and_go, deliver_garment and patrol_collect tasks", async t => {
  const { root, button } = dom(t); const started: unknown[] = [];
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    if (path === "/api/ros/tasks" && init?.method === "POST") {
      const body = JSON.parse(String(init.body)); started.push(body);
      return Response.json({ id: "b9e0e188-122b-449d-966b-f97b2bb63981", kind: body.kind, status: "accepted", phase: 1, feedback: null, result: null });
    }
    if (path === "/api/ros/tasks") return Response.json({ tasks: [] });
    if (path === "/api/setup") return Response.json({ device: null, accountConfigured: false, pinConfigured: false });
    return Response.json({ available: true, state: { connected: true, battery_known: true, battery_fraction: 0.9, evidence: { valid: true, stale: false } }, camera: null, arm: null, calibration: null });
  });
  await act(async () => root.render(createElement(App, { session: Promise.resolve({ authenticated: true }) }))); await flush();
  await act(async () => button("Pin & Go")!.click()); await flush();
  await act(async () => button("Deliver Garment")!.click()); await flush();
  await act(async () => button("Patrol & Collect")!.click()); await flush();
  assert.deepEqual(started, [
    { kind: "pin_and_go", goal: { target_x_m: 0, target_y_m: 0, max_duration_s: 120, dry_run: true } },
    { kind: "deliver_garment", goal: { category: "any", drop_x_m: 0, drop_y_m: 0, max_duration_s: 300, allow_towing: false, dry_run: true } },
    { kind: "patrol_collect", goal: { drop_x_m: 0, drop_y_m: 0, max_duration_s: 600, max_items: 5, allow_towing: false, dry_run: true } },
  ]);
});

test("setup renders required web inputs and bounded expired-login recovery", async t => {
  const { element, root, window } = dom(t); const submitted: unknown[] = []; let succeedLogin = false;
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    submitted.push({ path, body: JSON.parse(String(init.body)) });
    if (path === "/api/account/code") return Response.json({ sent: true });
    if (path === "/api/account/login" && succeedLogin) return Response.json({ devices: [{ id: "dev1", name: "Saros Z70", model: "roborock.vacuum.a179" }] });
    return Response.json({ error: "Account login expired. Request a new code in web setup." }, { status: 410 });
  });
  await act(async () => root.render(createElement(SetupForm, { setup: null, onUpdated: async () => {} })));
  assert.ok(element.querySelector("input[type=email]")); assert.ok(element.querySelector("input[type=password]"));
  const form = element.querySelector("form")!;
  await act(async () => form.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  const code = element.querySelector('input[pattern="[0-9]{6}"]')!; assert.ok(code);
  await act(async () => code.closest("form")!.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  assert.match(element.textContent!, /Account login expired/); assert.equal(element.querySelector('input[pattern="[0-9]{6}"]'), null);
  assert.equal((submitted[1] as { body: { code: string } }).body.code, "");
  succeedLogin = true;
  await act(async () => form.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  const code2 = element.querySelector('input[pattern="[0-9]{6}"]')!;
  await act(async () => code2.closest("form")!.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  assert.match(element.textContent!, /Settings > Product Info > Wi-Fi Name/i);
});

test("rendered web reconnects read-only and clears only connection errors", async t => {
  const { element, root } = dom(t);
  let available = false; const writes: string[] = [];
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    if (init.method === "POST") writes.push(path);
    if (path === "/api/ros/tasks") return Response.json({ tasks: [] });
    if (path === "/api/setup") return Response.json({ device: null, accountConfigured: false, pinConfigured: false });
    if (!available) return Response.json({ error: "ROS_UNAVAILABLE", dispatch: "not_sent" }, { status: 503 });
    return Response.json({ available: true, state: { connected: true, battery_known: true, battery_fraction: 0.75, evidence: { stale: false } } });
  });
  await act(async () => { root.render(createElement(App, { session: Promise.resolve({ authenticated: true }) })); });
  await flush(30);
  assert.match(element.textContent!, /ROS_UNAVAILABLE/);
  available = true;
  await flush(1600);
  assert.match(element.textContent!, /ROS driver available/); assert.match(element.textContent!, /75%/);
  assert.doesNotMatch(element.textContent!, /ROS_UNAVAILABLE/); assert.deepEqual(writes, []);
});

test("rendered session stop remains usable during a stalled command and requires explicit resume", async t => {
  const { root, button } = dom(t);
  const writes: { path: string; body: unknown }[] = []; let enabled = true; let finish!: () => void;
  const pending = new Promise<void>(r => { finish = r; });
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    if (init.method === "POST") writes.push({ path, body: JSON.parse(init.body as string) });
    if (path === "/api/ros/control/session") {
      enabled = JSON.parse(init.body as string).enabled;
      return Response.json({ enabled, streams_released: !enabled, cessation_observed: false, outcomes: [] });
    }
    if (path === "/api/ros/tasks" && init.method === "POST") { await pending; return Response.json({ id: "fake" }); }
    if (path === "/api/ros/tasks") return Response.json({ tasks: [] });
    if (path === "/api/setup") return Response.json({ device: null, accountConfigured: false, pinConfigured: false });
    return Response.json({ available: true, session_enabled: enabled });
  });
  try {
    await act(async () => { root.render(createElement(App, { session: Promise.resolve({ authenticated: true }) })); });
    await act(async () => { button("M3 + nominal step")!.click(); });
    const stop = button("Stop control & streams"); assert.ok(stop, "dedicated session stop must exist");
    assert.equal(stop.disabled, false);
    await act(async () => { stop.click(); });
    assert.deepEqual(writes.at(-1), { path: "/api/ros/control/session", body: { enabled: false } });
    assert.equal(button("M3 + nominal step")!.disabled, true);
    assert.ok(button("Resume control & streams"));
    await flush(1600);
    assert.equal(writes.filter(w => w.path === "/api/ros/control/session").length, 1, "polling must not resume");
    await act(async () => { button("Resume control & streams")!.click(); });
    assert.deepEqual(writes.at(-1), { path: "/api/ros/control/session", body: { enabled: true } });
  } finally { finish(); await act(async () => { await pending; }); }
});

test("first-run browser offers setup without ROS and returns to PIN sign-in on completion", async t => {
  const { window, element, root, button } = dom(t); const paths: string[] = []; let completed = false;
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    paths.push(path);
    if (path === "/api/session" && init.method === "POST") return Response.json({ authenticated: false, scope: "setup", setupAvailable: true });
    if (path === "/api/session") return Response.json({ authenticated: false, scope: completed ? null : "setup", setupAvailable: !completed });
    if (path === "/api/setup") return Response.json({ device: { id: "robot1", name: "Test Z70", model: "a179", host: "127.0.0.1", localProtocol: "L01" }, accountConfigured: true, pinConfigured: false });
    if (path === "/api/camera/pin") { completed = true; return Response.json({ pinCached: true }); }
    return Response.json({ error: "ROS_UNAVAILABLE" }, { status: 503 });
  });
  await act(async () => root.render(createElement(App, { session: Promise.resolve({ authenticated: false, scope: null, setupAvailable: true }) }))); await flush();
  const start = button("Start local setup"); assert.ok(start);
  await act(async () => start.click()); await flush();
  assert.match(element.textContent!, /Initial local setup/); assert.match(element.textContent!, /Test Z70/);
  assert.ok(element.querySelector("input[type=email]")); assert.doesNotMatch(element.textContent!, /Start Pick|STOP ALL|ROS_UNAVAILABLE/);
  assert.equal(paths.some(p => p.startsWith("/api/ros/")), false);
  const pin = element.querySelector('input[placeholder="4–12 digits (1–9)"]')!;
  await act(async () => pin.closest("form")!.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  assert.match(element.textContent!, /Setup complete. Sign in with your camera PIN/);
  assert.equal(element.querySelector("input[type=email]"), null); assert.equal(element.querySelectorAll("input").length, 1);
  assert.equal(element.querySelector("input")!.value, "");
  assert.equal(paths.some(p => p.startsWith("/api/ros/")), false);
});

test("stale first-run setup conflict refreshes session and offers usable PIN sign-in", async t => {
  const { window, element, root, button } = dom(t); const requests: string[] = [];
  t.mock.method(globalThis, "fetch", async (path: string, init: RequestInit) => {
    requests.push(`${init.method} ${path}`);
    if (path === "/api/session" && init.method === "POST") {
      const body = JSON.parse(init.body as string);
      if (body.setup) { assert.deepEqual(body, { setup: true }); return Response.json({}, { status: 409 }); }
      assert.deepEqual(body, { pin: "123456" });
      return Response.json({ authenticated: true, scope: "control", setupAvailable: false });
    }
    if (path === "/api/session") return Response.json({ authenticated: false, scope: null, setupAvailable: false });
    if (path === "/api/setup") return Response.json({ device: null, accountConfigured: true, pinConfigured: true });
    return Response.json({ error: "ROS_UNAVAILABLE" }, { status: 503 });
  });
  await act(async () => root.render(createElement(App, { session: Promise.resolve({ authenticated: false, scope: null, setupAvailable: true }) })));
  await act(async () => button("Start local setup")!.click()); await flush();
  assert.deepEqual(requests, ["POST /api/session", "GET /api/session"]);
  assert.equal(Boolean(button("Start local setup")), false);
  const pin = element.querySelector<HTMLInputElement>("input[type=password]"); assert.ok(pin);
  const props = Object.entries(pin).find(([key]) => key.startsWith("__reactProps$"))![1];
  await act(async () => { pin.value = "123456"; props.onChange({ target: pin }); });
  const signIn = button("Sign in")!;
  assert.equal(signIn.disabled, false);
  await act(async () => pin.closest("form")!.dispatchEvent(new window.Event("submit", { bubbles: true, cancelable: true }))); await flush();
  assert.equal(requests.filter(r => r === "POST /api/session").length, 2);
  assert.match(element.textContent!, /ROS unavailable/);
  assert.ok(button("Sign out"));
});

test("expired first-run setup clears secret inputs and offers setup again without a reload", async t => {
  const { window, element, root, button } = dom(t);
  t.mock.method(globalThis, "fetch", async (path: string) => Response.json(path === "/api/session"
    ? { authenticated: false, scope: null, setupAvailable: true }
    : { device: null, accountConfigured: false, pinConfigured: false }));
  await act(async () => root.render(createElement(App, { session: Promise.resolve({ authenticated: false, scope: "setup" as const, setupAvailable: true }) })));
  assert.ok(element.querySelector("input[type=email]"));
  await act(async () => window.dispatchEvent(new window.Event(SESSION_EXPIRED)));
  await flush();
  assert.equal(element.querySelector("input[type=email]"), null);
  assert.ok(button("Start local setup"));
});
