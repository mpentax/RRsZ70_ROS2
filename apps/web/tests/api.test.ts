import assert from "node:assert/strict";
import { test } from "node:test";
import { ApiError, deleteSession, initialSession, json, snapshot } from "../src/api.ts";

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
  await assert.rejects(json("/api/arm/action", { body: { action: "close" }, timeout: 5 }),
    /Outcome unknown; the request may have executed/);
});
