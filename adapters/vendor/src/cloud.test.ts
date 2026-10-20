import assert from "node:assert/strict";
import { createHash, createHmac } from "node:crypto";
import { test, type TestContext } from "node:test";
import { RoborockCloud } from "./cloud.ts";

const EMAIL = "test@example.invalid";
const KEY = "0123456789abcdef";
const login = (endpoint = "https://api-eu.roborock.com") => ({
  code: 200,
  data: { token: "session-token", rriot: { u: "user123", s: "salt123", h: "hawk-key", r: { a: endpoint } } },
});
const cameraLogin = (broker = "ssl://mqtt-eu.roborock.com:8883") => {
  const session = login();
  return { ...session, data: { ...session.data, rriot: { ...session.data.rriot, k: "domain-key", r: { ...session.data.rriot.r, m: broker } } } };
};
const home = () => ({
  success: true,
  result: {
    products: [
      { id: "2", model: "roborock.vacuum.other", name: "Shared model" },
      { id: 1, model: "roborock.vacuum.a147" },
    ],
    devices: [{ duid: "owned", name: "Z70", productId: "1", localKey: KEY, pv: "1.0" }],
    receivedDevices: [{ duid: "shared", productId: "2", localKey: KEY, pv: "1.0" }],
  },
});

interface Call { url: URL; init: RequestInit }

function mockResponses(t: TestContext, values: unknown[]): Call[] {
  const calls: Call[] = [];
  t.mock.method(globalThis, "fetch", async (input: URL, init: RequestInit) => {
    assert.ok(input instanceof URL);
    assert.equal(init.redirect, "error");
    assert.ok(init.signal instanceof AbortSignal);
    assert.equal(init.signal.aborted, false);
    calls.push({ url: input, init });
    assert.ok(values.length > 0, "Unexpected network request");
    const value = values.shift();
    if (value instanceof Error) throw value;
    if (value instanceof Response) return value;
    return Response.json(value);
  });
  return calls;
}

function flow(final: unknown = home(), session: unknown = login()): unknown[] {
  return [{ code: 200, data: { k: "sign-key" } }, session, { code: 200, data: { rrHomeId: 123 } }, final];
}

test("uses the current code flow, stable client headers, and signed home lookup", async (t) => {
  const calls = mockResponses(t, [{ code: 200 }, ...flow()]);
  const cloud = new RoborockCloud("eu");
  await cloud.requestCode(EMAIL);
  const devices = await cloud.loginWithCode(EMAIL, "123456");
  assert.deepEqual(devices, [
    { duid: "owned", name: "Z70", model: "roborock.vacuum.a147", localKey: KEY, protocolVersion: "1.0" },
    { duid: "shared", name: "Shared model", model: "roborock.vacuum.other", localKey: KEY, protocolVersion: "1.0" },
  ]);
  assert.deepEqual(calls.map(({ url }) => url.pathname), [
    "/api/v4/email/code/send", "/api/v3/key/sign", "/api/v4/auth/email/login/code",
    "/api/v1/getHomeDetail", "/v3/user/homes/123",
  ]);
  assert.deepEqual(calls.map(({ init }) => init.method), ["POST", "POST", "POST", "GET", "GET"]);
  assert.equal(String(calls[0].init.body), new URLSearchParams({ type: "login", email: EMAIL, platform: "" }).toString());
  const loginHeaders = new Headers(calls[2].init.headers);
  assert.equal(loginHeaders.get("header_clientid"), new Headers(calls[0].init.headers).get("header_clientid"));
  assert.equal(loginHeaders.get("x-mercy-k"), "sign-key");
  assert.equal(loginHeaders.get("x-mercy-ks"), calls[1].url.searchParams.get("s"));
  assert.match(calls[1].url.searchParams.get("s") ?? "", /^[A-Za-z0-9]{16}$/);
  assert.equal(String(calls[2].init.body), new URLSearchParams({
    country: "DE", countryCode: "49", email: EMAIL, code: "123456", majorVersion: "14", minorVersion: "0",
  }).toString());
  assert.equal(new Headers(calls[3].init.headers).get("Authorization"), "session-token");
  const signed = new Headers(calls[4].init.headers);
  const match = /^Hawk id="user123", s="salt123", ts="(\d+)", nonce="([A-Za-z0-9]{6})", mac="([^"]+)"$/.exec(signed.get("Authorization") ?? "");
  assert.ok(match);
  const pathHash = createHash("md5").update("/v3/user/homes/123").digest("hex");
  const expected = createHmac("sha256", "hawk-key").update(["user123", "salt123", match[2], match[1], pathHash, "", ""].join(":")).digest("base64");
  assert.equal(match[3], expected);
  assert.equal(signed.get("x-uid"), "user123");
  assert.equal(calls[4].url.origin, "https://api-eu.roborock.com");
});

test("uses explicit region origins and login country fields", async (t) => {
  const cases = [
    { region: "eu", host: "euiot.roborock.com", country: "DE", code: "49" },
    { region: "us", host: "usiot.roborock.com", country: "US", code: "1" },
    { region: "cn", host: "cniot.roborock.com", country: "CN", code: "86" },
    { region: "asia", host: "api.roborock.com", country: "SG", code: "65" },
  ] as const;
  for (const item of cases) {
    await t.test(item.region, async (t) => {
      const calls = mockResponses(t, flow());
      await new RoborockCloud(item.region).loginWithCode(EMAIL, "123456");
      assert.equal(calls[0].url.hostname, item.host);
      const params = new URLSearchParams(String(calls[1].init.body));
      assert.equal(params.get("country"), item.country);
      assert.equal(params.get("countryCode"), item.code);
    });

  }
});

test("camera login exports broker and signed home authorization privately", async (t) => {
  mockResponses(t, flow(home(), cameraLogin()));
  const result = await new RoborockCloud("eu").loginCameraWithCode(EMAIL, "123456");
  assert.equal(result.devices.length, 2);
  assert.deepEqual(result.account, {
    user: "user123", secret: "salt123", key: "domain-key", mqttUrl: "ssl://mqtt-eu.roborock.com:8883",
  });
  assert.ok(!JSON.stringify(result).includes("session-token"));
  assert.equal(result.home.homeId, 123);
  assert.deepEqual(result.home.rooms, []);
});

test("login extracts bounded account rooms without leaking web token", async (t) => {
  const response = home();
  Object.assign(response.result, { rooms: [{ id: 44, name: "Living room" }, { id: "45", name: "Hall" }] });
  mockResponses(t, flow(response, cameraLogin()));
  const result = await new RoborockCloud("eu").loginCameraWithCode(EMAIL, "123456");
  assert.deepEqual(result.home.rooms, [{ id: "44", name: "Living room" }, { id: "45", name: "Hall" }]);
  assert.ok(!JSON.stringify(result).includes("session-token"));
});

test("camera broker must be a Roborock TLS endpoint", async (t) => {
  for (const broker of ["mqtt://mqtt.roborock.com:1883", "ssl://attacker.invalid:8883",
    "ssl://mqtt.roborock.com:8883/path", "ssl://mqtt.roborock.com:8883?secret=value"]) {
    await t.test(broker, async (t) => {
      mockResponses(t, flow(home(), cameraLogin(broker)));
      await assert.rejects(new RoborockCloud("eu").loginCameraWithCode(EMAIL, "123456"), /broker/);
    });
  }
});

test("rejects untrusted dynamic endpoints before sending credentials", async (t) => {
  for (const endpoint of [
    "http://api-eu.roborock.com", "https://roborock.com.attacker.invalid", "https://evilroborock.com",
    "https://127.0.0.1", "https://api-eu.roborock.com:8443", "https://user:pass@api-eu.roborock.com",
    "https://api-eu.roborock.com/elsewhere", "https://api-eu.roborock.com?redirect=1",
    "https://api-eu.roborock.com/#fragment", "not-a-url",
  ]) {
    await t.test(endpoint, async (t) => {
      const calls = mockResponses(t, flow(home(), login(endpoint)));
      await assert.rejects(new RoborockCloud("eu").loginWithCode(EMAIL, "123456"), /endpoint/);
      assert.equal(calls.length, 2);
    });
  }
});

test("validates each response and does not return partial credentials", async (t) => {
  const badHome = home();
  badHome.result.receivedDevices[0].localKey = "bad";
  const unmatchedHome = home();
  unmatchedHome.result.devices[0].productId = "missing";
  const cases: unknown[][] = [
    [null], [{ code: 200 }], [{ code: 200, data: { k: "bad\r\nheader" } }],
    flow(home(), { code: 200, data: { token: "token" } }),
    flow(home(), { code: 200, data: { token: "token", rriot: { u: 'injected"id', s: "s", h: "h", r: { a: "https://api-eu.roborock.com" } } } }),
    [{ code: 200, data: { k: "k" } }, login(), { code: 200, data: { rrHomeId: "../bad" } }],
    flow({ success: false, result: home().result }),
    flow({ success: true, result: { products: {}, devices: [] } }),
    flow(badHome), flow(unmatchedHome),
  ];
  for (const [index, values] of cases.entries()) {
    await t.test(String(index), async (t) => {
      mockResponses(t, values);
      await assert.rejects(new RoborockCloud("eu").loginWithCode(EMAIL, "123456"), /incomplete or invalid/);
    });
  }
});

test("accepts a successful empty home, including omitted optional lists", async (t) => {
  mockResponses(t, flow({ success: true, result: { id: 123 } }));
  assert.deepEqual(await new RoborockCloud("eu").loginWithCode(EMAIL, "123456"), []);
});

test("sanitizes failures without exposing response or transport secrets", async (t) => {
  const secret = `${EMAIL} session-token ${KEY}`;
  for (const [index, response] of [
    { code: 2010, msg: secret },
    new Response(secret, { status: 401 }),
    new Response(secret, { status: 429 }),
    new Response(secret, { status: 500 }),
    new Response(secret, { status: 302, headers: { location: "https://attacker.invalid" } }),
    new Response(secret),
    new Error(secret),
  ].entries()) {
    await t.test(String(index), async (t) => {
      mockResponses(t, [response]);
      await assert.rejects(new RoborockCloud("eu").requestCode(EMAIL), (error: unknown) => {
        assert.ok(error instanceof Error);
        assert.doesNotMatch(error.message, /test@example|session-token|0123456789abcdef|attacker/);
        assert.equal(error.cause, undefined);
        return true;
      });
    });
  }
});

test("rejects invalid inputs without network calls", async (t) => {
  const calls = mockResponses(t, []);
  const cloud = new RoborockCloud("eu");
  await assert.rejects(cloud.requestCode("invalid"), /valid.*email/);
  await assert.rejects(cloud.loginWithCode(EMAIL, "abcd"), /six-digit/);
  assert.equal(calls.length, 0);
});

test("caps the streamed response size", async (t) => {
  mockResponses(t, [new Response("x".repeat(2 * 1024 * 1024 + 1))]);
  await assert.rejects(new RoborockCloud("eu").requestCode(EMAIL), /safe size limit/);
});

test("bounds fetch with an abort signal", async (t) => {
  t.mock.method(AbortSignal, "timeout", (milliseconds: number) => {
    assert.equal(milliseconds, 12_000);
    return AbortSignal.abort();
  });
  t.mock.method(globalThis, "fetch", async (_input: URL, init: RequestInit) => {
    init.signal?.throwIfAborted();
    throw new Error("Expected an aborted signal");
  });
  await assert.rejects(new RoborockCloud("eu").requestCode(EMAIL), /timed out/);
});

test("keeps the timeout active while consuming a stalled response body", async (t) => {
  const controller = new AbortController();
  t.mock.method(AbortSignal, "timeout", (milliseconds: number) => {
    assert.equal(milliseconds, 12_000);
    return controller.signal;
  });
  t.mock.method(globalThis, "fetch", async (_input: URL, init: RequestInit) => {
    assert.equal(init.signal, controller.signal);
    const body = new ReadableStream<Uint8Array>({
      start(stream) {
        controller.signal.addEventListener("abort", () => stream.error(new Error("private upstream failure")), { once: true });
        stream.enqueue(Buffer.from('{"code":'));
      },
      pull() {
        controller.abort();
      },
    });
    return new Response(body);
  });
  await assert.rejects(new RoborockCloud("eu").requestCode(EMAIL), /timed out/);
});

test("creates a fresh signature for each login attempt", async (t) => {
  const calls = mockResponses(t, [...flow(), ...flow()]);
  const cloud = new RoborockCloud("eu");
  await cloud.loginWithCode(EMAIL, "123456");
  await cloud.loginWithCode(EMAIL, "654321");
  assert.notEqual(calls[0].url.searchParams.get("s"), calls[4].url.searchParams.get("s"));
  assert.equal(new Headers(calls[0].init.headers).get("header_clientid"), new Headers(calls[4].init.headers).get("header_clientid"));
});
