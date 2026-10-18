import assert from "node:assert/strict";
import { createHash, createHmac } from "node:crypto";
import { test } from "node:test";
import { refreshRooms, selectedRooms, validateHomeAuthorization } from "./home-account.ts";

const home = { deviceId: "selected", homeId: 123, realApi: "https://api-eu.roborock.com",
  user: "user", secret: "secret", hawkKey: "hawk", rooms: [{ id: "44", name: "Hall" }] };

test("single selected home authorization validates exact shape and preserves room labels", () => {
  assert.deepEqual(selectedRooms(validateHomeAuthorization(home)), { source: "roborock-account-home", rooms: home.rooms });
  for (const bad of [ { ...home, deviceId: "" }, { ...home, realApi: "https://evil.invalid" },
    { ...home, realApi: "https://api-eu.roborock.com/other" }, { ...home, token: "unexpected" },
    { ...home, rooms: [{ id: "44", name: "Hall" }, { id: "44", name: "Duplicate" }] },
    { ...home, rooms: [{ id: "44", name: "bad\nheader" }] } ]) {
    assert.throws(() => validateHomeAuthorization(bad));
  }
});

test("read-only refresh is Hawk signed to selected home and never exposes credentials", async (t) => {
  t.mock.method(globalThis, "fetch", async (input: URL, init: RequestInit) => {
    assert.equal(input.href, "https://api-eu.roborock.com/v3/user/homes/123");
    assert.equal(init.method, "GET"); assert.equal(init.redirect, "error");
    assert.ok(init.signal instanceof AbortSignal);
    const header = new Headers(init.headers).get("Authorization") ?? "";
    const match = /^Hawk id="user", s="secret", ts="(\d+)", nonce="([A-Za-z0-9]{6})", mac="([^"]+)"$/.exec(header);
    assert.ok(match);
    const hash = createHash("md5").update("/v3/user/homes/123").digest("hex");
    assert.equal(match[3], createHmac("sha256", "hawk").update(["user", "secret", match[2], match[1], hash, "", ""].join(":")).digest("base64"));
    return Response.json({ success: true, result: { rooms: [{ id: 99, name: "Study" }] } });
  });
  assert.deepEqual(await refreshRooms(home), [{ id: "99", name: "Study" }]);
  assert.deepEqual(home.rooms, [{ id: "44", name: "Hall" }]);
});

test("refresh failure never turns a cloud error into a room list", async (t) => {
  t.mock.method(globalThis, "fetch", async () => new Response("private account material", { status: 403 }));
  await assert.rejects(refreshRooms(home), (error: unknown) => error instanceof Error &&
    !error.message.includes("private account material"));
});
