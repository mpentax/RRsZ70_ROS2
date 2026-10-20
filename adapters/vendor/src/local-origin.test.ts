import { test } from "node:test";
import assert from "node:assert/strict";
import { localServiceOrigin } from "./local-origin.ts";

test("explicit native and container service origins are accepted", () => {
  assert.equal(localServiceOrigin("http://127.0.0.1:57001", "go2rtc"), "http://127.0.0.1:57001");
  assert.equal(localServiceOrigin("http://go2rtc:57001", "go2rtc"), "http://go2rtc:57001");
  assert.equal(localServiceOrigin("http://host.docker.internal:57001"), "http://host.docker.internal:57001");
  assert.equal(localServiceOrigin("http://[::1]:57001"), "http://[::1]:57001");
});

test("public providers, credentials, paths, and ambiguous destinations are rejected", () => {
  for (const origin of [
    "https://api.example.com", "http://8.8.8.8:57001", "http://localhost.attacker.invalid",
    "http://user:password@127.0.0.1:57001", "http://127.0.0.1:57001/api",
    "http://127.0.0.1:57001?redirect=anything", "file:///tmp/model",
  ]) assert.throws(() => localServiceOrigin(origin), /origin/);
});
