import test from "node:test";
import assert from "node:assert/strict";
import { loadServiceConfig, localOrigin, localServiceOrigin } from "./service-config.ts";
import { fileURLToPath } from "node:url";

test("explicit native loopback and LAN service origins are accepted", () => {
  assert.equal(localServiceOrigin("http://127.0.0.1:57001", "go2rtc"), "http://127.0.0.1:57001");
  assert.equal(localServiceOrigin("http://localhost:57001", "go2rtc"), "http://localhost:57001");
  assert.equal(localServiceOrigin("http://[::1]:57001"), "http://[::1]:57001");
});

test("public providers, credentials, paths, and ambiguous destinations are rejected", () => {
  for (const origin of [
    "https://api.example.com", "http://8.8.8.8:57001", "http://localhost.attacker.invalid",
    "http://user:password@127.0.0.1:57001", "http://127.0.0.1:57001/api",
    "http://127.0.0.1:57001?redirect=anything", "file:///tmp/model",
  ]) assert.throws(() => localServiceOrigin(origin), /origin/);
});

test("workspace entrypoints retain one state/static root outside the repository cwd", t => {
  t.mock.method(process, "cwd", () => "/missing-caller-directory");
  const config = loadServiceConfig({});
  assert.equal(config.stateDir, fileURLToPath(new URL("../../../.local", import.meta.url)));
  assert.equal(config.webRoot, fileURLToPath(new URL("../../../dist/web", import.meta.url)));
});

test("native service defaults and strict port range", () => {
  const config = loadServiceConfig({});
  assert.equal(config.host, "127.0.0.1");
  assert.equal(config.port, 57000);
  assert.equal(config.publicOrigin, "http://127.0.0.1:57000");
  assert.equal(config.go2rtcOrigin, "http://127.0.0.1:57001");
  for (const port of ["0", "80", "56999", "57101", "57000junk", "57000.0"]) {
    assert.throws(() => loadServiceConfig({ ROBOCLAW_PORT: port }));
  }
  assert.equal(loadServiceConfig({ ROBOCLAW_PORT: "57100" }).port, 57100);
});

test("container and explicit TLS proxy configuration has exact host validation", () => {
  const config = loadServiceConfig({
    ROBOCLAW_HOST: "0.0.0.0", ROBOCLAW_PUBLIC_ORIGIN: "https://robot.example",
    ROBOCLAW_SECURE_COOKIE: "true", ROBOCLAW_ALLOWED_HOSTS: "127.0.0.1:57000,robot.internal:57000",
    ROBOCLAW_STATE_DIR: "/data", GO2RTC_ORIGIN: "http://127.0.0.1:57001",
  });
  assert.equal(config.secureCookie, true);
  assert.equal(config.stateDir, "/data");
  assert.deepEqual(config.allowedHosts, ["robot.example", "127.0.0.1:57000", "robot.internal:57000"]);
  for (const hosts of ["*", "evil.test/path", "user:pass@evil.test", "evil.test?x"]) {
    assert.throws(() => loadServiceConfig({ ROBOCLAW_ALLOWED_HOSTS: hosts }));
  }
  assert.throws(() => loadServiceConfig({ ROBOCLAW_PUBLIC_ORIGIN: "https://robot.example" }));
  assert.throws(() => loadServiceConfig({ ROBOCLAW_SECURE_COOKIE: "true" }));
});

test("local upstream origins reject cloud hosts, credentials and URL tricks", () => {
  for (const value of ["https://api.example", "http://go2rtc.evil:57001", "http://127.0.0.1:57001/path",
    "http://user:secret@127.0.0.1:57001", "http://127.0.0.1:57001#secret",
    "http://127.0.0.1:57001/", "http://0.0.0.0:57001", "http://169.254.169.254:57001"]) {
    assert.throws(() => localOrigin(value));
  }
  assert.equal(localOrigin("http://192.168.1.2:57001"), "http://192.168.1.2:57001");
  assert.equal(localOrigin("http://[::1]:57003"), "http://[::1]:57003");
  assert.throws(() => localOrigin("http://127.0.0.1:1984"));
});
