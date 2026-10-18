import { test } from "node:test";
import assert from "node:assert/strict";
import { chmod, lstat, mkdir, mkdtemp, readFile, readdir, realpath, rm, stat, symlink, writeFile } from "node:fs/promises";
import { randomInt } from "node:crypto";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SecretStore, type PersistentCamera } from "./secret-store.ts";
import { migrateState } from "./migrate-state.ts";

const device = {
  host: "192.0.2.10", duid: "synthetic-device", name: "Synthetic camera",
  model: "roborock.vacuum.test", localKey: "0123456789abcdef", protocolVersion: "1.0",
};

function camera(): PersistentCamera {
  return {
    account: {
      user: "synthetic-user", secret: "synthetic-secret", key: "synthetic-account-key",
      mqttUrl: "ssl://mqtt-eu.roborock.com:8883",
    },
    pin: Array.from({ length: 6 }, () => String(randomInt(1, 10))).join(""),
  };
}

async function fixture(t: { after(fn: () => Promise<void>): void }) {
  // macOS's system temporary directory may itself use an OS-provided symlink.
  const root = await mkdtemp(join(await realpath(tmpdir()), "roboclaw-private-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  const directory = join(root, "state");
  return { root, directory, store: new SecretStore(directory) };
}

function safeError(error: unknown): boolean {
  assert.ok(error instanceof Error);
  assert.match(error.message, /^(Cannot |Private state migration failed)/);
  assert.ok(!error.message.includes("synthetic"));
  // A cause may only be a safe reason: errno code, fixed internal label or error class name.
  assert.ok(!("cause" in error) || (typeof error.cause === "string" && !error.cause.includes("synthetic") &&
    /^(E[A-Z0-9]{2,20}|(Invalid|Unsafe|Private|Truncated)[ A-Za-z.]{0,80}|[A-Z][A-Za-z]*Error|Error|unknown)$/.test(error.cause)));
  return true;
}

function sourceFor(value: PersistentCamera): string {
  const broker = new URL(value.account.mqttUrl);
  const url = new URL(`roborock://${broker.host}`);
  url.search = new URLSearchParams({
    u: value.account.user, s: value.account.secret, k: value.account.key,
    did: device.duid, key: device.localKey, pin: value.pin,
  }).toString();
  return url.href;
}



test("private state survives restart with exact schemas and private permissions", async (t) => {
  const { root, directory, store } = await fixture(t);
  await chmod(root, 0o755);
  await mkdir(directory, { mode: 0o755 });
  assert.equal(await store.loadDevice(), null);
  assert.equal(await store.loadCamera(), null);
  const value = camera();
  await store.saveDevice(device);
  await store.saveCamera(value);
  await store.initialize();
  const token = await store.serviceToken();
  const restarted = new SecretStore(directory);
  await restarted.initialize();
  assert.deepEqual(await restarted.loadDevice(), device);
  assert.ok(JSON.stringify(await restarted.loadCamera()) === JSON.stringify(value));
  assert.ok(await restarted.serviceToken() === token);
  assert.equal(Buffer.from(token, "base64url").length, 32);
  assert.equal((await stat(root)).mode & 0o777, 0o755);
  assert.equal((await stat(directory)).mode & 0o777, 0o700);
  assert.deepEqual((await readdir(directory)).sort(), ["camera.json", "device.json", "service-token"]);
  for (const file of await readdir(directory)) {
    assert.equal((await stat(join(directory, file))).mode & 0o777, 0o600);
  }
  assert.deepEqual(Object.keys(JSON.parse(await readFile(join(directory, "camera.json"), "utf8"))).sort(),
    ["account", "pin"]);
  assert.equal(await readFile(join(directory, "device.json"), "utf8"), JSON.stringify(device));
});

test("selected home signing material survives restart and is private", async (t) => {
  const { directory, store } = await fixture(t);
  const home = { deviceId: device.duid, homeId: 123, realApi: "https://api-eu.roborock.com",
    user: "synthetic-user", secret: "synthetic-secret", hawkKey: "synthetic-hawk",
    rooms: [{ id: "44", name: "Living room" }] };
  assert.equal(await store.loadHome(), null);
  await store.saveHome(home);
  const restarted = new SecretStore(directory);
  assert.deepEqual(await restarted.loadHome(), home);
  assert.equal((await stat(join(directory, "home.json"))).mode & 0o777, 0o600);
  await store.clearHome();
  assert.equal(await restarted.loadHome(), null);
});

test("concurrent initialization publishes one complete persistent service token", async (t) => {
  const { directory, store } = await fixture(t);
  const stores = Array.from({ length: 24 }, () => new SecretStore(directory));
  await Promise.all(stores.map((value) => value.initialize()));
  const tokens = await Promise.all(stores.map((value) => value.serviceToken()));
  assert.equal(new Set(tokens).size, 1);
  assert.ok(tokens[0] === await store.serviceToken());
  assert.deepEqual(await readdir(directory), ["service-token"]);
});

test("independent processes race token creation without replacement or partial reads", async (t) => {
  const { directory, store } = await fixture(t);
  const script = `
    import { createHash } from "node:crypto";
    const { SecretStore } = await import(process.argv[1]);
    const store = new SecretStore(process.argv[2]);
    await store.initialize();
    process.stdout.write(createHash("sha256").update(await store.serviceToken()).digest("hex"));
  `;
  const results = await Promise.all(Array.from({ length: 8 }, () => promisify(execFile)(
    process.execPath,
    ["--input-type=module", "-e", script, new URL("./secret-store.ts", import.meta.url).href, directory],
  )));
  assert.equal(new Set(results.map((result) => result.stdout)).size, 1);
  assert.ok(results.every((result) => result.stderr === ""));
  assert.equal((await store.serviceToken()).length, 43);
  assert.deepEqual(await readdir(directory), ["service-token"]);
});

test("atomic replacement keeps device and camera documents whole", async (t) => {
  const { directory, store } = await fixture(t);
  await store.saveDevice(device);
  await store.saveCamera(camera());
  await Promise.all(Array.from({ length: 12 }, async (_, index) => {
    const peer = new SecretStore(directory);
    await peer.saveDevice({ ...device, name: `Synthetic ${index}` });
    assert.ok((await peer.loadDevice())?.name.startsWith("Synthetic"));
    await peer.saveCamera(camera());
    assert.ok(await peer.loadCamera());
  }));
  assert.deepEqual((await readdir(directory)).sort(), ["camera.json", "device.json"]);
});


test("malformed, oversized and invalid UTF-8 JSON never leaks contents", async (t) => {
  const { directory, store } = await fixture(t);
  await store.initialize();
  for (const name of ["device.json", "camera.json"]) {
    for (const contents of [
      '{"synthetic-secret":"private", broken}',
      "synthetic-secret".repeat(2048),
      Buffer.from([0xff, 0xfe]),
      "null", "[]", "{}",
    ]) {
      await writeFile(join(directory, name), contents, { mode: 0o600 });
      await assert.rejects(name === "device.json" ? store.loadDevice() : store.loadCamera(), safeError);
    }
  }
});

test("device validation rejects unknown fields, nonliteral IPs and empty strings", async (t) => {
  const { store } = await fixture(t);
  const invalid = [
    { ...device, email: "synthetic@example.invalid" },
    { ...device, localKey: "" },
    { ...device, duid: " " },
    { ...device, name: "invalid\nname" },
    { ...device, host: "localhost" },
    { ...device, host: "https://192.0.2.10" },
    { ...device, host: "192.0.2.10:1984" },
  ];
  for (const value of invalid) await assert.rejects(store.saveDevice(value), safeError);
  await store.saveDevice({ ...device, host: "2001:db8::1" });
  assert.equal((await store.loadDevice())?.host, "2001:db8::1");
});

test("camera validation allows only account plus valid PIN and trusted SSL broker", async (t) => {
  const { store } = await fixture(t);
  const valid = camera();
  for (const value of [
    { ...valid, email: "synthetic@example.invalid" },
    { ...valid, pattern: valid.pin },
    { ...valid, account: { ...valid.account, sessionToken: "synthetic-session" } },
    { ...valid, account: { ...valid.account, secret: "" } },
    ...["0".repeat(6), "1".repeat(3), "1".repeat(13), "a".repeat(6), " ".repeat(6)]
      .map((pin) => ({ ...valid, pin })),
    ...[
      "ssl://attacker.invalid:8883", "ssl://roborock.com.attacker.invalid:8883",
      "ssl://roborock.com:8883", "mqtt://mqtt-eu.roborock.com:8883",
      "ssl://mqtt-eu.roborock.com", "ssl://mqtt-eu.roborock.com:0",
      "ssl://mqtt-eu.roborock.com:8883/path", "ssl://mqtt-eu.roborock.com:8883?query=1",
      "ssl://mqtt-eu.roborock.com:8883#fragment", "ssl://user@mqtt-eu.roborock.com:8883",
      "synthetic-invalid-url",
    ].map((mqttUrl) => ({ ...valid, account: { ...valid.account, mqttUrl } })),
  ]) {
    await assert.rejects(store.saveCamera(value), safeError);
  }
  for (const length of [4, 12]) {
    await store.saveCamera({ ...valid, pin: Array.from({ length }, () => String(randomInt(1, 10))).join("") });
    assert.equal((await store.loadCamera())?.pin.length, length);
  }
});

test("invalid data already on disk receives the same strict validation", async (t) => {
  const { directory, store } = await fixture(t);
  await store.initialize();
  await writeFile(join(directory, "device.json"), JSON.stringify({ ...device, host: "synthetic.invalid" }), { mode: 0o600 });
  await writeFile(join(directory, "camera.json"), JSON.stringify({ ...camera(), email: "synthetic" }), { mode: 0o600 });
  await assert.rejects(store.loadDevice(), safeError);
  await assert.rejects(store.loadCamera(), safeError);
});

test("invalid or public tokens fail closed and are never overwritten", async (t) => {
  const { directory, store } = await fixture(t);
  await store.initialize();
  const file = join(directory, "service-token");
  for (const contents of ["", "synthetic-corrupt-token", "a".repeat(44), "_".repeat(43)]) {
    await writeFile(file, contents);
    await assert.rejects(store.initialize(), safeError);
    await assert.rejects(store.serviceToken(), safeError);
    assert.equal(await readFile(file, "utf8"), contents);
  }
  await chmod(file, 0o644);
  await assert.rejects(store.serviceToken(), safeError);
  assert.equal((await stat(file)).mode & 0o777, 0o644);
});

test("public credential files are refused rather than silently replaced", async (t) => {
  const { directory, store } = await fixture(t);
  await store.saveDevice(device);
  await store.saveCamera(camera());
  for (const file of ["device.json", "camera.json"]) await chmod(join(directory, file), 0o644);
  await assert.rejects(store.loadDevice(), safeError);
  await assert.rejects(store.loadCamera(), safeError);
  await assert.rejects(store.saveDevice(device), safeError);
  await assert.rejects(store.saveCamera(camera()), safeError);
});

test("state directory and ancestor symlinks are refused without chmod'ing their targets", async (t) => {
  const { root, directory } = await fixture(t);
  const target = join(root, "outside");
  await mkdir(target, { mode: 0o755 });
  await chmod(target, 0o755);
  await symlink(target, directory);
  await assert.rejects(new SecretStore(directory).initialize(), safeError);
  await assert.rejects(new SecretStore(join(directory, "nested")).initialize(), safeError);
  assert.equal((await stat(target)).mode & 0o777, 0o755);
  assert.deepEqual(await readdir(target), []);
});

test("all state file symlinks, including dangling links, are refused", async (t) => {
  const { root, directory, store } = await fixture(t);
  await mkdir(directory, { mode: 0o700 });
  const outside = join(root, "outside");
  await writeFile(outside, "untouched synthetic sentinel", { mode: 0o600 });
  for (const name of ["device.json", "camera.json", "service-token"]) {
    const file = join(directory, name);
    for (const target of [outside, join(root, "absent")]) {
      await symlink(target, file);
      await assert.rejects(store.initialize(), safeError);
      if (name === "device.json") {
        await assert.rejects(store.loadDevice(), safeError);
        await assert.rejects(store.saveDevice(device), safeError);
      } else if (name === "camera.json") {
        await assert.rejects(store.loadCamera(), safeError);
        await assert.rejects(store.saveCamera(camera()), safeError);
      } else if (name === "service-token") {
        await assert.rejects(store.serviceToken(), safeError);
      } else {
        await assert.rejects(store.initialize(), /actuation is disabled/);
      }
      assert.ok((await lstat(file)).isSymbolicLink());
      await rm(file);
    }
  }
  assert.equal(await readFile(outside, "utf8"), "untouched synthetic sentinel");
});


test("environment-selected default directory is honored without touching real state", async (t) => {
  const { directory } = await fixture(t);
  const original = process.env.ROBOCLAW_STATE_DIR;
  process.env.ROBOCLAW_STATE_DIR = directory;
  try {
    const store = new SecretStore();
    await store.initialize();
    assert.ok(await store.serviceToken() === await new SecretStore(directory).serviceToken());
  } finally {
    if (original === undefined) delete process.env.ROBOCLAW_STATE_DIR;
    else process.env.ROBOCLAW_STATE_DIR = original;
  }
});

test("migration copies only matching selected-device account and PIN from the fixed loopback endpoint", async (t) => {
  const { directory, store } = await fixture(t);
  const value = camera();
  value.account.secret = "synthetic+escaped&secret";
  await store.saveDevice(device);
  await writeFile(join(directory, "motion-hold.json"), "synthetic hold", { mode: 0o600 });
  t.mock.method(globalThis, "fetch", async (endpoint: URL, options: RequestInit) => {
    assert.equal(endpoint.href, "http://127.0.0.1:57001/api/streams?src=z70");
    assert.equal(options.redirect, "error");
    assert.ok(options.signal);
    return Response.json({ producers: [{ url: sourceFor(value) }] });
  });
  assert.deepEqual(await migrateState(directory, "http://127.0.0.1:57001"), { migrated: true, pinCached: true });
  assert.ok(JSON.stringify(await store.loadCamera()) === JSON.stringify(value));
  assert.deepEqual(await store.loadDevice(), device);
  assert.equal(await readFile(join(directory, "motion-hold.json"), "utf8"), "synthetic hold");
  assert.deepEqual((await readdir(directory)).sort(), ["camera.json", "device.json", "motion-hold.json"]);
  await assert.rejects(migrateState(directory), safeError);
});

test("migration rejects nonloopback or nonallowlisted origins before fetch", async (t) => {
  const { directory } = await fixture(t);
  t.mock.method(globalThis, "fetch", () => { assert.fail("Unexpected network access"); });
  for (const origin of [
    "http://example.invalid:1984", "http://127.0.0.1:9999", "https://127.0.0.1:1984",
    "http://localhost:1984", "http://user@127.0.0.1:1984",
    "http://127.0.0.1:1984/path", "http://127.0.0.1:1984?query=1", "http://127.0.0.1:1984#fragment",
  ]) {
    await assert.rejects(migrateState(directory, origin), safeError);
  }
});

test("migration refuses mismatched identifiers, extra or duplicate parameters, and untrusted brokers", async (t) => {
  const { directory, store } = await fixture(t);
  await store.saveDevice(device);
  const source = sourceFor(camera());
  const invalid = [
    source.replace(device.duid, "synthetic-other-device"),
    source.replace(device.localKey, "synthetic-other-local-key"),
    `${source}&did=${device.duid}`, `${source}&email=synthetic`,
    source.replace("mqtt-eu.roborock.com", "synthetic.invalid"),
    source.replace("pin=", "missing="),
  ];
  let current = source;
  t.mock.method(globalThis, "fetch", async () => Response.json({ producers: [{ url: current }] }));
  for (const value of invalid) {
    current = value;
    await assert.rejects(migrateState(directory), safeError);
    assert.equal(await store.loadCamera(), null);
  }
});

test("migration rejects ambiguous, oversized, malformed and failed driver responses safely", async (t) => {
  const { directory, store } = await fixture(t);
  await store.saveDevice(device);
  const source = sourceFor(camera());
  let response: () => Response = () => Response.json({});
  t.mock.method(globalThis, "fetch", async () => response());
  for (const createResponse of [
    () => Response.json({}),
    () => Response.json({ producers: [{ url: source }, { url: source }] }),
    () => new Response('{"synthetic-secret":broken}'),
    () => new Response("synthetic-secret", { status: 500 }),
    () => new Response("synthetic-secret".repeat(100_000)),
    () => { throw new Error("synthetic-secret upstream failure"); },
  ]) {
    response = createResponse;
    await assert.rejects(migrateState(directory), safeError);
    assert.equal(await store.loadCamera(), null);
  }
});

test("migration CLI emits only the success flags and uses only mocked synthetic credentials", async (t) => {
  const { directory, store } = await fixture(t);
  await store.saveDevice(device);
  const value = camera();
  const preload = `globalThis.fetch = async (url, options) => {
    if (url.href !== "http://127.0.0.1:1984/api/streams?src=z70" || options.redirect !== "error") {
      throw new Error("Unexpected migration endpoint");
    }
    return Response.json(${JSON.stringify({ producers: [{ url: sourceFor(value) }] })});
  };`;
  const result = await promisify(execFile)(process.execPath, [
    "--import", `data:text/javascript,${encodeURIComponent(preload)}`,
    fileURLToPath(new URL("./migrate-state.ts", import.meta.url)),
  ], { env: { ...process.env, ROBOCLAW_STATE_DIR: directory } });
  assert.equal(result.stdout, '{"migrated":true,"pinCached":true}\n');
  assert.equal(result.stderr, "");
  assert.ok(JSON.stringify(await store.loadCamera()) === JSON.stringify(value));
});

test("migration CLI rejects arguments without printing values or touching state", async (t) => {
  const { directory } = await fixture(t);
  await assert.rejects(promisify(execFile)(process.execPath, [
    fileURLToPath(new URL("./migrate-state.ts", import.meta.url)),
    "--unrecognized", "synthetic-sensitive-argument",
  ], { env: { ...process.env, ROBOCLAW_STATE_DIR: directory } }), (error: unknown) => {
    assert.ok(error instanceof Error && "stdout" in error && "stderr" in error && "code" in error);
    assert.equal(error.stdout, "");
    assert.equal(error.stderr, "Private state migration failed.\n");
    assert.equal(error.code, 1);
    return true;
  });
  await assert.rejects(lstat(directory), { code: "ENOENT" });
});

test("obsolete motion holds and uncertainty journals are not interpreted or required during startup", async t => {
  const { directory, store } = await fixture(t);
  await mkdir(directory, { recursive: true, mode: 0o700 });
  await symlink("/does/not/exist", join(directory, "motion-hold.json"));
  await writeFile(join(directory, "command-authority.json"), "malformed historical unknown action", { mode: 0o600 });
  await store.initialize();
  assert.ok(await store.serviceToken());
  assert.ok((await lstat(join(directory, "motion-hold.json"))).isSymbolicLink());
  assert.equal(await readFile(join(directory, "command-authority.json"), "utf8"), "malformed historical unknown action");
});

test("atomic pairing is validated as one document and survives restart without mixed credentials", async t => {
  const { directory, store } = await fixture(t);
  const home = { deviceId: device.duid, homeId: 1, realApi: "https://api-eu.roborock.com", user: "user", secret: "secret", hawkKey: "hawk", rooms: [] };
  const pairing = { device, camera: camera(), home };
  await store.savePairing(pairing);
  assert.deepEqual(await new SecretStore(directory).loadPairing(), pairing);
  await assert.rejects(store.savePairing({ ...pairing, home: { ...home, deviceId: "wrong" } }));
  assert.deepEqual(await store.loadPairing(), pairing);
  assert.equal((await stat(join(directory, "pairing.json"))).mode & 0o777, 0o600);
});
