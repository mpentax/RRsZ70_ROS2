import { test } from "node:test";
import assert from "node:assert/strict";
import { chmod, lstat, mkdir, mkdtemp, readFile, readdir, realpath, rm, stat, symlink, writeFile } from "node:fs/promises";
import { randomInt } from "node:crypto";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { SecretStore, safeReason, selectedLocalProtocol, type PersistentCamera, type PersistentPairing } from "./secret-store.ts";

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

const home = {
  deviceId: device.duid, homeId: 123, realApi: "https://api-eu.roborock.com",
  user: "synthetic-user", secret: "synthetic-secret", hawkKey: "synthetic-hawk",
  rooms: [{ id: "44", name: "Living room" }],
};

function pairing(overrides: Partial<PersistentPairing> = {}): PersistentPairing {
  return { device, camera: camera(), home, ...overrides };
}

async function fixture(t: { after(fn: () => Promise<void>): void }) {
  const root = await mkdtemp(join(await realpath(tmpdir()), "roboclaw-private-"));
  t.after(() => rm(root, { recursive: true, force: true }));
  const directory = join(root, "state");
  return { root, directory, store: new SecretStore(directory) };
}

function safeError(error: unknown): boolean {
  assert.ok(error instanceof Error);
  assert.ok(!error.message.includes("synthetic"));
  assert.ok(!("cause" in error) || (typeof error.cause === "string" && !error.cause.includes("synthetic")));
  return true;
}

test("pairing and service token survive restart with exact schemas and private permissions", async (t) => {
  const { root, directory, store } = await fixture(t);
  await chmod(root, 0o755);
  await mkdir(directory, { mode: 0o755 });
  assert.equal(await store.loadPairing(), null);
  const value = pairing();
  await store.savePairing(value);
  await store.initialize();
  const token = await store.serviceToken();
  const restarted = new SecretStore(directory);
  await restarted.initialize();
  assert.deepEqual(await restarted.loadPairing(), value);
  assert.equal(await restarted.serviceToken(), token);
  assert.equal(Buffer.from(token, "base64url").length, 32);
  assert.equal((await stat(root)).mode & 0o777, 0o755);
  assert.equal((await stat(directory)).mode & 0o777, 0o700);
  assert.deepEqual((await readdir(directory)).sort(), ["pairing.json", "service-token"]);
  for (const file of await readdir(directory)) {
    assert.equal((await stat(join(directory, file))).mode & 0o777, 0o600);
  }
  assert.deepEqual(Object.keys(JSON.parse(await readFile(join(directory, "pairing.json"), "utf8"))).sort(),
    ["camera", "device", "home"]);
});

test("concurrent initialization publishes one complete persistent service token", async (t) => {
  const { directory, store } = await fixture(t);
  const stores = Array.from({ length: 24 }, () => new SecretStore(directory));
  await Promise.all(stores.map((value) => value.initialize()));
  const tokens = await Promise.all(stores.map((value) => value.serviceToken()));
  assert.equal(new Set(tokens).size, 1);
  assert.equal(tokens[0], await store.serviceToken());
  assert.deepEqual(await readdir(directory), ["service-token"]);
});

test("atomic replacement keeps pairing document whole without mixed credentials", async (t) => {
  const { directory, store } = await fixture(t);
  const initial = pairing();
  await store.savePairing(initial);
  await Promise.all(Array.from({ length: 12 }, async (_, index) => {
    const peer = new SecretStore(directory);
    await peer.savePairing(pairing({ device: { ...device, name: `Synthetic ${index}` }, camera: camera() }));
    assert.ok((await peer.loadPairing())?.device.name.startsWith("Synthetic"));
  }));
  await assert.rejects(store.savePairing(pairing({ home: { ...home, deviceId: "wrong" } })), safeError);
  assert.ok((await store.loadPairing())?.device.name.startsWith("Synthetic"));
  assert.deepEqual(await readdir(directory), ["pairing.json"]);
});

test("malformed, oversized and invalid UTF-8 JSON never leaks contents", async (t) => {
  const { directory, store } = await fixture(t);
  await store.initialize();
  for (const contents of [
    '{"synthetic-secret":"private", broken}',
    "synthetic-secret".repeat(2048),
    Buffer.from([0xff, 0xfe]),
    "null", "[]", "{}",
    JSON.stringify({ ...pairing(), device: { ...device, host: "synthetic.invalid" } }),
    JSON.stringify({ ...pairing(), camera: { ...camera(), email: "synthetic" } }),
  ]) {
    await writeFile(join(directory, "pairing.json"), contents, { mode: 0o600 });
    await assert.rejects(store.loadPairing(), safeError);
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
  for (const badDevice of invalid) {
    await assert.rejects(store.savePairing(pairing({ device: badDevice as typeof device, home: { ...home, deviceId: badDevice.duid } })), safeError);
  }
  await store.savePairing(pairing({ device: { ...device, host: "2001:db8::1" } }));
  assert.equal((await store.loadPairing())?.device.host, "2001:db8::1");
});

test("camera validation allows only account plus valid PIN and trusted SSL broker", async (t) => {
  const { store } = await fixture(t);
  const valid = camera();
  for (const badCamera of [
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
    await assert.rejects(store.savePairing(pairing({ camera: badCamera as PersistentCamera })), safeError);
  }
  for (const length of [4, 12]) {
    await store.savePairing(pairing({ camera: { ...valid, pin: Array.from({ length }, () => String(randomInt(1, 10))).join("") } }));
    assert.equal((await store.loadPairing())?.camera.pin.length, length);
  }
});

test("invalid or public files fail closed and are never overwritten", async (t) => {
  const { directory, store } = await fixture(t);
  await store.initialize();
  await store.savePairing(pairing());
  const tokenFile = join(directory, "service-token");
  for (const contents of ["", "synthetic-corrupt-token", "a".repeat(44), "_".repeat(43)]) {
    await writeFile(tokenFile, contents);
    await assert.rejects(store.initialize(), safeError);
    await assert.rejects(store.serviceToken(), safeError);
    assert.equal(await readFile(tokenFile, "utf8"), contents);
  }
  await chmod(tokenFile, 0o644);
  await assert.rejects(store.serviceToken(), safeError);
  assert.equal((await stat(tokenFile)).mode & 0o777, 0o644);

  const pairingFile = join(directory, "pairing.json");
  await chmod(pairingFile, 0o644);
  await assert.rejects(store.loadPairing(), safeError);
  await assert.rejects(store.savePairing(pairing()), safeError);
});

test("directory and state file symlinks are refused without modifying targets", async (t) => {
  const { root, directory, store } = await fixture(t);
  const targetDir = join(root, "outside-dir");
  await mkdir(targetDir, { mode: 0o755 });
  await chmod(targetDir, 0o755);
  await symlink(targetDir, directory);
  await assert.rejects(new SecretStore(directory).initialize(), safeError);
  assert.equal((await stat(targetDir)).mode & 0o777, 0o755);
  await rm(directory);

  await mkdir(directory, { mode: 0o700 });
  const outsideFile = join(root, "outside-file");
  await writeFile(outsideFile, "untouched synthetic sentinel", { mode: 0o600 });
  for (const name of ["pairing.json", "service-token"] as const) {
    const file = join(directory, name);
    await symlink(outsideFile, file);
    await assert.rejects(store.initialize(), safeError);
    if (name === "pairing.json") {
      await assert.rejects(store.loadPairing(), safeError);
      await assert.rejects(store.savePairing(pairing()), safeError);
    } else {
      await assert.rejects(store.serviceToken(), safeError);
    }
    assert.ok((await lstat(file)).isSymbolicLink());
    await rm(file);
  }
  assert.equal(await readFile(outsideFile, "utf8"), "untouched synthetic sentinel");
});

test("environment-selected default directory and unrelated files are handled cleanly", async (t) => {
  const { directory, store } = await fixture(t);
  await mkdir(directory, { recursive: true, mode: 0o700 });
  await writeFile(join(directory, "device.json"), "legacy split file ignored", { mode: 0o600 });
  const original = process.env.ROBOCLAW_STATE_DIR;
  process.env.ROBOCLAW_STATE_DIR = directory;
  try {
    const envStore = new SecretStore();
    await envStore.initialize();
    assert.equal(await envStore.serviceToken(), await store.serviceToken());
    assert.equal(await readFile(join(directory, "device.json"), "utf8"), "legacy split file ignored");
  } finally {
    if (original === undefined) delete process.env.ROBOCLAW_STATE_DIR;
    else process.env.ROBOCLAW_STATE_DIR = original;
  }
});

test("startup failure reasons keep errno codes and fixed labels, never paths or secrets", () => {
  const fsError = Object.assign(new Error("EACCES: permission denied, open '/data/pairing.json'"), { code: "EACCES" });
  assert.equal(safeReason(fsError), "EACCES");
  assert.equal(safeReason(new Error("Unsafe state owner.")), "Unsafe state owner.");
  assert.equal(safeReason(new SyntaxError("Unexpected token s in JSON at position 3: secret123")), "SyntaxError");
  assert.equal(safeReason("raw string"), "unknown");
});

test("explicit local L01 persists independently of cloud pv 1.0", async (t) => {
  const { store } = await fixture(t);
  await store.savePairing(pairing({ device: { ...device, localProtocol: "L01" } }));
  const saved = (await store.loadPairing())?.device;
  assert.ok(saved); assert.equal(saved.protocolVersion, "1.0");
  assert.equal(selectedLocalProtocol(saved), "L01");
  assert.equal(selectedLocalProtocol({ ...device, protocolVersion: "L01", localProtocol: undefined }), "1.0");
  await assert.rejects(store.savePairing(pairing({ device: { ...device, localProtocol: "unknown" as "L01" } })));
  assert.equal((await store.loadPairing())?.device.localProtocol, "L01");
});
