import { test } from "node:test";
import assert from "node:assert/strict";
import { chmod, lstat, mkdir, mkdtemp, readFile, readdir, realpath, rm, stat, symlink, unlink, writeFile } from "node:fs/promises";
import { createCipheriv, createHash, randomInt } from "node:crypto";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { gzipSync } from "node:zlib";
import {
  type BlobSecurity,
  createBlobSecurity,
  decodeCloudMapResponse,
  decodeMapResponse,
  encodeV1MqttFrame,
  inspectMapFrame,
  MAX_CLOUD_MAP_FRAME_BYTES,
} from "./cloud.ts";
import { localProtocolCodec } from "./robot.ts";
import {
  mapMetadataFromRead,
  savePrivateV1Map,
  SecretStore,
  safeReason,
  selectedLocalProtocol,
  summarizeV1Map,
  syntheticV1Image,
  validatedMapMetadata,
  type PersistentCamera,
  type PersistentPairing,
} from "./secret-store.ts";

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

function encodeMapResponse(data: Buffer, security: BlobSecurity, requestId = 20009): Buffer {
  const cipher = createCipheriv("aes-128-cbc", security.nonce, Buffer.alloc(16));
  const encrypted = Buffer.concat([cipher.update(gzipSync(data)), cipher.final()]);
  const header = Buffer.alloc(24);
  header.write(security.endpoint, 0, "ascii");
  header.writeUInt16LE(requestId, 16);
  return Buffer.concat([header, encrypted]);
}

const LOCAL_KEY = "0123456789abcdef";
function response(data: Buffer, security = createBlobSecurity("map-test-key"), requestId = 20009): Buffer {
  return encodeV1MqttFrame(LOCAL_KEY, 301, encodeMapResponse(data, security, requestId), 1753606905);
}

test("blob security uses account-key hash bytes 8..13 and a fresh 16-byte nonce", () => {
  const first = createBlobSecurity("synthetic-account-key");
  const second = createBlobSecurity("synthetic-account-key");
  assert.equal(first.endpoint, createHash("md5").update("synthetic-account-key")
    .digest().subarray(8, 14).toString("base64"));
  assert.equal(first.nonce.length, 16);
  assert.notDeepEqual(first.nonce, second.nonce);
  assert.throws(() => createBlobSecurity(""), /account key/);
});

test("map envelope checks endpoint and request, then decrypts CBC and decompresses", () => {
  const security = createBlobSecurity("synthetic-key");
  const data = Buffer.from("synthetic-decompressed-map");
  const payload = encodeMapResponse(data, security);
  assert.deepEqual(decodeMapResponse(301, payload, security, 20009), { requestId: 20009, data });
  assert.equal(decodeMapResponse(102, payload, security), undefined);
  assert.equal(decodeMapResponse(301, payload, security, 3), undefined);
  assert.equal(decodeMapResponse(301, payload, createBlobSecurity("different-key")), undefined);
  assert.throws(() => decodeMapResponse(301, payload.subarray(0, -1), security), /frame size/);
  const damaged = Buffer.from(payload);
  damaged.fill(0, 24);
  assert.throws(() => decodeMapResponse(301, damaged, security), /decrypt|compressed/);
  assert.throws(() => decodeMapResponse(301, payload, { ...security, nonce: Buffer.alloc(1) }), /security/);
});

test("cloud map probe accepts only a CRC-verified protocol-301 response bound to its nonce and request", () => {
  const security = createBlobSecurity("map-test-key");
  const data = Buffer.from("private map bytes");
  const frame = response(data, security);
  assert.deepEqual(decodeCloudMapResponse(frame, LOCAL_KEY, security, 20009), { requestId: 20009, data });
  assert.equal(decodeCloudMapResponse(frame, LOCAL_KEY, security, 20010), undefined);
  assert.equal(decodeCloudMapResponse(frame, LOCAL_KEY, createBlobSecurity("another-key"), 20009), undefined);
  frame[20] ^= 1;
  assert.throws(() => decodeCloudMapResponse(frame, LOCAL_KEY, security, 20009), /Invalid cloud map/);
});

test("cloud map probe refuses malformed and oversize outer frames before parsing", () => {
  const security = createBlobSecurity("map-test-key");
  assert.throws(() => decodeCloudMapResponse(Buffer.alloc(22), LOCAL_KEY, security, 1), /Invalid cloud map/);
  assert.throws(() => decodeCloudMapResponse(Buffer.alloc(MAX_CLOUD_MAP_FRAME_BYTES + 1), LOCAL_KEY, security, 1), /Invalid cloud map/);
});

test("map-frame inspection never calls an opaque 301 frame correlated", () => {
  const security = createBlobSecurity("synthetic-map-key");
  const frame = response(Buffer.from("synthetic map"), security, 15001);
  assert.notDeepEqual(frame.subarray(19, 27), Buffer.from(security.endpoint));
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15001), "correlated-v1");
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15002), "unrelated");
  const opaque = Buffer.from(frame);
  opaque.fill(0x91, 19, 27);
  opaque.writeUInt32BE(localProtocolCodec.crc32(opaque.subarray(0, -4)), opaque.length - 4);
  assert.equal(inspectMapFrame(opaque, LOCAL_KEY, security, 15001), "opaque-301");
  assert.equal(inspectMapFrame(Buffer.alloc(23), LOCAL_KEY, security, 15001), "malformed");
});

const L01_KEY = "b8Hj5mFk3QzT7rLp";
const ACK_NONCE = 485592656;

test("CRC32 matches the standard check vector", () => {
  assert.equal(localProtocolCodec.crc32(Buffer.from("123456789")), 0xcbf43926);
  assert.equal(localProtocolCodec.crc32(Buffer.alloc(0)), 0);
});

test("L01 matches upstream cryptoEngine.test.ts official vector", () => {
  const context = {
    version: "L01" as const, localKey: L01_KEY, timestamp: 1753606905, sequence: 1,
    random: 304251, connectNonce: 893563, ackNonce: ACK_NONCE,
  };
  const plain = Buffer.from('{"dps":{"101":"{\\"id\\":1806,\\"method\\":\\"get_prop\\",\\"params\\":[\\"get_status\\"]}"},"t":1753606905}');
  const encrypted = Buffer.from([
    253,96,200,218,202,28,202,230,127,96,119,71,123,250,157,55,24,154,56,215,91,60,74,144,124,36,53,211,193,70,238,132,216,249,149,151,
    227,225,87,26,1,89,97,206,170,77,100,188,54,149,250,224,36,195,65,103,55,215,113,80,52,29,226,156,173,47,149,191,175,83,35,88,241,
    43,191,248,159,20,15,239,91,30,226,132,195,171,254,59,131,165,119,145,10,114,5,109,171,77,90,117,177,130,209,160,203,161,69,227,
    228,80,243,146,116,67,
  ]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), encrypted);
  assert.deepEqual(localProtocolCodec.decryptPayload(encrypted, context), plain);
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, ackNonce: ACK_NONCE + 1 }));
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, connectNonce: context.connectNonce + 1 }));
});

test("V1 follows the independent upstream MD5 timestamp permutation and AES-ECB specification", () => {
  const context = {
    version: "1.0" as const, localKey: L01_KEY, timestamp: 0x12345678,
    sequence: 1, random: 2, connectNonce: 3, ackNonce: 4,
  };
  const key = createHash("md5").update("67482315" + L01_KEY + "TXdfu$jyZ#TZHsg4").digest();
  const cipher = createCipheriv("aes-128-ecb", key, null);
  const plain = Buffer.from("read-only status");
  const expected = Buffer.concat([cipher.update(plain), cipher.final()]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), expected);
  assert.deepEqual(localProtocolCodec.decryptPayload(expected, context), plain);
});

function block(type: number, headerSize: number, data: Buffer, image = false): Buffer {
  const result = Buffer.alloc(headerSize + data.length);
  result.writeUInt16LE(type); result.writeUInt16LE(headerSize, 2); result.writeUInt32LE(data.length, 4);
  if (image) {
    result.writeInt32LE(2, headerSize - 16); result.writeInt32LE(4, headerSize - 12);
    result.writeUInt32LE(2, headerSize - 8); result.writeUInt32LE(2, headerSize - 4);
  }
  data.copy(result, headerSize); return result;
}

function sample(vacuumX = 200, vacuumY = 100): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = block(2, 24, Buffer.from([1, 31, 0, 255]), true);
  const position = Buffer.alloc(12); position.writeInt32LE(vacuumX); position.writeInt32LE(vacuumY, 4);
  const digest = block(1024, 8, Buffer.alloc(20));
  const raw = Buffer.concat([header, image, block(8, 8, position), digest]);
  raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}

test("bounded V1 summary extracts image extent, segment and embedded position", () => {
  const result = summarizeV1Map(sample());
  assert.deepEqual(result.image.roomSegments, [3]);
  assert.deepEqual(result.vacuumPosition, { x: 200, y: 100, angle: 0 });
  assert.equal(result.embeddedVacuumRoomSegment, null);
  assert.equal(result.image.pixels, 4);
});

test("fractional embedded position uses Android plugin's floor grid index, not rounding", () => {
  assert.equal(summarizeV1Map(sample(230, 100)).embeddedVacuumRoomSegment, null);
  assert.equal(summarizeV1Map(sample(250, 100)).embeddedVacuumRoomSegment, 3);
  assert.equal(summarizeV1Map(sample(300, 100)).embeddedVacuumRoomSegment, null);
});

test("V1 summary fails closed on altered digest or invalid image geometry", () => {
  const damaged = sample(); damaged[27] ^= 1;
  assert.throws(() => summarizeV1Map(damaged), /integrity/);
  const bad = sample(); bad.writeUInt32LE(3, 20 + 24 - 4);
  createHash("sha1").update(bad.subarray(0, -20)).digest().copy(bad, bad.length - 20);
  assert.throws(() => summarizeV1Map(bad), /image size/);
});

test("private map metadata persists bound to selected device, checks bytes and private modes", async t => {
  const root = await realpath(await mkdtemp(join(tmpdir(), "roboclaw-map-metadata-")));
  t.after(() => rm(root, { recursive: true, force: true }));
  const store = new SecretStore(join(root, "state"));
  await store.initialize();
  const raw = syntheticV1Image(), sha256 = createHash("sha256").update(raw).digest("hex");
  const result = { raw, sha256, bytes: raw.length, mapIndex: 0, mapSequence: 0 };
  await store.saveMapBytes(raw);
  await store.saveMapCache(mapMetadataFromRead("private-device", result));
  const loaded = await store.loadMapRasterSource();
  assert.ok(loaded);
  const metadata = validatedMapMetadata(loaded.metadata);
  assert.equal(metadata.deviceId, "private-device");
  assert.equal(metadata.sha256, sha256);
  assert.equal((await lstat(join(root, "state", "map-cache.json"))).mode & 0o777, 0o600);
  assert.equal((await lstat(join(root, "state", "maps"))).mode & 0o777, 0o700);
  assert.equal((await lstat(join(root, "state"))).mode & 0o777, 0o700);
  const file = join(root, "state", "maps", `v1-${sha256}.bin`);
  const corrupted = Buffer.from(await readFile(file)); corrupted[25] ^= 1;
  await writeFile(file, corrupted);
  await assert.rejects(store.loadMapRasterSource(), /Cannot load private map raster source/);
});

test("private cache is content-addressed, non-overwriting, restricted and idempotent", async t => {
  const base = await realpath(await mkdtemp(join(tmpdir(), "roboclaw-map-test-")));
  t.after(() => rm(base, { recursive: true, force: true }));
  const state = join(base, "state"), raw = syntheticV1Image();
  const first = await savePrivateV1Map(raw, state);
  assert.equal(first.sha256, createHash("sha256").update(raw).digest("hex"));
  assert.equal(first.bytes, raw.length); assert.equal(first.saved, true);
  const path = join(state, "maps", `v1-${first.sha256}.bin`);
  assert.deepEqual(await readFile(path), raw);
  assert.equal((await lstat(state)).mode & 0o777, 0o700);
  assert.equal((await lstat(join(state, "maps"))).mode & 0o777, 0o700);
  assert.equal((await lstat(path)).mode & 0o777, 0o600);
  assert.equal((await lstat(path)).nlink, 1);
  assert.equal((await savePrivateV1Map(raw, state)).saved, false);
  assert.deepEqual(await readdir(join(state, "maps")), [`v1-${first.sha256}.bin`]);
  await writeFile(path, Buffer.alloc(raw.length));
  await assert.rejects(savePrivateV1Map(raw, state), /Cannot save private V1 map/);
});

test("invalid integrity and symlinked state or destination fail closed", async t => {
  const base = await realpath(await mkdtemp(join(tmpdir(), "roboclaw-map-test-")));
  t.after(() => rm(base, { recursive: true, force: true }));
  const raw = syntheticV1Image(); const damaged = Buffer.from(raw); damaged[24] ^= 1;
  await assert.rejects(savePrivateV1Map(damaged, join(base, "invalid")), /Invalid private V1 map/);
  await assert.rejects(lstat(join(base, "invalid")), { code: "ENOENT" });
  await symlink(base, join(base, "linked"));
  await assert.rejects(savePrivateV1Map(raw, join(base, "linked")), /Cannot save private V1 map/);
  const state = join(base, "state"), hash = createHash("sha256").update(raw).digest("hex");
  await savePrivateV1Map(raw, state);
  const path = join(state, "maps", `v1-${hash}.bin`);
  await unlink(path);
  await symlink(join(base, "target"), path);
  await assert.rejects(savePrivateV1Map(raw, state), /Cannot save private V1 map/);
  await assert.rejects(lstat(join(base, "target")), { code: "ENOENT" });
});
