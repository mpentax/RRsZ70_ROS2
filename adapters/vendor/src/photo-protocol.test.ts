import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { lstat, mkdtemp, readFile, readdir, realpath, rm, symlink, unlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { gzipSync } from "node:zlib";
import { test } from "node:test";
import { encodeV1MqttFrame } from "./cloud-control.ts";
import { localProtocolCodec } from "./local.ts";
import {
  type BlobSecurity,
  createBlobSecurity,
  decodeCloudMapResponse,
  decodeMapResponse,
  inspectMapFrame,
  MAX_CLOUD_MAP_FRAME_BYTES,
} from "./photo-protocol.ts";
import { mapMetadataFromRead, savePrivateV1Map, summarizeV1Map, syntheticV1Image, validatedMapMetadata } from "./private-map-cache.ts";
import { SecretStore } from "./secret-store.ts";

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
