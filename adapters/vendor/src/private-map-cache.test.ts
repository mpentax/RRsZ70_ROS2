import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { lstat, mkdtemp, readFile, readdir, realpath, rm, symlink, unlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { syntheticV1Image } from "./map-raster-fixture.ts";
import { mapMetadataFromRead, savePrivateV1Map, validatedMapMetadata } from "./private-map-cache.ts";
import { SecretStore } from "./secret-store.ts";

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
