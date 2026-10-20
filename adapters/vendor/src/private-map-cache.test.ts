import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { lstat, mkdtemp, readFile, readdir, realpath, rm, symlink, unlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { syntheticV1Image } from "./map-raster-fixture.ts";
import { savePrivateV1Map } from "./private-map-cache.ts";

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
