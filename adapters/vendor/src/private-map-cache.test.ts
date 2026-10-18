import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { lstat, mkdtemp, readFile, readdir, realpath, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { savePrivateV1Map } from "./private-map-cache.ts";

function synthetic(): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = Buffer.alloc(25); image.writeUInt16LE(2); image.writeUInt16LE(24, 2);
  image.writeUInt32LE(1, 4); image.writeUInt32LE(1, 16); image.writeUInt32LE(1, 20); image[24] = 7;
  const digest = Buffer.alloc(28); digest.writeUInt16LE(1024); digest.writeUInt16LE(8, 2); digest.writeUInt32LE(20, 4);
  const raw = Buffer.concat([header, image, digest]);
  raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}

test("private cache is content-addressed, non-overwriting, restricted and idempotent", async t => {
  const base = await realpath(await mkdtemp(join(tmpdir(), "roboclaw-map-test-")));
  t.after(async () => { const { rm } = await import("node:fs/promises"); await rm(base, { recursive: true, force: true }); });
  const state = join(base, "state"), raw = synthetic();
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
  t.after(async () => { const { rm } = await import("node:fs/promises"); await rm(base, { recursive: true, force: true }); });
  const raw = synthetic(); const damaged = Buffer.from(raw); damaged[24] ^= 1;
  await assert.rejects(savePrivateV1Map(damaged, join(base, "invalid")), /Invalid private V1 map/);
  await assert.rejects(lstat(join(base, "invalid")), { code: "ENOENT" });
  await symlink(base, join(base, "linked"));
  await assert.rejects(savePrivateV1Map(raw, join(base, "linked")), /Cannot save private V1 map/);
  const state = join(base, "state"), hash = createHash("sha256").update(raw).digest("hex");
  await savePrivateV1Map(raw, state);
  const path = join(state, "maps", `v1-${hash}.bin`);
  const { unlink } = await import("node:fs/promises"); await unlink(path);
  await symlink(join(base, "target"), path);
  await assert.rejects(savePrivateV1Map(raw, state), /Cannot save private V1 map/);
  await assert.rejects(lstat(join(base, "target")), { code: "ENOENT" });
});
